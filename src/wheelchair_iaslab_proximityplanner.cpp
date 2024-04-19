#include "wheelchair_iaslab_proximityplanner/wheelchair_iaslab_proximityplanner.h"
#include <pluginlib/class_list_macros.h>

// using ofstream constructors.
#include <iostream>
#include <fstream>  

PLUGINLIB_EXPORT_CLASS(wheelchair_iaslab_proximityplanner::ProximityPlanner, nav_core::BaseLocalPlanner)

namespace wheelchair_iaslab_proximityplanner{

ProximityPlanner::ProximityPlanner() : costmap_ros_(NULL),
                               //_costmap_converter_loader("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
                               tf_(NULL),
                               initialized_(false)
{
    ROS_INFO("Building proximity local planner");
}

ProximityPlanner::~ProximityPlanner() {}

// Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
void ProximityPlanner::initialize(std::string name, tf2_ros::Buffer* tf,
                              costmap_2d::Costmap2DROS* costmap_ros)
{
    ROS_INFO("Initilizing my local planner");
    if(!initialized_)
    {
        // create Node Handle with name of plugin (as used in move_base for loading)
        ros::NodeHandle nh("~/" + name);
        ROS_INFO("My name is %s",name.c_str());

        nh.param<double>("min_cost", this->_min_cost, 0.0d);
        // init costmap variables
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        _costmap     = costmap_ros_->getCostmap();  // locking should be done in MoveBase.

        // special parameters
        nh.param("odom_topic", _params.odom_topic, _params.odom_topic);

        nh.param<double>("arrival_distance", this->_params.arrival_distance, 0.5f);

        // velocity parameters
        nh.param<float>("repellor_angular_strength", this->repellor_angular_strength_, 0.1f);
        nh.param<float>("repellor_angular_decay", this->repellor_angular_decay_, 0.1f);
        nh.param<float>("vel_linear_min", this->vel_linear_min_, 0.0f);
        nh.param<float>("vel_linear_max", this->vel_linear_max_, 1.0f);
        nh.param<float>("vel_angular_min", this->vel_angular_min_, 0.0f);
        nh.param<float>("vel_angular_max", this->vel_angular_max_, 1.0f);

        ROS_INFO("The current configuration is the following: \n repellor_angular_strength: %f, repellor_angular_decay: %f, vel_linear_min: %f, vel_linear_max: %f, vel_angular_min: %f, vel_angular_max: %f", 
            this->repellor_angular_strength_, this->repellor_angular_decay_, this->vel_linear_min_, 
            this->vel_linear_max_, this->vel_angular_min_, this->vel_angular_max_);

        // load the vertices of the vertices of the rectangle
        nh.param<std::vector<float>>("rel_verts_x", this->rel_verts_x_, this->rel_verts_x_);
        nh.param<std::vector<float>>("rel_verts_y", this->rel_verts_y_, this->rel_verts_y_);

        for (int i = 0; i < rel_verts_x_.size(); i++)
        {
            ROS_INFO("rel_verts_x[%d]: %f, rel_verts_y[%d]: %f", i, rel_verts_x_[i], i, rel_verts_y_[i]);
        }

        // load the range parametes
        nh.param<float>("range_min", this->range_min_, 0.0f);
        nh.param<float>("range_max", this->range_max_, 6.0f);

        // Init the force vectos
        this->force_repellors_  = Force(0.0f, 0.0f);
        this->force_attractors_ = Force(0.0f, 0.0f);

        ROS_INFO("%f", this->force_repellors_.intensity);

        initialized_ = true;
        ROS_DEBUG("proxymity_local_planner plugin initialized.");
    }
}

void ProximityPlanner::samplePlan()
{
    _global_plan.sampled_global_plan = {};
    
    if (_params.sampling_distance > 0)
    {
        for(int i = 0; i < (int)_global_plan.global_plan.size(); i += _params.sampling_distance){
            _global_plan.sampled_global_plan.push_back(_global_plan.global_plan[i]);
        }
    }

    _global_plan.sampled_global_plan.push_back(_global_plan.global_plan.back());
    _global_plan.current_index = 0;

}


bool ProximityPlanner::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& orig_global_plan
)
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    // Store the global plan
    _global_plan.global_plan = orig_global_plan;
    samplePlan();

    return true;
}

bool ProximityPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }

    updateCurrentPosition();

    if (_global_plan.current_index < _global_plan.sampled_global_plan.size() )
    {
        // Check the distance from the current subgoal and if its ok update the mpc goal
        geometry_msgs::Point current_goal = _global_plan.sampled_global_plan[_global_plan.current_index].pose.position;
        double distance = getDistanceFromOdometry(current_goal);

        if(distance < _params.arrival_distance)
        {
            _global_plan.current_index++;
        }
    }

    this->resetForces();
    this->computeRepellorForce();
    this->computeAttractorForce();
    this->computeTotalForce();
    this->computeTwist(cmd_vel);
    
    //this->publishSideInformation();

    return true;
}

void ProximityPlanner::resetForces()
{
    this->force_repellors_.intensity = 0.0f;
    this->force_repellors_.theta = 0.0f;
    this->force_attractors_.intensity = 0.0f;
    this->force_attractors_.theta = 0.0f;
    this->final_force_.intensity = 0.0f;
    this->final_force_.theta = 0.0f;
}

void ProximityPlanner::computeTotalForce() {
    this->final_force_ = this->force_repellors_ + this->force_attractors_;
    this->final_force_.theta = normalizeAngle(this->final_force_.theta);
}

void ProximityPlanner::computeAttractorForce() {

    current_objective = &_global_plan.sampled_global_plan[_global_plan.current_index].pose.position;

    this->force_attractors_.intensity = 1.0f / getDistanceFromOdometry(*current_objective);
    this->force_attractors_.theta = std::atan2(current_objective->y - _current_position.y, current_objective->x - _current_position.x);

    this->force_attractors_.theta = this->force_attractors_.theta - _current_position.z;
    this->force_attractors_.theta = normalizeAngle(this->force_attractors_.theta);

}

void ProximityPlanner::computeRepellorForce() {
    // TODO: complete this function
    this->force_repellors_.intensity = 0.0f;
    this->force_repellors_.theta = 0.0f;
}

void ProximityPlanner::computeTwist(geometry_msgs::Twist& cmd_vel) {
    cmd_vel.linear.x  = this->final_force_.intensity;
    cmd_vel.angular.z = this->final_force_.theta;
}

void ProximityPlanner::updateCurrentPosition()
{
    geometry_msgs::PoseStamped robot_pose;
    costmap_ros_->getRobotPose(robot_pose);
    
    _current_position = robot_pose.pose.position;
    _current_position.z = tf::getYaw(robot_pose.pose.orientation);
}

double ProximityPlanner::getDistanceFromOdometry(geometry_msgs::Point point)
{
    
    // Calucate the euclidee distance between the two points
    double distance = pow(_current_position.x-point.x,2) + pow(_current_position.y-point.y,2);  
    distance = pow(distance, 0.5);

    return distance;
}

    
bool ProximityPlanner::isGoalReached()
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }

    updateCurrentPosition();
    geometry_msgs::Point goal = _global_plan.global_plan.back().pose.position;
    double distance = getDistanceFromOdometry(goal);
    
    if(distance < _params.arrival_distance)
        return true;
    
    return false;
}

double ProximityPlanner::getDisstancePoints(geometry_msgs::Point p1, geometry_msgs::Point p2 )
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

//void ProximityPlanner::updateObstacleContainerWithCostmapConverter()
//{
//    if (!_costmap_converter) return;
//
//    // Get obstacles from costmap converter
//    costmap_converter::ObstacleArrayConstPtr obstacles = _costmap_converter->getObstacles();
//    if (!obstacles) return;
//
//    std::vector<geometry_msgs::Pose> points = {};
//    points.resize( obstacles->obstacles.size() );
//
//    int index = 0;
//
//    for (std::size_t i = 0; i < obstacles->obstacles.size(); ++i)
//    {
//        const costmap_converter::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
//        const geometry_msgs::Polygon* polygon          = &obstacle->polygon;
//
//        geometry_msgs::Pose errorpoint = geometry_msgs::Pose();
//        errorpoint.position.x = polygon->points[0].x;
//        errorpoint.position.y = polygon->points[0].y;
//
//        // TODO: put the 2 to be a configuration parameter
//        if(ProximityPlanner::getDisstancePoints(_current_position, errorpoint.position) < 3 )
//        {
//            points[index] = errorpoint;
//            index++;
//        }
//
//    }
//
//    points.resize( index );
//    
//}
//
//void ProximityPlanner::updateObstacleContainerWithCostmap() {
//    this->costmap_ros_->updateMap();
//    this->_costmap = costmap_ros_->getCostmap();
//
//    // Get the resolution of the costmap
//    double resolution = this->_costmap->getResolution();// / 2.0f;
//    // Get the length of the costmap in x and y
//    unsigned int length_x = this->_costmap->getSizeInCellsX();
//    unsigned int length_y = this->_costmap->getSizeInCellsY();
//
//    // From the center of the map evaluate the nearest obstacle for each direction
//    int num_iteration = (int) ((this->proxymity_msg_.angle_max - this->proxymity_msg_.angle_min) / this->proxymity_msg_.angle_increment);
//
//    std::vector<float> ranges;
//    std::vector<float> costs;
//
//    // Initialize the ranges to infinity 
//    ranges.reserve(num_iteration);
//    ranges.assign(num_iteration, std::numeric_limits<float>::infinity());
//
//    costs.reserve(num_iteration);
//    costs.assign(num_iteration, 0);
//
//    // Now we evaluate all the cell of the map and get the highest cost for each direction
//    // Starting from the center of the map
//    int center_x = length_x / 2;
//    int center_y = length_y / 2;
//
//    for (int idx_x = 0; idx_x < length_x; idx_x++) {
//        for (int idx_y = 0; idx_y < length_y; idx_y++) {
//
//            double current_angle = this->getAngle(idx_x, idx_y, center_x, center_y, resolution);
//
//            // Check if the current angle is in the range
//            if (current_angle < this->proxymity_msg_.angle_max && current_angle > this->proxymity_msg_.angle_min) {
//
//                float cost = this->_costmap->getCost(idx_x, idx_y);
//
//                if (cost > this->_min_cost) {
//
//                    int current_index = 0;
//
//                    while (current_angle > this->proxymity_msg_.angle_min + current_index * this->proxymity_msg_.angle_increment) {
//                        current_index++;
//                    }
//
//                    current_index--; // This should fix a small bug on the last iteration
//
//                    if(cost > costs[current_index]) {
//                        costs[current_index] = cost;
//
//                        double distance = (idx_x - center_x) * (idx_x - center_x) + (idx_y - center_y) * (idx_y - center_y);
//                        distance = std::sqrt(distance) * (resolution);
//
//                        if(distance < this->proxymity_msg_.range_max && distance > this->proxymity_msg_.range_min) 
//                            ranges[current_index] = distance;
//                    }
//                }
//            }
//        }
//    } 
//
//    // Now save the ranges into the proximity_msgs range
//    this->proxymity_msg_.ranges = ranges;
//
//    return;
//}

double ProximityPlanner::getAngle(int pos_x, int pos_y, int center_x, int center_y, double resolution) {
    double angle = 0.0;

    // Calculate the angle
    angle = atan2((center_y - pos_y) * resolution, (center_x - pos_x) * resolution) - this->_current_position.z;
    //angle = atan2((center_x - pos_x) * resolution, (center_y - pos_y) * resolution) - this->_current_position.z;

    angle -= M_PI;

    // Normalize the angle
    if (angle < -M_PI) {
        angle = 2 * M_PI + angle;
    }else if (angle > M_PI) {
        angle = -2 * M_PI - angle;
    }

    return angle; 
}


float ProximityPlanner::normalizeAngle(float angle) {
    if (angle < -M_PI) {
        angle = 2 * M_PI + angle;
    }else if (angle > M_PI) {
        angle = -2 * M_PI - angle;
    }
    return angle;
}

}


