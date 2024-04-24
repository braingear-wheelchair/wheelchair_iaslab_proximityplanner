#include "wheelchair_iaslab_proximityplanner/wheelchair_iaslab_proximityplanner.h"
#include <pluginlib/class_list_macros.h>

// using ofstream constructors.
#include <iostream>
#include <fstream>  

PLUGINLIB_EXPORT_CLASS(wheelchair_iaslab_proximityplanner::ProximityPlanner, nav_core::BaseLocalPlanner)

namespace wheelchair_iaslab_proximityplanner{

ProximityPlanner::ProximityPlanner() : //costmap_.costmap_ros(NULL),
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
        this->costmap_.costmap_ros = costmap_ros;
        this->costmap_.costmap     = costmap_ros->getCostmap();  // locking should be done in MoveBase.

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

        ROS_INFO("The current configuration is the following: \n repellor_angular_strength: \
            %f, repellor_angular_decay: %f, vel_linear_min: %f, vel_linear_max: %f,\
            vel_angular_min: %f, vel_angular_max: %f", 
            this->repellor_angular_strength_, this->repellor_angular_decay_, this->vel_linear_min_, 
            this->vel_linear_max_, this->vel_angular_min_, this->vel_angular_max_);

        // Load the parameters for the visual range
        nh.param<float>("angle_min", this->visual_range_.angle_min,  -M_PI);
        nh.param<float>("angle_max", this->visual_range_.angle_max,   M_PI);
        nh.param<float>("angle_inc", this->visual_range_.delta_angle, M_PI / 180.0f);

        // load the vertices of the vertices of the rectangle
        nh.param<std::vector<float>>("rel_verts_x", this->rel_verts_x_, this->rel_verts_x_);
        nh.param<std::vector<float>>("rel_verts_y", this->rel_verts_y_, this->rel_verts_y_);

        for (int i = 0; i < rel_verts_x_.size(); i++) {
            ROS_INFO("rel_verts_x[%d]: %f, rel_verts_y[%d]: %f", i, rel_verts_x_[i], i, rel_verts_y_[i]);
        }

        // Now convert the index as a polar coordinate and add the list of the Forces
        for (int i = 0; i < rel_verts_x_.size(); i++) {
            rel_verts_d_.push_back(sqrt(pow(rel_verts_x_[i], 2) + pow(rel_verts_y_[i], 2)));
            rel_verts_theta_.push_back(atan2(rel_verts_y_[i], rel_verts_x_[i]));

            reppellors_list_.push_back(std::list<Force>());
        }

        this->raw_repellors_ = std::list<Force>();
        this->tmp_repellors_ = std::list<Force>();

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
    // As initial step clear the data
    this->force_repellors_.intensity = 0.0f;
    this->force_repellors_.theta = 0.0f;
    int index_vertex = 0;

    this->cleanRawRepellors();

    //for (index_vertex = 0; index_vertex < this->reppellors_list_.size(); index_vertex++) {
    //    this->reppellors_list_[index_vertex].clear();
    //}
    //this->raw_repellors_.clear();
    //this->tmp_repellors_.clear();

    // First update the internal map
    this->updateInternalMap();

    // Then look for each point in the map if could be an obstacle and attach to the lists
    this->updateRawRepellors();

    // Then clean the lists if the points are to close to each other or are under a closer obstacle
    this->cleanRawRepellors();

    // Convert the list of distance to forces

    // As a final step collapse the list to a single force
}

void ProximityPlanner::cleanRawRepellors() {
    // For each vertex initilize the list of repellors to inifinity
    
    int index_vertex = 0;
    int current_angle = 0;

    for (index_vertex = 0; index_vertex < this->reppellors_list_.size(); index_vertex++) {
        this->reppellors_list_[index_vertex].clear();

        current_angle = this->visual_range_.angle_min;

        while (current_angle < this->visual_range_.angle_max) {
            this->reppellors_list_[index_vertex].push_back(Force(INFINITY, current_angle));
            current_angle += this->visual_range_.delta_angle;
        }
    }


    // There should be better way to do this but the idea is the following
    // First randomize the list, than for each point look if is the nearest 
    // from the center point in a radius defined in the angle_min_
    
    /*std::random_device rd;
    std::mt19937 generator(rd());
    std::shuffle(v.begin(), v.end(), generator);*/

    /*

    std::list<Force>::iterator it, nit;
    Force tmp_force = Force(0.0f, 0.0f);

    ROS_INFO("Cleaning repellors");

    try {

    for (it = this->raw_repellors_.begin(); it != this->raw_repellors_.end(); it++) {
        ROS_INFO("loooool");

        tmp_force = *it;
        for (nit = this->raw_repellors_.begin(); nit != this->raw_repellors_.end(); nit++) {
            ROS_INFO("loooool 2, theta = %f", tmp_force.theta);

            // TODO: better implement this, for now is to slow


            if ( tmp_force.theta > this->normalizeAngle(nit->theta - this->visual_range_.delta_angle) && \
                 tmp_force.theta < this->normalizeAngle(nit->theta + this->visual_range_.delta_angle) ) {
                // This is the case that the point is in the sector
                // Here intesity is the distance from the center
                if (tmp_force.intensity < nit->intensity) {
                    tmp_force.intensity = nit->intensity;
                }
            }
        }

        // Before pushing the force into the other list, check if is not already present
        bool found = false;
        for (nit = this->tmp_repellors_.begin(); nit != this->tmp_repellors_.end(); nit++) {
            ROS_INFO("loooool 3");
            if (tmp_force.theta > this->normalizeAngle(nit->theta - this->visual_range_.delta_angle) && \
                tmp_force.theta < this->normalizeAngle(nit->theta + this->visual_range_.delta_angle) ) {
                found = true;
            }
        }
        if (!found) {
          this->tmp_repellors_.push_back(tmp_force);
          ROS_INFO("-----------------------------------------------------------loooool 4");
        } else {
          ROS_INFO("***********************************************************loooool 5");
        }
    }

    } catch (std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
    }

    ROS_INFO("Repellors cleaned");

    */

}

void ProximityPlanner::addRawPoint(Force force) {
    bool found = false;

    std::list<Force>::iterator it;
    for (it = this->raw_repellors_.begin(); it != this->raw_repellors_.end(); it++) {
        if (force.theta > this->normalizeAngle(it->theta - this->visual_range_.delta_angle) && \
            force.theta < this->normalizeAngle(it->theta + this->visual_range_.delta_angle) ) {
            found = true;
            it->intensity = std::min(it->intensity, force.intensity);
        }
    }
    if (!found)
      this->raw_repellors_.push_back(force);
}

void ProximityPlanner::updateRawRepellors() {

    float current_angle    = 0.0f;
    float current_distance = 0.0f;
    float current_cost     = 0.0f;

    for (int indx_x = 0; indx_x < this->costmap_.length_x; indx_x++) {
        for(int indx_y = 0; indx_y < this->costmap_.length_y; indx_y++) {

            current_cost = this->costmap_.costmap->getCost(indx_x, indx_y);

            if (current_cost > _min_cost) {
                // The point need to be considered as a possible raw point

                current_angle = getAngle(indx_x, indx_y, this->costmap_.center_x, \
                                         this->costmap_.center_y, this->costmap_.resolution);

                // This is the euclidean distance to the center
                current_distance  = (indx_x - this->costmap_.center_x) * (indx_x - this->costmap_.center_x);
                current_distance += (indx_y - this->costmap_.center_y) * (indx_y - this->costmap_.center_y);
                current_distance  = sqrt(current_distance) * this->costmap_.resolution;

                // Using force since it is a simple vector in polar coordinate
                this->addRawPoint(Force(current_distance, current_angle));
            }
        }
    }
}

void ProximityPlanner::updateInternalMap() {
    // First request the updated map
    this->costmap_.costmap_ros->updateMap();
    this->costmap_.costmap = this->costmap_.costmap_ros->getCostmap();

    this->costmap_.length_x = this->costmap_.costmap->getSizeInCellsX();
    this->costmap_.length_y = this->costmap_.costmap->getSizeInCellsY();
    this->costmap_.center_x = this->costmap_.length_x / 2;
    this->costmap_.center_y = this->costmap_.length_y / 2;

    this->costmap_.resolution = this->costmap_.costmap->getResolution();
}

void ProximityPlanner::computeTwist(geometry_msgs::Twist& cmd_vel) {
    cmd_vel.linear.x  = this->final_force_.intensity;
    cmd_vel.angular.z = this->final_force_.theta;

    cmd_vel.linear.x  = this->normalizeVelocity(cmd_vel.linear.x,  this->vel_linear_min_,  this->vel_linear_max_);
    cmd_vel.angular.z = this->normalizeVelocity(cmd_vel.angular.z, this->vel_angular_min_, this->vel_angular_max_);
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

float ProximityPlanner::normalizeVelocity(float vel_a, float v_min, float v_max) {
    // More than a "normalize" this will put boundaries on the velocity 
    int current_sing(sgn(vel_a));

    vel_a = std::abs(vel_a);

    if (vel_a < v_min) {
        vel_a = v_min;
    }else if(vel_a > v_max) {
        vel_a = v_max;
    }

    return vel_a * ((float) current_sing);
}

void ProximityPlanner::updateCurrentPosition()
{
    geometry_msgs::PoseStamped robot_pose;
    this->costmap_.costmap_ros->getRobotPose(robot_pose);
    
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

    // Normalize and return the angle
    return this->normalizeAngle(angle); 
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


