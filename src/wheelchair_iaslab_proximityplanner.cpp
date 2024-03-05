#include "wheelchair_iaslab_proximityplanner/wheelchair_iaslab_proximityplanner.h"
#include <pluginlib/class_list_macros.h>

// using ofstream constructors.
#include <iostream>
#include <fstream>  

PLUGINLIB_EXPORT_CLASS(wheelchair_iaslab_proximityplanner::ProximityPlanner, nav_core::BaseLocalPlanner)

namespace wheelchair_iaslab_proximityplanner{

ProximityPlanner::ProximityPlanner() : costmap_ros_(NULL),
                               _costmap_converter_loader("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
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
        // costmap converter plugin related parameters
        nh.param("costmap_converter_plugin", _costmap_conv_params.costmap_converter_plugin, _costmap_conv_params.costmap_converter_plugin);
        nh.param("costmap_converter_rate",   _costmap_conv_params.costmap_converter_rate,   _costmap_conv_params.costmap_converter_rate);
        nh.param("costmap_converter_spin_thread", _costmap_conv_params.costmap_converter_spin_thread,
                 _costmap_conv_params.costmap_converter_spin_thread);

        nh.param<double>("min_cost", this->_min_cost, 0.0d);
        // init costmap variables
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        _costmap     = costmap_ros_->getCostmap();  // locking should be done in MoveBase.

        // special parameters
        nh.param("odom_topic", _params.odom_topic, _params.odom_topic);
        
        // initialize the costmap to polygon converter
        if (!_costmap_conv_params.costmap_converter_plugin.empty()){
            try{
                _costmap_converter         = _costmap_converter_loader.createInstance(_costmap_conv_params.costmap_converter_plugin);
                std::string converter_name = _costmap_converter_loader.getName(_costmap_conv_params.costmap_converter_plugin);
                // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
                boost::replace_all(converter_name, "::", "/");
                _costmap_converter->setOdomTopic(_params.odom_topic); 
                _costmap_converter->initialize(ros::NodeHandle(nh, "costmap_converter/" + converter_name));
                _costmap_converter->setCostmap2D(_costmap);
                _costmap_converter->startWorker(ros::Rate(_costmap_conv_params.costmap_converter_rate), _costmap,
                                                _costmap_conv_params.costmap_converter_spin_thread);
                ROS_INFO_STREAM("Costmap conversion plugin " << _costmap_conv_params.costmap_converter_plugin << " loaded.");
            }catch (pluginlib::PluginlibException& ex){
                ROS_WARN(
                    "The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error "
                    "message: %s", ex.what());
                _costmap_converter.reset();
            }
        }else
            ROS_INFO("No costmap conversion plugin specified.");

        if(!configure_proxymity_msg(nh))
            ROS_ERROR("COnfiguration of proxymity message fallied");

        // Create the publisher for the proximity gird
        this->proximity_pub_ = nh.advertise<proximity_grid::ProximityGridMsg>("repellors", 1);
        ROS_INFO("the string is %s.", this->proxymity_msg_.header.frame_id.c_str());

        initialized_ = true;
        ROS_DEBUG("proxymity_local_planner plugin initialized.");
    }
}

bool ProximityPlanner::configure_proxymity_msg(ros::NodeHandle nh) {
    // TODO add a checker if the paramethers here
    this->proxymity_msg_ = proximity_grid::ProximityGridMsg();

    nh.param<std::string>("frame_id", this->proxymity_msg_.header.frame_id, "base_link");
    nh.param<float>("angle_min",      this->proxymity_msg_.angle_min, -120.0f);
    nh.param<float>("angle_max",      this->proxymity_msg_.angle_max,  120.0f);
    nh.param<float>("angle_inc",      this->proxymity_msg_.angle_increment,  9.0f);
    nh.param<float>("range_min",      this->proxymity_msg_.range_min, 0.0f);
    nh.param<float>("range_max",      this->proxymity_msg_.range_max, 6.0f);

    return true;
}

void ProximityPlanner::update_proximity_msg() {
    this->proxymity_msg_.header.stamp	= ros::Time::now();
    // TODO update the ranges
    //  msg.ranges			= grid.GetGrid();
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

    // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
    if (_costmap_converter)
        updateObstacleContainerWithCostmapConverter();
    else
        updateObstacleContainerWithCostmap();

    updateCurrentPosition();

    if (_global_plan.current_index < _global_plan.sampled_global_plan.size() )
    {
        // Check the distance from the current subgoal and if its ok update the mpc goal
        geometry_msgs::Point current_goal = _global_plan.sampled_global_plan[_global_plan.current_index].pose.position;
        double distance = getDistanceFromOdometry(current_goal);

        // ROS_INFO("x=%f y=%f",current_goal.x,current_goal.y);

        if(distance < _params.arrival_distance)
        {
            _global_plan.current_index++;
            //_pub_set_next_point.publish(_global_plan.sampled_global_plan[_global_plan.current_index].pose);
        }
    }

    // Update and publish the proxymity grid for the repellors
    this->update_proximity_msg();
    proximity_pub_.publish(this->proxymity_msg_);
    
    return true;
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
    
    // ROS_INFO("current distance from the objective =%f", distance);

    if(distance < _params.arrival_distance)
        return true;
    
    return false;
}

double ProximityPlanner::getDisstancePoints(geometry_msgs::Point p1, geometry_msgs::Point p2 )
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

void ProximityPlanner::updateObstacleContainerWithCostmapConverter()
{
    if (!_costmap_converter) return;

    // Get obstacles from costmap converter
    costmap_converter::ObstacleArrayConstPtr obstacles = _costmap_converter->getObstacles();
    if (!obstacles) return;

    std::vector<geometry_msgs::Pose> points = {};
    points.resize( obstacles->obstacles.size() );

    int index = 0;

    for (std::size_t i = 0; i < obstacles->obstacles.size(); ++i)
    {
        const costmap_converter::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
        const geometry_msgs::Polygon* polygon          = &obstacle->polygon;

        geometry_msgs::Pose errorpoint = geometry_msgs::Pose();
        errorpoint.position.x = polygon->points[0].x;
        errorpoint.position.y = polygon->points[0].y;

        // TODO: put the 2 to be a configuration parameter
        if(ProximityPlanner::getDisstancePoints(_current_position, errorpoint.position) < 3 )
        {
            points[index] = errorpoint;
            index++;
        }

    }

    points.resize( index );
    
}

void ProximityPlanner::updateObstacleContainerWithCostmap() {
    this->costmap_ros_->updateMap();
    this->_costmap = costmap_ros_->getCostmap();

    // Get the resolution of the costmap
    double resolution = this->_costmap->getResolution();
    // Get the length of the costmap in x and y
    unsigned int length_x = this->_costmap->getSizeInCellsX();
    unsigned int length_y = this->_costmap->getSizeInCellsY();

    // From the center of the map evaluate the nearest obstacle for each direction
    int num_iteration = (int) ((this->proxymity_msg_.angle_max - this->proxymity_msg_.angle_min) / this->proxymity_msg_.angle_increment);

    std::vector<float> ranges;
    std::vector<float> costs;

    // Initialize the ranges to infinity 
    ranges.reserve(num_iteration);
    ranges.assign(num_iteration, std::numeric_limits<float>::infinity());

    costs.reserve(num_iteration);
    costs.assign(num_iteration, 0);

    // Now we evaluate all the cell of the map and get the highest cost for each direction
    // Starting from the center of the map
    int center_x = length_x / 2;
    int center_y = length_y / 2;

    for (int idx_x = 0; idx_x < length_x; idx_x++) {
        for (int idx_y = 0; idx_y < length_y; idx_y++) {

            double current_angle = this->getAngle(idx_x, idx_y, center_x, center_y, resolution);

            // Check if the current angle is in the range
            if (current_angle < this->proxymity_msg_.angle_max && current_angle > this->proxymity_msg_.angle_min) {

                float cost = this->_costmap->getCost(idx_x, idx_y);

                if (cost > this->_min_cost) {

                    int current_index = 0;

                    while (current_angle > this->proxymity_msg_.angle_min + current_index * this->proxymity_msg_.angle_increment) {
                        current_index++;
                    }

                    if(cost > costs[current_index]) {
                        costs[current_index] = cost;

                        double distance = (idx_x - center_x) * (idx_x - center_x) + (idx_y - center_y) * (idx_y - center_y);
                        distance = std::sqrt(distance) * resolution;

                        if(distance < this->proxymity_msg_.range_max && distance > this->proxymity_msg_.range_min) 
                            ranges[current_index] = distance;
                    }
                }
            }
        }
    } 

    // Now save the ranges into the proximity_msgs range
    this->proxymity_msg_.ranges = ranges;

    return;
}

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

}
