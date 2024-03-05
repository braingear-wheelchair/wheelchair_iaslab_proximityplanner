#ifndef WHEELCHAIR_IASLAB_PROXIMITYPLANNER_H_
#define WHEELCHAIR_IASLAB_PROXIMITYPLANNER_H_

// abstract class from which our plugin inherits
#include <base_local_planner/costmap_model.h>
// #include <base_local_planner/odometry_helper_ros.h>
#include <mbf_costmap_core/costmap_controller.h>
#include <nav_core/base_local_planner.h>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

// costmap converter libraries
#include <costmap_converter/ObstacleMsg.h>
#include <costmap_converter/costmap_converter_interface.h>

// Proximity grid msg
#include "proximity_grid/ProximityGridMsg.h"

// teb_local_planner related classes
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/pose_se2.h>
#include <teb_local_planner/robot_footprint_model.h>

#include <cmath>
#include <limits>
#include <vector>

using namespace std;

namespace wheelchair_iaslab_proximityplanner{

/**
 * @class ProximityPlanner
 * @brief Implements both nav_core::BaseLocalPlanner that convert the local costamat into 
 * a proximity grid that need to be passed to the shared_intelligence package to work with the pfs
 * This is a REACTIVE approach
 * @todo Directly integrate the pfs
 **/
class ProximityPlanner : public nav_core::BaseLocalPlanner
{
    using ObstContainer    = teb_local_planner::ObstContainer;
    
    using ObstaclePtr      = teb_local_planner::ObstaclePtr;
    using PointObstacle    = teb_local_planner::PointObstacle;
    using CircularObstacle = teb_local_planner::CircularObstacle;
    using LineObstacle     = teb_local_planner::LineObstacle;
    using PolygonObstacle  = teb_local_planner::PolygonObstacle;
    
public:

    ProximityPlanner();
    ~ProximityPlanner();

    void initialize(std::string name, tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    bool isGoalReached();
    
protected:
    void updateObstacleContainerWithCostmapConverter();
    void updateObstacleContainerWithCostmap();

    double getDisstancePoints(geometry_msgs::Point p1, geometry_msgs::Point p2 );

    void updateCurrentPosition();
    void samplePlan();

    double getDistanceFromOdometry(geometry_msgs::Point point);

    bool configure_proxymity_msg(ros::NodeHandle nh);
    void update_proximity_msg();

    double getAngle(int pos_x, int pos_y, int center_x, int center_y, double resolution);

    ros::Publisher proximity_pub_;

private:
    costmap_2d::Costmap2DROS*  costmap_ros_;
    costmap_2d::Costmap2D*     _costmap;         //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)
    geometry_msgs::Point       _current_position;
    tf2_ros::Buffer*           tf_;

    double _min_cost;

    proximity_grid::ProximityGridMsg proxymity_msg_;

    bool initialized_ = false;

    // The costmap converter plugin setting
    pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> _costmap_converter_loader;  //!< Load costmap converter plugins at runtime
    boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> _costmap_converter;              //!< Store the current costmap_converter
    costmap_converter::ObstacleArrayMsg _custom_obstacle_msg;                                    //!< Copy of the most recent obstacle message

    ObstContainer _obstacles;  //!< Obstacle vector that should be considered during local trajectory optimization

    struct GlobalPlan // Store the information of the plan
    {
        std::vector<geometry_msgs::PoseStamped> global_plan;  //!< Store the current global plan
        std::vector<geometry_msgs::PoseStamped> sampled_global_plan; //!< Sample in the distance the global plan
        int current_index = -1;
    } _global_plan;

    
    struct Parameters
    {
        std::string odom_topic = "/odom";
        //std::string mpc_topic  = "/mpc_command";

        double arrival_distance = 0.2;  // [m] the distance from the goal that when it is assumed to be reached
        int sampling_distance = -1;  // the subsapling distance in the array, put -1 to give direct command to mpc 
    } _params;


    struct CostmapConverterPlugin
    {
        std::string costmap_converter_plugin;
        double costmap_converter_rate      = 5;
        bool costmap_converter_spin_thread = true;
        
    } _costmap_conv_params;

};

}; // end namespace wheelchair_iaslab_proximityplanner

#endif // WHEELCHAIR_IASLAB_PROXIMITYPLANNER_H_
