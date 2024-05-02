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

#include <cmath>
#include <limits>
#include <vector>
#include <list>
#include <numeric>

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
public:

    ProximityPlanner();
    ~ProximityPlanner();

    void initialize(std::string name, tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    bool isGoalReached();

    float normalizeAngle(float angle);
    
protected:
    double getDistancePoints(geometry_msgs::Point p1, geometry_msgs::Point p2 );

    void updateCurrentPosition();
    void samplePlan();

    double getDistanceFromOdometry(geometry_msgs::Point point);

    bool configure_proxymity_msg(ros::NodeHandle nh);
    void update_proximity_msg();

    double getAngle(int pos_x, int pos_y, int center_x, int center_y, double resolution);

    ros::Publisher proximity_pub_;

private:
    void resetForces();
    void computeRepellorForce();
    void computeAttractorForce();
    void computeTotalForce();
    void computeTwist(geometry_msgs::Twist& cmd_vel); 
    void publishSideInformation();

    // Utilities
    float normalizeVelocity(float vel_a, float v_min, float v_max);
    void updateInternalMap();
    void updateRawRepellors();
    void cleanRawRepellors();
    void convertRawRepellorsToForces();
    void collapseRepellorsList();
    void updateGlobalPlanIndex();
    float computeHangle(float thata_1, float thata_2);
    float convertToPfs(float distance, float safe_distance);
    float getVlin();

private:

    ros::Publisher partial_angle_pub_;

    struct maps_ptrs {
        costmap_2d::Costmap2DROS*  costmap_ros;
        costmap_2d::Costmap2D*     costmap; // Pointer to the 2d costmap (obtained from the costmap ros wrapper)
        unsigned int length_x, length_y;
        unsigned int center_x, center_y;
        double resolution;
    } costmap_;

    geometry_msgs::Point       _current_position;
    tf2_ros::Buffer*           tf_;

    double _min_cost;

    bool initialized_ = false;

    struct GlobalPlan // Store the information of the plan
    {
        std::vector<geometry_msgs::PoseStamped> global_plan;  //!< Store the current global plan
        std::vector<geometry_msgs::PoseStamped> sampled_global_plan; //!< Sample in the distance the global plan
        int current_index = -1;
        int prev_index = 0;
    } _global_plan;

    geometry_msgs::Point* current_objective;

    
    struct Parameters
    {
        std::string odom_topic = "/odom";

        double arrival_distance;  // [m] the distance from the goal that when it is assumed to be reached
        double sampling_distance; // [m]the subsapling distance in the array 
    } _params;

    // Parameters for compute the fields
    float vel_angular_min_;
    float vel_angular_max_;
    float vel_linear_min_;
    float vel_linear_max_;

    float range_min_;
    float range_max_;

    struct VisualRange {
        float angle_min;
        float angle_max;
        float delta_angle;
        float safe_distance;
    } visual_range_;

    float repellor_angular_strength_;
    float repellor_angular_decay_;

    std::vector<float> rel_verts_x_;
    std::vector<float> rel_verts_y_;

    std::vector<float> rel_verts_d_;
    std::vector<float> rel_verts_theta_;

    // vector for the forces composed as [intensity, theta]
    struct Force {
        float intensity;
        float theta;

        Force(float intensity = 0.0f, float theta = 0.0f) : intensity(intensity), theta(theta) {}

        Force operator+ (const Force& other) const {
            std::vector<float> xs = {
                this->intensity * std::cos(this->theta),
                other.intensity * std::cos(other.theta)
            };
            std::vector<float> ys = {
                this->intensity * std::sin(this->theta),
                other.intensity * std::sin(other.theta)
            };

            float new_x = std::accumulate(xs.begin(), xs.end(), 0.0f);
            float new_y = std::accumulate(ys.begin(), ys.end(), 0.0f);

            float new_theta = std::atan2(new_y, new_x);
            float new_intensity = std::sqrt(new_x * new_x + new_y * new_y);

            return Force(new_intensity, new_theta);
        }
     } force_repellors_, force_attractors_, final_force_;

    // Now the internal list for the repellors
    std::vector<std::list<Force>> reppellors_list_;
    std::vector<Force> attractors_list_;
    std::list<Force> raw_repellors_;

    void addRawPoint(Force force);
    float convertToDecay(float distance, float theta);

};

}; // end namespace wheelchair_iaslab_proximityplanner

#endif // WHEELCHAIR_IASLAB_PROXIMITYPLANNER_H_
