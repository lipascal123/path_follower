/*
 * lonomy_path_controller.h
 *
 *  Created on: 31.03.2022
 *  
 *  Copyright 2022 Pascal Lieberherr (ETH Zurich)
 *                 [lipascal@ethz.ch]
 *  All rights reserved.
 * 
 */

/* 
 * class PathController
 * 
 * This class gets the reference waypoints as an input and computes the
 * twist command for the robot such that it follows the reference waypoints. 
 * The class uses the estimated pose of the robot to compute the control commands.
 * 
 */

#pragma once

// ros includes
#include <ros/ros.h>
#include <ros/package.h> // to find ros package path
#include <nav_msgs/Odometry.h> // to subscribe to odometry msgs
#include <nav_msgs/Path.h> // to publish reference path
#include <grid_map_msgs/GridMap.h> // to subscribe to elevation map
#include <std_msgs/Bool.h> // to publish bool msgs
#include <visualization_msgs/Marker.h> // to publish markers
#include <rowesys_navigation_msgs/AutonomousMode.h> // custom msg
#include <tf/transform_listener.h> // for tf lookups
#include <eigen_conversions/eigen_msg.h> //  to convert geometry msgs to eigen
// std includes
#include <vector> 
#include <tuple> // to return multiple values from function
// eth includes
#include <elevation_map_msgs/CheckSafety.h> // ROS ServiceClient
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp> // to convert grid map msgs between grid map
// own includes
#include "lonomy_path_controller/pp_controller.h" // pure pursuit steering controller class
#include "lonomy_path_controller/waypoint.h" // waypoint class
#include "lonomy_path_controller/swath.h" // to compute swath

namespace lonomy_path_controller {

using std::vector;

class PathController
{
  public:
    // Constructor
    explicit PathController(ros::NodeHandle& node_handle);

    // Destructor
    virtual ~PathController();
    
  private:
    /*!
    * Reads and verifies the parameters.
    * @return true if successful.
    */
    bool readParameters();

    /*! 
    * ROS topic callback method.
    * @param odommsg the received pose estimate of the robot from the simulation.
    */
    void odomCB(const nav_msgs::Odometry& odommsg);
    
    /*! 
    * ROS topic callback method.
    * @param odommsg the received pose estimate of the robot from onboard state estimator.
    */
    void seCB(const geometry_msgs::PoseStamped& odommsg);

    /*! 
    * ROS topic callback method.
    * @param modemsg the received mode message.
    */
    void modeCB(const rowesys_navigation_msgs::AutonomousMode& modemsg);

    /*!
    * ROS topic callback method.
    * @param gridmsg the received elevation map grid message.
    */
    void elevationCB(const grid_map_msgs::GridMap& gridmsg);

    /*!
    * ROS twist message publisher.
    * @param v_x x velocity command for the robot when in Ackermann steering mode.
    * @param omega_z angular z velocity command for the robot when in Ackermann steering mode.
    */
    void twistPub(const double& v_x, const double& omega_z);

    /*!
    * Propagate pose based on path controller and kinematic ackermann model of the vehicle by one time step
    * @param T_world_baselink current pose of the base link frame w.r.t the world frame
    * @param delta_t time step used to propagate current pose with controller
    * @param closest_wp_idx the closest idx in the waypoint vector w.r.t. the current robot pose
    * @return propagated 3D pose
    */
    geometry_msgs::Pose propagate3DPose(const geometry_msgs::Pose& T_world_baselink, double delta_t, int closest_wp_idx);

    /*!
    * Predicts the path of the robot based on current pose, reference trajectory and the controller.
    * @param  T_world_baselink pose of base link in world frame
    * @param  gridmap
    * @return predicted path as a vector of poses
    */
    vector<geometry_msgs::Pose> predictPath(const geometry_msgs::TransformStamped& T_world_baselink,
                                            const grid_map::GridMap& gridmap);

    /*!
     * Computes the euclidean distance and heading error between the two poses passed to the function.
     * @param propagated_pose
     * @param waypoint_pose
     * @return euclidean 2D distance and heading error
     */
     std::tuple<double, double> distanceHeadingError(const geometry_msgs::Pose& propagated_pose,
                                                     const geometry_msgs::Pose& waypoint_pose);

    /*!
    * Function to publish a boost::geometry polygon for visualization in rviz. Points of polygon must be defined w.r.t.
    * the frame_id passed to the function.
    * @param polygon to be published
    * @param frame_id base frame of polygon edge coordinates
    * @param publisher the ros publisher through which polygon is published
    */
    void publishPolygon(const MultiPolygon& polygon, const std::string& frame_id, const ros::Publisher& publisher);

    // ROS node handle.
    ros::NodeHandle& node_handle_;

    // ROS client
    ros::ServiceClient collision_check_client_; // client to request collision check
    
    // ROS topic subscriber.
    ros::Subscriber odom_sub_; // to subscribe to odom pose measuremetns in simulation
    ros::Subscriber mode_sub_; // to subscribe to mode status e.g. AUTONOMOUS_IN_FIELD or MANUAL
    ros::Subscriber se_sub_;  // to subscribe to state estimation on real robot
    ros::Subscriber ele_map_sub_; // to subscribe to elevation grid map

    // ROS topic publisher
    ros::Publisher twist_pub_; // to publish twist message
    ros::Publisher ref_path_pub_; // to publish the reference path
    ros::Publisher lookahead_marker_pub_; // to publish marker at the lookahead distance for pure pursuit controller
    ros::Publisher footprint_pub_; // DEV
    ros::Publisher implement_control_pub_; // to publish the recorded implement control
    ros::Publisher predicted_path_pub_; // to publish propagated path along which collision check is performed
    ros::Publisher swath_pub_; // to publish the swath i.e. the polygon to perform the collision check on

    // Waypoints and path controller
    Waypoint waypoint_; // waypoint class
    vector<vector<double>> waypoints_ip_; // reference waypoints interpolated
    vector<int> waypoints_ip_hash_; // idx of recorded waypoints in waypoints interpolated
    double resolution_; // distance between interpolated waypoints
    int closest_idx_; // closest index in waypoints w.r.t. current robot pose

    // Steering controller
    PPController pp_controller_; // pure pursuit controller object
    PPController pp_prop_controller_; // pure pursuit controller object to propagate state
    double length_robot_; // length of robot (wheel center to wheel center) [m]
    double lookahead_dist_; // lookahead distance w.r.t current robot pose
    double K_dd_; // gain for dynamic lookahead distance
    double v_; // reference velocity
    bool waypoint_following_; // bool to trigger activation of path following 

    // collision checker
    bool collision_; // true if collision is detected
    vector<geometry_msgs::Point> footprint_; // footprint of the robot w.r.t base_link frame

    // tf
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener listener_;

    // Parameters
    bool simulation_; // ture if node is launched in simulation, false if launched on real robot
    std::string state_estimation_topic_sim_; // ROS topic for state estimation data in simulation
    std::string state_estimation_topic_real_; // ROS topic for state estimation data on real robot
    std::string reference_waypoint_file_name_; // filename where reference waypoints are stored
    std::string world_frame_; // world frame e.g. ENU (real robot) or odom (simulation)

    // Logging
    std::ofstream predict_path_timer;
};
} // namespace lonomy_path_controller