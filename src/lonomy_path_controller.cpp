/*
 * lonomy_path_controller.cpp
 *
 *  Created on: 31.03.2022
 *  
 *  Copyright 2022 Pascal Lieberherr (ETH Zurich)
 *                 [lipascal@ethz.ch]
 *  All rights reserved.
 * 
*/

#include "lonomy_path_controller/lonomy_path_controller.h"

namespace lonomy_path_controller {

using std::vector;

// Constructor
PathController::PathController(ros::NodeHandle& node_handle)
  : node_handle_(node_handle),
    listener_(tf_buffer_),
    closest_idx_(0),
    collision_(false),
    waypoint_following_(false)
{
  if(!readParameters()) {
    ROS_ERROR("[lonomy_path_controller] Could not read parameters.");
    ros::requestShutdown();
  }

  // ROS client
  collision_check_client_ = node_handle_.serviceClient<elevation_map_msgs::CheckSafety>("/elevation_mapping/check_safety");

  // ROS subscriber
  odom_sub_ = node_handle_.subscribe(state_estimation_topic_sim_, 10, &PathController::odomCB, this);
  se_sub_ = node_handle_.subscribe(state_estimation_topic_real_, 10, &PathController::seCB, this);
  mode_sub_ = node_handle_.subscribe("/rowesys/robot_autonomous_mode", 10, &PathController::modeCB, this);
  ele_map_sub_ = node_handle_.subscribe("/elevation_mapping/elevation_map_recordable", 10, &PathController::elevationCB, this);

  // ROS publisher
  twist_pub_ = node_handle_.advertise<geometry_msgs::Twist>("/rowesys/robot_twist", 10);
  ref_path_pub_ = node_handle_.advertise<nav_msgs::Path>("/rowesys/ref_path", 10);
  lookahead_marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("/rowesys/lookahead_marker",0);
  footprint_pub_ = node_handle_.advertise<nav_msgs::Path>("/rowesys/footprint", 10);
  predicted_path_pub_ = node_handle_.advertise<nav_msgs::Path>("/rowesys/predicted_path", 10);
  implement_control_pub_ = node_handle_.advertise<std_msgs::Bool>("/rowesys/implement/teach_repeat_control",10);
  swath_pub_ = node_handle_.advertise<nav_msgs::Path>("/lonomy/path_controller/swath",10);

  // Waypoint
  std::string rel_path_pkg = ros::package::getPath("lonomy_teach_path");
  std::string abs_path_csv = rel_path_pkg + "/data/" + reference_waypoint_file_name_;
  ROS_INFO_STREAM("[lonomy_path_controller] path to waypoint file: " << abs_path_csv);
  waypoint_ = Waypoint(abs_path_csv, resolution_);
  ROS_INFO_STREAM("[lonomy_path_controller] " << waypoint_.getSizeWp() << " waypoints found");
  // check if waypoints from csv could be stored in matrix
  if (waypoint_.getSizeWp()==0) {
    ROS_ERROR("[lonomy_path_controller] Could not read waypoints from csv file - vector is empty. Shut down ROS node.");
    ros::requestShutdown();
  }
  ROS_INFO_STREAM("[lonomy_path_controller] " << waypoint_.getSizeWpip() << " interpolated waypoints found");

  // Collision checker
  vector<vector<double>> footprint = { {-0.8, 0.5}, {-0.8, -0.5}, {0.9, -0.5}, {0.9, 0.5} };
  for(int i=3; i>=0; --i) {
    geometry_msgs::Point point;
    point.x = footprint.at(i).at(0);
    point.y = footprint.at(i).at(1);
    footprint_.push_back(point);
  }

  // Pure pursuit steering controller
  pp_controller_ = PPController(lookahead_dist_, length_robot_, v_,K_dd_,
                                std::make_shared<const Waypoint>(waypoint_));

  // To propagate robots path
  pp_prop_controller_ = PPController(lookahead_dist_, length_robot_, v_, K_dd_,
                                     std::make_shared<const Waypoint>(waypoint_));

  // Logging
  predict_path_timer.open("/home/pascal/Documents/predict_path.csv"); // measure runtime of predictPath() function

}

// To read parameters from parameter server
bool PathController::readParameters() {

  // Independent of simulation vs. real robot
  if( !node_handle_.getParam("/lonomy_path_controller/waypoints/waypoints_file_name",reference_waypoint_file_name_)
      || !node_handle_.getParam("/lonomy_path_controller/simulation", simulation_)
      || !node_handle_.getParam("simulation_params/state_estimation_topic", state_estimation_topic_sim_)
      || !node_handle_.getParam("real_robot_params/state_estimation_topic", state_estimation_topic_real_)
      || !node_handle_.getParam("waypoints/resolution", resolution_)
      || !node_handle_.getParam("pure_pursuit_controller/lookahead_dist", lookahead_dist_)
      || !node_handle_.getParam("pure_pursuit_controller/K_dd", K_dd_)
      || !node_handle_.getParam("pure_pursuit_controller/length_robot", length_robot_)
      || !node_handle_.getParam("pure_pursuit_controller/velocity", v_)
      )
    return false;


  // Params dependent on simulation vs. real robot
  if(simulation_) {
    node_handle_.getParam("simulation_params/world_frame", world_frame_);
  }
  else {
    node_handle_.getParam("real_robot_params/world_frame", world_frame_);
  }

  return true;
}

// Destructor
PathController::~PathController()
{}

// Callback
void PathController::seCB(const geometry_msgs::PoseStamped& semsg)
{
  // Check if waypoint following mode is active
  if(!waypoint_following_) return;

  // Check if collision was detected
  if (collision_) {
    ROS_INFO_THROTTLE(3, "[lonomy_path_controller]: collision detected -> zero velocity command");
    twistPub(0, 0);
    return;
  }

  // Find closest waypoint w.r.t. current pose of robot, return the index
  closest_idx_ = waypoint_.getClosestIdxWp(semsg.pose.position.x, semsg.pose.position.y, semsg.pose.position.z, closest_idx_);
  if(closest_idx_ == waypoint_.getSizeWp()-1) {
    ROS_INFO_THROTTLE(3, "[lonomy_path_controller][odomCB]: end of path reached -> zero velocity command");
    twistPub(0, 0);
    return;
  }

  // Compute yaw
  tf::Pose pose;
  tf::poseMsgToTF(semsg.pose, pose);
  double yaw = tf::getYaw(pose.getRotation());

  // Update state 
  pp_controller_.updateState(semsg.pose.position.x, semsg.pose.position.y,
                             yaw, 0, 0, closest_idx_); // currently just give 0,0 velocity
                                                                        // as velocity cmd is hardcoded

  // Get and publish control commands
  ControlOutput ctrl_cmd = pp_controller_.getControlCmdip();
  twistPub(ctrl_cmd.v_cmd, ctrl_cmd.steer_angle_cmd);

  // publish implement control
  std_msgs::Bool implement_active_msg;
  implement_active_msg.data = waypoint_.getFullWp(closest_idx_).implement;
  implement_control_pub_.publish(implement_active_msg);
}

// Callback
void PathController::odomCB(const nav_msgs::Odometry& odommsg)
{
  // Check if waypoint following mode is active
  if(!waypoint_following_) return;
    
  // Check if collision was detected
  if (collision_) {
    ROS_INFO_THROTTLE(3, "[lonomy_path_controller][odomCB]: collision detected -> zero velocity command");
    twistPub(0, 0);
    return;
  } 

  // Find the closest waypoint w.r.t. current pose of robot (base link), return the index
  closest_idx_ = waypoint_.getClosestIdxWp(odommsg.pose.pose.position.x, odommsg.pose.pose.position.y, odommsg.pose.pose.position.z, closest_idx_);
  if(closest_idx_ == waypoint_.getSizeWp()-1) {
    ROS_INFO_THROTTLE(3, "[lonomy_path_controller][odomCB]: end of path reached -> zero velocity command");
    twistPub(0, 0);
    return;
  }

  // Compute yaw
  tf::Pose pose;
  tf::poseMsgToTF(odommsg.pose.pose, pose);
  double yaw = tf::getYaw(pose.getRotation());

  // Update state
  pp_controller_.updateState(odommsg.pose.pose.position.x, odommsg.pose.pose.position.y,
                             yaw, odommsg.twist.twist.linear.x, 
                             odommsg.twist.twist.linear.y, waypoint_.correspondingWpipIdx(closest_idx_));

  // Get and publish control commands
  ControlOutput ctrl_cmd = pp_controller_.getControlCmdip();
  twistPub(ctrl_cmd.v_cmd, ctrl_cmd.steer_angle_cmd);

  // Publish waypoint in lookahead distance as marker to visualize in rviz
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.pose.position.x = waypoint_.get3DWpip(ctrl_cmd.idx_lookahead_wp).position.x;
  marker.pose.position.y = waypoint_.get3DWpip(ctrl_cmd.idx_lookahead_wp).position.y;
  marker.pose.position.z = 0;
  marker.scale.x = 0.15;
  marker.scale.y = 0.15;
  marker.scale.z = 0.15;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  lookahead_marker_pub_.publish(marker);

  // Publish implement control
  std_msgs::Bool implement_active_msg;
  implement_active_msg.data = waypoint_.getFullWp(closest_idx_).implement;
  implement_control_pub_.publish(implement_active_msg);
}

// Callback
void PathController::modeCB(const rowesys_navigation_msgs::AutonomousMode& modemsg)
{
  // Activates waypoint following
  if (modemsg.autonomous_mode == rowesys_navigation_msgs::AutonomousMode::AUTONOMOUS_IN_FIELD) {
    
    waypoint_following_ = true;

    // Publish reference path such that it can be visualized in rviz
    nav_msgs::Path ref_path;
    // Header
    ref_path.header.seq = 1;
    ref_path.header.stamp = ros::Time::now();
    ref_path.header.frame_id = world_frame_; // simulation: map / real robot: ENU
    //!
    for(int i=0; i<waypoint_.getSizeWp(); i++) {
      // Pose stamped
      geometry_msgs::PoseStamped pose;
      pose.header.seq = i;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = world_frame_; // simulation: map / real robot: ENU
      pose.pose.position.x = waypoint_.getFullWp(i).x;
      pose.pose.position.y = waypoint_.getFullWp(i).y;
      pose.pose.position.z = waypoint_.getFullWp(i).z;
      pose.pose.orientation.x = waypoint_.getFullWp(i).qx;
      pose.pose.orientation.y = waypoint_.getFullWp(i).qy;
      pose.pose.orientation.z = waypoint_.getFullWp(i).qz;
      pose.pose.orientation.w = waypoint_.getFullWp(i).qw;
      // add to path
      ref_path.poses.push_back(pose);
    }
      // publish
      ref_path_pub_.publish(ref_path);      
  }

  // Deactivates waypoint following
  else if (modemsg.autonomous_mode == rowesys_navigation_msgs::AutonomousMode::MANUAL) 
    waypoint_following_ = false;
}

// Callback elevation_mapping_cupy
void PathController::elevationCB(const grid_map_msgs::GridMap& gridmsg){

  geometry_msgs::TransformStamped T_world_baselink;
  try {
    T_world_baselink = tf_buffer_.lookupTransform(world_frame_, "base_link", ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
    ROS_WARN("[lonomy_path_controller][elevationCB] Exception in transform lookup");
  }

  // Create grid map object
  grid_map::GridMap grid_map;
  grid_map::GridMapRosConverter::fromMessage(gridmsg, grid_map);

  // Get predicted path inside map
  vector<geometry_msgs::Pose> predicted_path = predictPath(T_world_baselink, grid_map);

  // Compute Swath
  MultiPolygon swath_poly = computeSwath(footprint_, predicted_path);
  publishPolygon(swath_poly, world_frame_, swath_pub_);

  // Collision check
  vector<geometry_msgs::PolygonStamped> polys_stamped = convertToPolygonStamped(swath_poly, world_frame_ ,ros::Time::now());
  elevation_map_msgs::CheckSafety srv;
  srv.request.polygons = polys_stamped;
  srv.request.compute_untraversable_polygon = false;
//  // TODO [lipascal]: uncomment below once ros service works
//  if(collision_check_client_.call(srv)) {
//    if (std::find(srv.response.is_safe.begin(), srv.response.is_safe.end(), false) != srv.response.is_safe.end())
//      collision_ = true;
//    else
//      collision_ = false;
//  }
//  else
//    ROS_ERROR("Failed to call service check_safety");
}

// To predict the path of the robot
vector<geometry_msgs::Pose> PathController::predictPath(const geometry_msgs::TransformStamped& T_world_baselink,
                                                             const grid_map::GridMap& gridmap)
{
  ros::Time time_start = ros::Time::now();

  // INIT variables
  // Conversion into new data structure
  geometry_msgs::Pose T_world_baselink_temp;
  T_world_baselink_temp.orientation = T_world_baselink.transform.rotation;
  T_world_baselink_temp.position.x = T_world_baselink.transform.translation.x;
  T_world_baselink_temp.position.y = T_world_baselink.transform.translation.y;
  T_world_baselink_temp.position.z = T_world_baselink.transform.translation.z;
  // Path message to visualize in rviz
  nav_msgs::Path path_prop_pathmsgs;
  path_prop_pathmsgs.header.seq = 1;
  path_prop_pathmsgs.header.stamp = ros::Time::now();
  path_prop_pathmsgs.header.frame_id = world_frame_; // simulation: odom, real robot: ENU
  geometry_msgs::PoseStamped pose;
  pose.pose = T_world_baselink_temp;
  path_prop_pathmsgs.poses.push_back(pose);
  // Init error values
  double dist_error, heading_error;
  std::tie(dist_error, heading_error) = distanceHeadingError(T_world_baselink_temp,waypoint_.get3DWp(closest_idx_));
  // Add first pose to path
  vector<geometry_msgs::Pose> path_predicted;
  path_predicted.push_back(T_world_baselink_temp);
  // More init variables
  double delta_t = 0.15;
  int closest_prop_idx = closest_idx_;
  geometry_msgs::Pose T_world_baselink_prop = T_world_baselink_temp;

  // PROPAGATE
  // Propagate until propagated pose is close to reference waypoint
  while( (dist_error > 0.03 || heading_error > 0.1) )
  {
    // Propagate pose based on controller and kinematic model
    T_world_baselink_prop = propagate3DPose(T_world_baselink_prop, delta_t, closest_prop_idx);
    // Update closest wp idx
    closest_prop_idx = waypoint_.getClosestIdxWp(T_world_baselink_prop.position.x,
                                                 T_world_baselink_prop.position.y,
                                                 T_world_baselink_prop.position.z, // keep height const
                                                 closest_prop_idx);
    // Check if propagated pose is inside map and
    // check if closest_prop_idx is the last waypoint
    // This is needed because when the path ends the distance and heading errors keep increasing
    // instead of decreasing. Hence, we would never exit this while loop
    Eigen::Vector2d position(T_world_baselink_prop.position.x,T_world_baselink_prop.position.y);
    if (!gridmap.isInside(position) || closest_prop_idx == waypoint_.getSizeWp()-1)
      break;
    // append propagated pose
    path_predicted.push_back(T_world_baselink_prop);

    // Add propagated pose to path msgs for visualization
    geometry_msgs::PoseStamped pose_prop;
    pose_prop.pose = T_world_baselink_prop;
    path_prop_pathmsgs.poses.push_back(pose_prop);

    // Compute difference between propagated pose and closes waypoint
    std::tie(dist_error, heading_error) = distanceHeadingError(T_world_baselink_prop,
                                                                     waypoint_.get3DWp(closest_prop_idx));
  }

  // EXTRACT
  // Extract reference waypoints until end of map is reached
  geometry_msgs::Pose T_world_baselink_extracted = waypoint_.get3DWp(closest_prop_idx);
  // check if the next waypoint is inside the map
  Eigen::Vector2d position(T_world_baselink_extracted.position.x,T_world_baselink_extracted.position.y);
  while (gridmap.isInside(position))
  {
    // Add extracted pose to path and to path msgs for visualization
    path_predicted.push_back(T_world_baselink_extracted);
    geometry_msgs::PoseStamped pose_extr;
    pose_extr.pose = T_world_baselink_extracted;
    path_prop_pathmsgs.poses.push_back(pose_extr);

    // Get the next waypoint unless it's the end of the path
    closest_prop_idx +=1;
    if(closest_prop_idx > waypoint_.getSizeWp()-1)
      break;
    T_world_baselink_extracted = waypoint_.get3DWp(closest_prop_idx);
    position = Eigen::Vector2d(T_world_baselink_extracted.position.x,T_world_baselink_extracted.position.y);
  }

  // publish predicted path
  predicted_path_pub_.publish(path_prop_pathmsgs);

  ros::Duration predict_path_dur = ros::Time::now() - time_start;
  predict_path_timer << predict_path_dur << "\n" ;

  return  path_predicted;
}

// Computes the euclidean distance and heading error
std::tuple<double, double> PathController::distanceHeadingError(const geometry_msgs::Pose& propagated_pose,
                                                                const geometry_msgs::Pose& waypoint_pose)
{
  // Compute errors w.r.t reference waypoints
  // distance error
  double dist_error = sqrt( pow(propagated_pose.position.x - waypoint_pose.position.x, 2 ) +
                            pow(propagated_pose.position.y - waypoint_pose.position.y, 2 ) );
  // Orientation error
  // yaw of recorded waypoint
  geometry_msgs::Quaternion q_current = waypoint_pose.orientation;
  double yaw_current = tf::getYaw(q_current);
  // yaw of propagated pose
  geometry_msgs::Quaternion q_previous = propagated_pose.orientation;
  double yaw_previous = tf::getYaw(q_previous);
  // error
  double heading_error = yaw_previous - yaw_current;

  return {dist_error, heading_error};
}

// Propagate a pose through the path controller and return 3D pose
geometry_msgs::Pose PathController::propagate3DPose(const geometry_msgs::Pose& T_world_baselink, double delta_t,
                                                    int closest_wp_idx)
{
  // get yaw
  tf2::Quaternion quat(T_world_baselink.orientation.x, T_world_baselink.orientation.y,
                       T_world_baselink.orientation.z, T_world_baselink.orientation.w);
  double yaw, pitch, roll;
  tf2::getEulerYPR(quat, yaw, pitch, roll);

  // Update state
  pp_prop_controller_.updateState(T_world_baselink.position.x,
                                  T_world_baselink.position.y, yaw,
                                  0.2, 0, closest_wp_idx);
  // Compute control cmd
  auto [v_cmd, steer_angle, idx_lookahead_wp] = pp_prop_controller_.getControlCmd();
  // Propagate with kinematic uni-bicycle model with reference point base_link
  // Note: x-distance from rear_axle to base_link is half the length of the robot
  double beta = atan2(tan(steer_angle),2); // Derivation: https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357
  double x_dot_prop = v_cmd * cos(yaw+beta);
  double y_dot_prop = v_cmd * sin(yaw+beta);
  double theta_dot_prop = v_cmd * cos(beta) * tan(steer_angle) / length_robot_;

  geometry_msgs::Pose T_wold_baselink_prop;
  T_wold_baselink_prop.position.x = T_world_baselink.position.x + x_dot_prop * delta_t; // propagated with ackermann model
  T_wold_baselink_prop.position.y = T_world_baselink.position.y + y_dot_prop * delta_t; // propagated with ackermann model
  T_wold_baselink_prop.position.z = T_world_baselink.position.z; // just keep it const
  // set quaternion
  //! with setEulerZYX it seems to work better than with setEuler
  quat.setEulerZYX(yaw + theta_dot_prop * delta_t, pitch, roll);
  // Convert tf2 quaternion to geometry msgs
  geometry_msgs::Quaternion quat_msg;
  quat_msg = tf2::toMsg(quat);
  T_wold_baselink_prop.orientation = quat_msg;

  return T_wold_baselink_prop;
}

// Publish a polygon e.g. swath
void PathController::publishPolygon(const MultiPolygon& polygon, const std::string& frame_id, const ros::Publisher& publisher)
{
  // Iterate over every polygon in multi polygon
  for(auto poly : polygon) {
    auto poly_ring = poly.outer();
    nav_msgs::Path polygon_path;
    // header
    polygon_path.header.seq = 1;
    polygon_path.header.stamp = ros::Time::now();
    polygon_path.header.frame_id = frame_id;
    int i = 0;
    //Iterate over all points in the polygon
    for (auto points: poly_ring) {
      // Pose stamped
      geometry_msgs::PoseStamped pose;
      pose.header.seq = i;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = frame_id;
      pose.pose.position.x = points.x();
      pose.pose.position.y = points.y();
      pose.pose.position.z = 0;
      pose.pose.orientation.x = 0;
      pose.pose.orientation.y = 0;
      pose.pose.orientation.z = 0;
      pose.pose.orientation.w = 1;
      // add to path
      polygon_path.poses.push_back(pose);
      ++i;
    }

    // add first point of footprint again to get closed shape
    polygon_path.poses.push_back(polygon_path.poses[0]);
    // publish footprint
    publisher.publish(polygon_path);
  }
}

// Function to publish twist command
void PathController::twistPub(const double& v_x, const double& omega_z)
{
  geometry_msgs::Twist twistmsg;
  twistmsg.linear.x = v_x;
  twistmsg.linear.y = 0;
  twistmsg.linear.z = 0;

  twistmsg.angular.x = 0;
  twistmsg.angular.y = 0;
  twistmsg.angular.z = omega_z;

  twist_pub_.publish(twistmsg);
}

} // namespace lonomy_path_controller
