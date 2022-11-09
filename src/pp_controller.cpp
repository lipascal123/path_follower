/*
 * pp_controller.cpp
 *
 *  Created on: 15.08.2022
 *
 *  Copyright 2022 Pascal Lieberherr (ETH Zurich)
 *                 [lipascal@ethz.ch]
 *  All rights reserved.
*/

#include "lonomy_path_controller/pp_controller.h"
#include "lonomy_path_controller/waypoint.h"

namespace lonomy_path_controller {

using std::cout, std::endl;

// Constructor
PPController::PPController(double lookahead_dist, double L, double v,
                           double K_dd, std::shared_ptr<const lonomy_path_controller::Waypoint> wp_object) {
  lookahead_dist_ = lookahead_dist;
  L_ = L;
  K_dd_ = K_dd;
  wp_object_ = wp_object;
  if(wp_object == nullptr)
    cout << RED_START << "[pp_controller] Pointer not yet initalized to waypoint object" << COLOR_END << endl;
  v_ = v;
}

// Update state with world_T_world_baselink
void PPController::updateState(double world_t_world_baselink_x, double world_t_world_baselink_y,
                               double yaw_world_baselink, double world_v_baselink_x,
                               double world_v_baselink_y, int closest_idx) {
  state_.world_t_world_baselink_x = world_t_world_baselink_x;
  state_.world_t_world_baselink_y = world_t_world_baselink_y;
  state_.world_v_baselink_x = world_v_baselink_x;
  state_.world_v_baselink_y = world_v_baselink_y;
  state_.yaw_world_baselink = yaw_world_baselink;
  state_.closest_idx = closest_idx;
}

// Compute control outputs based on recorded waypoints
ControlOutput PPController::getControlCmd() const {
  // First idx is one behind the closest idx
  int wp_subset_first_idx = state_.closest_idx - 1;
  if (wp_subset_first_idx < 0) // beginning of path
    wp_subset_first_idx = 0;
  // Compute lookahead distance
  //  double v = 0.5; // CURRENTLY STATIC
  // Note: Currently static lookahead distance is used
  // pure pursuit can also use dynamic lookahaead scaled with velocity
  // lookahead_dist = v*K_dd_; // dynamic lookahead distance

  // Compute rear axle center point
  double x_rear = state_.world_t_world_baselink_x - cos(state_.yaw_world_baselink) * L_ / 2;
  double y_rear = state_.world_t_world_baselink_y - sin(state_.yaw_world_baselink) * L_ / 2;

  double rear_axle_2_waypoint_x = 0;
  double rear_axle_2_waypoint_y = 0;
  int idx_lookahead_wp = 0;
  //  Compute vector from rear axle center point to waypoint in lookahead distance
  for (int i = wp_subset_first_idx; i < (wp_object_->getSizeWp()); i++) {
    geometry_msgs::Pose2D wp = wp_object_->get2DWp(i);
    double dist = sqrt(pow(wp.x - x_rear, 2) + pow(wp.y - y_rear, 2));

    // Check if corresponding waypoint in lookahead distance was found
    // or if end of waypoints reached. In this case we set lookahead waypoint to last waypoint
    if (dist > lookahead_dist_ || i == wp_object_->getSizeWp() - 1) {
      // compute line from rear axle to waypoint at lookahead distance
      rear_axle_2_waypoint_x = wp.x - x_rear;
      rear_axle_2_waypoint_y = wp.y - y_rear;

      idx_lookahead_wp = i;
      break;
    }
  }

  // Compute alpha
  double alpha_hat = atan2(rear_axle_2_waypoint_y, rear_axle_2_waypoint_x);
  double alpha = alpha_hat - state_.yaw_world_baselink;

  // Compute steer angle for uni-bicycle model
  double steer_angle = atan2(2 * L_ * sin(alpha), lookahead_dist_);

  // Compute twist as we did in focus project (temporary solution)
  double v_cmd = v_; // [m/s]

  ControlOutput output;
  output.v_cmd = v_cmd;
  output.steer_angle_cmd = steer_angle;
  output.idx_lookahead_wp = idx_lookahead_wp;

  return output;
}

// Compute control outputs based on interpolated waypoints
ControlOutput PPController::getControlCmdip() const {
  // First idx is one behind the closest idx
  int wp_subset_first_idx = state_.closest_idx - 1; //! ip
  if (wp_subset_first_idx < 0) // beginning of path
    wp_subset_first_idx = 0;

  // Compute lookahead distance
  double v = 0.5; // CURRENTLY STATIC
  // Note: Currently static lookahead distance is used
  // pure pursuit can also use danymic lookahaead scaled with velocity
  // lookahead_dist = v*K_dd_; // dynamic lookahead distance

  // Compute rear axle center point
  double x_rear = state_.world_t_world_baselink_x - cos(state_.yaw_world_baselink) * L_ / 2;
  double y_rear = state_.world_t_world_baselink_y - sin(state_.yaw_world_baselink) * L_ / 2;

  //  Compute vector from rear axle center point to waypoint in lookahead distance
  double rear_axle_2_waypoint_x = 0;
  double rear_axle_2_waypoint_y = 0;
  int idx_lookahead_wp = 0;
  for (int i = wp_subset_first_idx; i < wp_object_->getSizeWpip(); i++) //! ip
  {
    geometry_msgs::Pose2D wp = wp_object_->get2DWpip(i);
    double dist = sqrt(pow(wp.x - x_rear, 2) +
                       pow(wp.y - y_rear, 2));

    if (dist > lookahead_dist_ || i == wp_object_->getSizeWpip() - 1) {
      // compute line from rear axle to waypoint at lookahead distance
      rear_axle_2_waypoint_x = wp.x - x_rear;
      rear_axle_2_waypoint_y = wp.y - y_rear;

      idx_lookahead_wp = i;
      break;
    }
  }

  // Compute alpha
  double alpha_hat = atan2(rear_axle_2_waypoint_y, rear_axle_2_waypoint_x);
  double alpha = alpha_hat - state_.yaw_world_baselink;

  // Compute steer angle for unibicycle model
  double steer_angle = atan2(2 * L_ * sin(alpha), lookahead_dist_);

  // Compute twist as we did in focus project (temporary solution)
  double v_cmd = v_; // [m/s]

  ControlOutput output;
  output.v_cmd = v_cmd;
  output.steer_angle_cmd = steer_angle;
  output.idx_lookahead_wp = idx_lookahead_wp;

  return output;
}

// Get state from controller
State PPController::getState() const {
  return state_;
}

} // namespace lonomy_path_controller