/*
 * pp_controller.h
 *
 *  Created on: 03.04.2022
 *  
 *  Copyright 2022 Pascal Lieberherr (ETH Zurich)
 *                 [lipascal@ethz.ch]
 *  All rights reserved.
*/

/*
* class PPController2D
*
* This class implements a pure pursuit controller.
* Based on the pose of the robot and the closest index 
* in the waypoints vector w.r.t the robot. It computes the control commands.
*
*/

#pragma once

// std
#include <fstream>
// project includes
#include "lonomy_path_controller/waypoint.h"

namespace lonomy_path_controller {

// State of robot
struct State {
  double world_t_world_baselink_x; // x value of translation from world to baselink in world frame
  double world_t_world_baselink_y; // y value of translation from world to baselink in world frame
  double yaw_world_baselink; // yaw orientation of baselink frame w.r.t world frame
  double world_v_baselink_x; // x velocity of baselink velocity
  double world_v_baselink_y; // y velocity of baselink velocity
  int closest_idx; // waypoint idx of waypoint closest to current robot pose
};

// output of controller
struct ControlOutput {
  double v_cmd; // the velocity command
  double steer_angle_cmd; // the steer angle command
  int idx_lookahead_wp; // the index of the waypoint at the lookahead distance
};

class PPController
{
  public:

    // Default constructor
    PPController() {};

    // Constructor
    PPController(double lookahead_dist, double L, double v,
                 double K_dd, std::shared_ptr<const lonomy_path_controller::Waypoint> wp_object);

    // Destructor
    virtual ~PPController() {}

    // TODO [lipascal] Can't these functions be private?
    /*!
    * Update state
    * @param world_t_world_baselink_x x position of base link in world frame
    * @param world_t_world_baselink_y y position of base link in world frame
    * @param yaw_world_baselink yaw of base link in world frame
    * @param world_v_baselink_x x velocity of base link in world frame
    * @param world_v_baselink_y y velocity of base link in world frame
    * @param closest_idx in the reference waypoint vector
    */
    void updateState(double world_t_world_baselink_x, double world_t_world_baselink_y, double yaw_world_baselink,
                     double world_v_baselink_x, double world_v_baselink_y, int closest_idx);

    /*!
    * Compute control outputs based on recorded waypoints
    * @return velocity, steer angle, waypoints index at lookahead distance
    */
    ControlOutput getControlCmd() const ;

    /*!
    * Compute control outputs based on interpolated waypoints
    * @return velocity, steer angle, waypoints interpolated index at lookahead distance
    */
    ControlOutput getControlCmdip() const ;

    /*!
    * Get the state from the controller
    * @return state of robot
    */
    State getState() const ;

  private:

    double lookahead_dist_; // lookahead distance w.r.t current robot pose
    double L_; // length of robot (wheel center to wheel center) [m]
    double K_dd_; // gain for velocity dependent lookahead distance
    double v_; // reference velocity
    std::shared_ptr<const lonomy_path_controller::Waypoint> wp_object_ = nullptr;
    State state_;
};
} // namespace lonomy_path_controller