/*
 * waypoint.h
 *
 *  Created on: 04.05.2022
 *  
 *  Copyright 2022 Pascal Lieberherr (ETH Zurich)
 *                 [lipascal@ethz.ch]
 *  All rights reserved.
*/

/*
* class Waypoint
*
* This class stores and handles the recorded and interpolated waypoints.
* It implements functionalities in connection with the waypoints.
*
*/

#pragma once

// std includes
#include <fstream> // to read from csv file
#include <memory> // for shared ptr
#include <iostream> // for cout
#include <stdexcept> // for throw
// ros includes
#include <geometry_msgs/Pose.h> // To subscribe to pose msgs
#include <geometry_msgs/Pose2D.h> // To subscribe to pose msgs
#include <tf2/utils.h> // for tf2 get yaw

namespace lonomy_path_controller {

// Defined macros
#define YELLOW_START "\033[33m"
#define RED_START "\033[31m"
#define COLOR_END "\033[0m"
using std::vector;

// Struct for a single full waypoint
struct SingleFullWaypoint {
  double x,y,z;
  double qx, qy, qz, qw;
  double v;
  bool implement;
};

class Waypoint
{
  public:
    // Constructor
    Waypoint(const std::string& path, double resolution);

    // Default constructor
    Waypoint()=default;

    // Default destructor
    virtual ~Waypoint()=default;

    /*!
    * Function to get a single recorded full waypoint (pose, velocity, implement)
    * @param idx index of desired recorded waypoint
    * @return single recorded waypoint as a struct
    */
    inline SingleFullWaypoint getFullWp(int idx) const
    {
      return wp_.at(idx);
    }

    /*!
    * Function to get a single interpolated full waypoint (pose, velocity, implement)
    * @param idx index of desired interpolated waypoint
    * @return single interpolated waypoint as a struct
    */
    inline SingleFullWaypoint getFullWpip(int idx) const
    {
      return wp_ip_.at(idx);
    }

    /*!
    * Function to get a single 3D waypoint (pose in 3D)
    * @param idx index of desired interpolated waypoint
    * @return single 3D waypoint (pose in 3D)
    */
    geometry_msgs::Pose get3DWp(int idx) const;

    /*!
    * Function to get a single 2D waypoint (x,y,yaw)
    * @param idx index of desired interpolated waypoint
    * @return single 2D waypoint (x,y,yaw)
    */
    geometry_msgs::Pose2D get2DWp(int idx) const;

    /*!
    * Function to get a single interpolated 3D waypoint (pose)
    * @param idx index of desired interpolated waypoint
    * @return single interpolated 3D waypoint
    */
    geometry_msgs::Pose get3DWpip(int idx) const;

    /*!
    * Function to get a single interpolated 2D waypoint (x,y,yaw)
    * @param idx index of desired interpolated waypoint
    * @return single interpolated 2D waypoint
    */
    geometry_msgs::Pose2D get2DWpip(int idx) const;

    /*!
    * Function to find closest waypoint w.r.t. the input position.
    * First increment the index from the previous index until the new distance calculations
    * are increasing. Apply the same rule decrementing the index.
    * The final index should be the closest point.
    * @param xpos x position
    * @param ypos y position
    * @param zpos z position
    * @param prev_closest_idx closest waypoint index from previous iteration
    * @return closest index in recorded waypoints
    */
    int getClosestIdxWp(double xpos, double ypos, double zpos, int prev_closest_idx) const;

    /*!
    * Function to find closest interpolated waypoint w.r.t. the input position.
    * First increment the index from the previous index until the new distance calculations
    * are increasing. Apply the same rule decrementing the index.
    * The final index should be the closest point.
    * @param xpos x position
    * @param ypos y position
    * @param zpos z position
    * @param prev_closest_idx closest waypoint index from previous iteration
    * @return closest index in interpolated waypoints
    */
    int getClosestIdxWpip(double xpos, double ypos, double zpos, int prev_closest_idx) const;

    /*!
    * Function to get the index of the corresponding waypoint in the
    * interpolated waypoints.
    * @param wp_idx index of recorded waypoints
    * @return the corresponding index in the interpolated waypoints
    */
    inline int correspondingWpipIdx(int wp_idx) const
    {
      return wp_ip_hash_.at(wp_idx);
    }

    /*!
    * Function to get set of full waypoints between start_idx and start_idx+lookahead
    * @param start_idx start index of recorded waypoints
    * @param lookahead distance from start index until last point to be extracted
    * @return vector with extracted full waypoint
    */
    vector<SingleFullWaypoint> getSetFullWpip(int start_idx, double lookahead)  const;

    /*!
    * Function to get set of 3D waypoints between start_idx and start_idx+lookahead.
    * @param start_idx start index of recorded waypoints
    * @param lookahead distance from start index until last point to be extracted
    * @return vector with extracted 3D waypoints
    */
    vector<geometry_msgs::Pose> getSet3DWpip(int start_idx, double lookahead) const;

    /*!
    * Function to get set of 2D waypoints between start_idx and start_idx+lookahead.
    * @param start_idx start index of recorded waypoints
    * @param lookahead distance from start index until last point to be extracted
    * @return vector with extracted 2D waypoints
    */
    vector<geometry_msgs::Pose2D> getSet2DWpip(int start_idx, double lookahead) const;

    /*!
    * Function to get the length of the waypoint vector i.e. the number of waypoints
    * @return the size of waypoint vector.
    */
    inline int getSizeWp() const
    {
      return wp_.size();
    }

    /*!
    * Function to get the length of the interpolated waypoint
    * vector i.e. the number of interpolated waypoints
    * @return the size of interpolated waypoint vector.
    */
    inline int getSizeWpip() const
    {
      return wp_ip_.size();
    }

  private:

    /*!
    * Function to interpolate between recorded waypoints to get a reference path
    * with a specified resolution.
    * @param resolution distance between interpolated points
    * @return interpolated waypoints as multidimensional vector and hash table
    */
    std::tuple< vector<int>, vector<SingleFullWaypoint> > interpolateWP(double resolution);

    /*!
    * Function to read csv file into Matrix.
    * @param path absolute path to csv file.
    * @return eigen matrix e.g. Eigen::MatrixXd
    */
    vector<SingleFullWaypoint> loadCsv (const std::string& path);

    vector<SingleFullWaypoint> wp_; // recorded waypoints
    vector<SingleFullWaypoint> wp_ip_; // interpolated waypoints
    vector<int> wp_ip_hash_; // hash table to store idx of recorded waypoints in waypoints interpolated
    double resolution_; // resolution of interpolated waypoints
};
} // namespace lonomy_path_controller
