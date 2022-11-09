/*
 * waypoints.cpp
 *
 *  Created on: 04.05.2022
 *  
 *  Copyright 2022 Pascal Lieberherr (ETH Zurich)
 *                 [lipascal@ethz.ch]
 *  All rights reserved.
 * 
*/

#include "lonomy_path_controller/waypoint.h"

using std::vector;

namespace lonomy_path_controller {

// Constructor
Waypoint::Waypoint(const std::string& path, double resolution)
  : resolution_(resolution),
    wp_(loadCsv(path))
{
  // Interpolate waypoints
  std::tie(wp_ip_hash_, wp_ip_) = interpolateWP(resolution_);
}

// Function to interpolate between recorded waypoints
std::tuple< vector<int>, vector<SingleFullWaypoint> > Waypoint::interpolateWP(double resolution)
{
  // TODO: [lipascal] also consider the implement in the interpolation
  // Compute distances between waypoints and store them in a vector
  vector<double> wp_distance;
  for (int i=0; i<wp_.size()-1; i++) {
    // compute xy-distance: at idx i it's the distance from i to i+1
    // TODO [lipascal] Also consider z position here?
    double dist = sqrt( pow( wp_.at(i).x - wp_.at(i+1).x, 2) +
                        pow( wp_.at(i).y - wp_.at(i+1).y, 2) );
    wp_distance.push_back(dist); 
  }
  wp_distance.push_back(0); // last distance is 0

  // Initialize
  vector<SingleFullWaypoint> wp_ip; // interpolated values, rows: waypoints, columns: [x,y,v]
  vector<int> wp_ip_hash; // hash table to find recorded waypoint in interpolated waypoints
  int ip_counter = 0;

  // Iterate over every recorded waypoint
  for(int i=0; i<wp_.size()-1; i++) {
    // add recorded waypoint to interpolated waypoints and to hash table
    // x,y,z,qx,qy,qz,qw,v,i
    // TODO [lipascal] Currently just dummy value for implement -> also implement interpolation of implement
    wp_ip.push_back(wp_.at(i));
    wp_ip_hash.push_back(ip_counter);
    ip_counter += 1; 

    // Compute number of points to interpolate to get desired resolution of interpolated waypoints
    int num_points_to_ip = (int)((wp_distance.at(i) / resolution) -1 );

    // Compute direction vector which will be added during linear interpolation
    //! Currently orientation is not being interpolated
    // TODO [lipascal] interpolate orientation
    SingleFullWaypoint vector = {};
    vector.x = wp_.at(i+1).x - wp_.at(i).x;   // delta_x
    vector.y = wp_.at(i+1).y - wp_.at(i).y;   // delta_y
    vector.z = wp_.at(i+1).z - wp_.at(i).z;   // delta_z
    vector.qx = 0; // delta_qx
    vector.qy = 0; // delta_qy
    vector.qz = 0; // delta_qz
    vector.qw = 0; // delta_qw
    vector.v = wp_.at(i+1).v - wp_.at(i).v; // delta_v
    vector.implement = false;

    // compute norm of x,y,z (i.e. without v)
    double norm_xyz = sqrt( pow(vector.x,2) + pow(vector.y,2) + pow(vector.z,2));
    // compute vector direction vector
    std::vector<double> dir_vector = {vector.x / norm_xyz, // x
                                      vector.y / norm_xyz, // y
                                      vector.z / norm_xyz, // z
                                      vector.v / (num_points_to_ip+1) }; // v
                                
    // perform linear interpolation
    for(int j=1; j<=num_points_to_ip; j++) {
      SingleFullWaypoint interpolated_waypoint = {};
      interpolated_waypoint.x = wp_.at(i).x + j*resolution*dir_vector[0];
      interpolated_waypoint.y = wp_.at(i).y + j*resolution*dir_vector[1];
      interpolated_waypoint.z = wp_.at(i).z + j*resolution*dir_vector[2];
      interpolated_waypoint.qx = wp_.at(i).qx;
      interpolated_waypoint.qy = wp_.at(i).qy;
      interpolated_waypoint.qz = wp_.at(i).qz;
      interpolated_waypoint.qw = wp_.at(i).qw;
      interpolated_waypoint.v = wp_.at(i).v + j*dir_vector[3];
      interpolated_waypoint.implement = false;
      // add interpolated waypoint
      wp_ip.push_back(interpolated_waypoint);
        
      ip_counter += 1;
    }
  }

  // add last waypoint at the end
  wp_ip.push_back(wp_.back());
  wp_ip_hash.push_back(ip_counter);

  return {wp_ip_hash, wp_ip}; // structured binding -> only works with C++ 17
}

// Function to read csv file and save it in eigen matrix
vector<SingleFullWaypoint> Waypoint::loadCsv (const std::string& path)
{
  std::ifstream indata;
  indata.open(path); // open file
  std::string line;

  vector<SingleFullWaypoint> values;
  uint row = 0;
  // check if there is data in indata
  while (std::getline(indata, line)) {
    // stringstream has all characters from the file
    std::stringstream lineStream(line);
    std::string cell;
    // extract characters in new string until '\n' and save it to cell
    while (std::getline(lineStream, cell, '\n'))
    {
      SingleFullWaypoint single_wp;
      // save cell elements to vector and skip the comma
      std::stringstream ss(cell);
      int counter = 0;
      for (double i = 0; ss >> i;) {
        // switch to assign extracted value to right field in struct
        switch (counter) {
          case 0:
            single_wp.x = i;
            break;
          case 1:
            single_wp.y = i;
            break;
          case 2:
            single_wp.z = i;
            break;
          case 3:
            single_wp.qx = i;
            break;
          case 4:
            single_wp.qy = i;
            break;
          case 5:
            single_wp.qz = i;
            break;
          case 6:
            single_wp.qw = i;
            break;
          case 7:
            single_wp.v = i;
            break;
          case 8:
            single_wp.implement = static_cast<bool>(i);
            break;
          default:
            std::cout << RED_START <<"[waypoint.cpp][loadCsv] No field in the struct for extracted value in csv file" << COLOR_END <<std::endl;
        } // switch

        if (ss.peek() == ',')
          ss.ignore();
        ++counter;
      }
      values.push_back(single_wp);
    }
    ++row;
  }
  indata.close(); // close file
  return values;
}

// Function to get a single 3D waypoint (pose)
geometry_msgs::Pose Waypoint::get3DWp(int idx) const
{
  geometry_msgs::Pose pose3D;
  pose3D.position.x = wp_.at(idx).x;
  pose3D.position.y = wp_.at(idx).y;
  pose3D.position.z = wp_.at(idx).z;
  pose3D.orientation.x = wp_.at(idx).qx;
  pose3D.orientation.y = wp_.at(idx).qy;
  pose3D.orientation.z = wp_.at(idx).qz;
  pose3D.orientation.w = wp_.at(idx).qw;

  return pose3D;
}

// Function to get a single 2D waypoint (x,y,yaw)
geometry_msgs::Pose2D Waypoint::get2DWp(int idx) const
{
  geometry_msgs::Pose2D pose2D;
  // get position
  pose2D.x = wp_.at(idx).x;
  pose2D.y = wp_.at(idx).y;
  // get yaw
  geometry_msgs::Quaternion quat;
  quat.x = wp_.at(idx).qx;
  quat.y = wp_.at(idx).qy;
  quat.z = wp_.at(idx).qz;
  quat.w = wp_.at(idx).qw;
  pose2D.theta = tf2::getYaw(quat);

  return pose2D;
}

// Function to get a single interpolated 3D waypoint (pose)
geometry_msgs::Pose Waypoint::get3DWpip(int idx) const
{
  geometry_msgs::Pose pose3D;
  pose3D.position.x = wp_ip_.at(idx).x;
  pose3D.position.y = wp_ip_.at(idx).y;
  pose3D.position.z = wp_ip_.at(idx).z;
  pose3D.orientation.x = wp_ip_.at(idx).qx;
  pose3D.orientation.y = wp_ip_.at(idx).qy;
  pose3D.orientation.z = wp_ip_.at(idx).qz;
  pose3D.orientation.w = wp_ip_.at(idx).qw;

  return pose3D;
}

// Function to get a single interpolated 2D waypoint (x,y,yaw)
geometry_msgs::Pose2D Waypoint::get2DWpip(int idx) const
{
  geometry_msgs::Pose2D pose2D;
  // get position
  pose2D.x = wp_ip_.at(idx).x;
  pose2D.y = wp_ip_.at(idx).y;
  // get yaw
  geometry_msgs::Quaternion quat;
  quat.x = wp_ip_.at(idx).qx;
  quat.y = wp_ip_.at(idx).qy;
  quat.z = wp_ip_.at(idx).qz;
  quat.w = wp_ip_.at(idx).qw;
  pose2D.theta = tf2::getYaw(quat);

  return pose2D;
}

// Function to get closest waypoint index w.r.t. input position
int Waypoint::getClosestIdxWp(double xpos, double ypos, double zpos, int prev_closest_idx) const
{
  double closest_dist = sqrt( pow(wp_.at(prev_closest_idx).x - xpos, 2) +
                              pow(wp_.at(prev_closest_idx).y - ypos, 2) +
                              pow(wp_.at(prev_closest_idx).z - zpos, 2) );

  // Initialize variabels
  double new_dist = closest_dist;
  int new_idx = prev_closest_idx;
  int closest_idx = prev_closest_idx;

  // Find closest waypoint by incrementing idx from previous idx
  // until new distance calculations are increasing  
  while (new_dist <= closest_dist) {
    closest_dist = new_dist;
    closest_idx = new_idx;
    new_idx += 1;

    if (new_idx >= wp_.size()) // end of path
    {
      break;
    }
    new_dist = sqrt( pow(wp_.at(new_idx).x - xpos, 2) +
                     pow(wp_.at(new_idx).y - ypos, 2) +
                     pow(wp_.at(new_idx).z - zpos, 2) );
  }

  // Apply the same rule decrementing the idx
  // to avoid getting stuck at an outlier
  // Initialize:
  int idx_lookahead = 4;
  int back_start_idx = closest_idx + idx_lookahead;
  int decremented_idx = back_start_idx;
  while(decremented_idx > back_start_idx-idx_lookahead) {
    try
    {
      // It could be that idx_lookahead+closest_idx is out of range when closest_idx
      // is almost the size of the waypoint vector. In this case just catch
      // and break the while loop.
      new_dist = sqrt(pow(wp_.at(decremented_idx).x - xpos, 2) +
                      pow(wp_.at(decremented_idx).y - ypos, 2) +
                      pow(wp_.at(decremented_idx).z - zpos, 2));
    }
    catch (const std::out_of_range& ex)
    {
      // Query index + lookahead index bigger than
      // size of waypoint vector Thus, lookahead index is ignored
      break;
    }

    if (new_dist <= closest_dist) {
      closest_dist = new_dist;
      closest_idx = decremented_idx;
    }

    decremented_idx -= 1;

    if (decremented_idx<0) // beginning of path
      break;
  }
  return closest_idx;
}

// Function to get closest interpolated waypoint index w.r.t. input position
int Waypoint::getClosestIdxWpip(double xpos, double ypos, double zpos, int prev_closest_idx) const
{
  double closest_dist = sqrt( pow(wp_ip_.at(prev_closest_idx).x - xpos, 2) +
                              pow(wp_ip_.at(prev_closest_idx).y - ypos, 2) +
                              pow(wp_ip_.at(prev_closest_idx).z - zpos, 2) );

  // Initialize variabels
  double new_dist = closest_dist;
  int new_idx = prev_closest_idx;
  int closest_idx = prev_closest_idx;

  // Find closest waypoint by incrementing idx from previous idx
  // until new distance calculations are increasing  
  while (new_dist <= closest_dist) {
    closest_dist = new_dist;
    closest_idx = new_idx;
    new_idx += 1;

    if (new_idx >= wp_ip_.size()) // end of path
    {
      break;
    }
    new_dist = sqrt( pow(wp_ip_.at(new_idx).x - xpos, 2) +
                     pow(wp_ip_.at(new_idx).y - ypos, 2) +
                     pow(wp_ip_.at(new_idx).z - zpos, 2) );
  }

  // Apply the same rule decrementing the idx
  // to avoid getting stuck at an outlier
  // Initialize:
  int idx_lookahead = 4;
  int back_start_idx = closest_idx + idx_lookahead;
  int decremented_idx = back_start_idx;
  while(decremented_idx > back_start_idx-idx_lookahead) {
    try {
      new_dist = sqrt(pow(wp_ip_.at(decremented_idx).x - xpos, 2) +
                      pow(wp_ip_.at(decremented_idx).y - ypos, 2) +
                      pow(wp_ip_.at(decremented_idx).z - zpos, 2));
    }
    catch (const std::out_of_range& ex)
    {
      // Query index + lookahead index bigger than
      // size of waypoint vector Thus, lookahead index is ignored
      break;
    }

    if (new_dist <= closest_dist) {
      closest_dist = new_dist;
      closest_idx = decremented_idx;
    }

    decremented_idx -= 1;

    if (decremented_idx<0) // beginning of path
      break;
  }
  return closest_idx;
}

// Function to get set of full waypoints between start_idx and start_idx+lookahead
vector<SingleFullWaypoint> Waypoint::getSetFullWpip(int start_idx, double lookahead) const
{
  // ensure that start iterator points to valid vector entry
  if(start_idx >= wp_ip_.size()) {
    std::cout << "[lonomy_path_controller]: In Function getPoints() access to element outside of vector"<< std::endl;
  }

  // Initialize
  vector<SingleFullWaypoint> wp_ip_extracted;
  double dist = 0;
  double x_prev = wp_ip_.at(start_idx).x;
  double y_prev = wp_ip_.at(start_idx).y;
  double z_prev = wp_ip_.at(start_idx).z;

  // Extract points
  for(auto it = std::begin(wp_ip_)+start_idx; it != std::end(wp_ip_); ++it) {
    // extract next waypoint
    wp_ip_extracted.push_back(*it);
    // check if lookahead is reached
    dist += sqrt( pow((*it).x-x_prev, 2) + pow((*it).y-y_prev, 2) + pow((*it).z-z_prev, 2) );
    x_prev = (*it).x;
    y_prev = (*it).y;
    z_prev = (*it).z;
    if(dist>=lookahead)
      break;
  }

  return wp_ip_extracted;
}

// Function to get set of 3D waypoints between start_idx and start_idx+lookahead
vector<geometry_msgs::Pose> Waypoint::getSet3DWpip(int start_idx, double lookahead) const
{
  // ensure that start iterator points to valid vector entry
  if(start_idx >= wp_ip_.size()) {
    std::cout << "[lonomy_path_controller]: In Function getPoints() access to element outside of vector"<< std::endl;
  }

  // Initialize
  vector<geometry_msgs::Pose> wp_ip_extracted;
  double dist = 0;
  double x_prev = wp_ip_.at(start_idx).x;
  double y_prev = wp_ip_.at(start_idx).y;
  double z_prev = wp_ip_.at(start_idx).z;

  // Extract points
  geometry_msgs::Pose Pose3D_temp;
  for(auto it = std::begin(wp_ip_)+start_idx; it != std::end(wp_ip_); ++it) {
    // extract next waypoint
    Pose3D_temp.position.x = it->x;
    Pose3D_temp.position.y = it->y;
    Pose3D_temp.position.z = it->z;
    Pose3D_temp.orientation.x = it->qx;
    Pose3D_temp.orientation.y = it->qy;
    Pose3D_temp.orientation.z = it->qz;
    Pose3D_temp.orientation.w = it->qw;
    wp_ip_extracted.push_back(Pose3D_temp);
    // check if lookahead is reached
    dist += sqrt( pow((*it).x-x_prev, 2) + pow((*it).y-y_prev, 2) + pow((*it).z-z_prev, 2) );
    x_prev = (*it).x;
    y_prev = (*it).y;
    z_prev = (*it).z;
    if(dist>=lookahead)
      break;
  }

  return wp_ip_extracted;
}

// Function to get set of 2D waypoints between start_idx and start_idx+lookahead
vector<geometry_msgs::Pose2D> Waypoint::getSet2DWpip(int start_idx, double lookahead) const
{
  // ensure that start iterator points to valid vector entry
  if(start_idx >= wp_ip_.size()) {
    std::cout << "[lonomy_path_controller]: In Function getPoints() access to element outside of vector"<< std::endl;
  }

  // Initialize
  vector<geometry_msgs::Pose2D> wp_ip_extracted;
  double dist = 0;
  double x_prev = wp_ip_.at(start_idx).x;
  double y_prev = wp_ip_.at(start_idx).y;
  double z_prev = wp_ip_.at(start_idx).z;

  // Extract points
  geometry_msgs::Pose2D pose2D_temp;
  for(auto it = std::begin(wp_ip_)+start_idx; it != std::end(wp_ip_); ++it) {
    // extract next waypoint
    pose2D_temp.x = it->x;
    pose2D_temp.y = it->y;
    // get yaw
    geometry_msgs::Quaternion quat;
    quat.x = it->qx;
    quat.y = it->qy;
    quat.z = it->qz;
    quat.w = it->qw;
    pose2D_temp.theta = tf2::getYaw(quat);

    wp_ip_extracted.push_back(pose2D_temp);
    // check if lookahead is reached
    dist += sqrt( pow((*it).x-x_prev, 2) + pow((*it).y-y_prev, 2) + pow((*it).z-z_prev, 2) );
    x_prev = (*it).x;
    y_prev = (*it).y;
    z_prev = (*it).z;
    if(dist>=lookahead)
      break;
  }

  return wp_ip_extracted;
}

// TODO: [lipascal] write the functions
// getSetFullWp()
// getSet3DWp()
// getSet2DWp()

} // namespace lonomy_path_controller