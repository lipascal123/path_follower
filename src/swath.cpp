/*
 * swath.cpp
 *
 *  Created on: 15.08.2022
 *
 *  Copyright 2022 Pascal Lieberherr (ETH Zurich)
 *                 [lipascal@ethz.ch]
 *  All rights reserved.
 *
*/

#include "lonomy_path_controller/swath.h"

namespace lonomy_path_controller {

using std::vector;

// operator overloading for some boost::geometry objects
std::ostream &operator<<(std::ostream& o, const Point& pt) {
  o << pt.get<0>() << ", " << pt.get<1>() << "\n";
  return o;
}

std::ostream &operator<<(std::ostream& o, const Ring& ring) {
  for (Point pt: ring) {
    o << pt;
  }
  return o;
}

std::ostream &operator<<(std::ostream& o, const Polygon& poly) {
  auto r = poly.outer();
  o << r;
  return o;
}

std::ostream &operator<<(std::ostream& o, const MultiPolygon& polys) {
  for (Polygon poly: polys) {
    o << poly << "\n";
  }
  return o;
}

// To compute the footprint at a given path pose
vector<geometry_msgs::Point> transformFootprintAtPathPose(const vector<geometry_msgs::Point>& bl_t_bl_footprintedges,
                                                           const geometry_msgs::Pose& pose) {
  // TODO [lipascal] Use eigen to make it more compact
  geometry_msgs::TransformStamped T_world_pathpoint;
  T_world_pathpoint.transform.translation.x = pose.position.x;
  T_world_pathpoint.transform.translation.y = pose.position.y;
  T_world_pathpoint.transform.translation.z = pose.position.z;
  T_world_pathpoint.transform.rotation = pose.orientation;

  // Compute oriented footprint
  vector<geometry_msgs::Point> world_t_world_footprintedges;
  for (auto bl_t_bl_footprintedgei: bl_t_bl_footprintedges) {
    geometry_msgs::Point world_t_world_footprintedgei;
    // TODO [lipascal] use eigen here and then remove include tf2_geometry_msgs.h
    tf2::doTransform(bl_t_bl_footprintedgei, world_t_world_footprintedgei, T_world_pathpoint);
    world_t_world_footprintedges.push_back(world_t_world_footprintedgei);
  }

  return world_t_world_footprintedges;
}

// To compute the swath along a given path
MultiPolygon computeSwath(const vector<geometry_msgs::Point>& footprint,
                          const vector<geometry_msgs::Pose>& predicted_path) {
  MultiPolygon swath_poly;
  for (auto pose: predicted_path) {
    vector<geometry_msgs::Point> world_t_world_footprintedgesi = transformFootprintAtPathPose(footprint, pose);

    Polygon footprint_poly = {{{world_t_world_footprintedgesi.at(0).x, world_t_world_footprintedgesi.at(0).y},
                               {world_t_world_footprintedgesi.at(1).x, world_t_world_footprintedgesi.at(1).y},
                               {world_t_world_footprintedgesi.at(2).x, world_t_world_footprintedgesi.at(2).y},
                               {world_t_world_footprintedgesi.at(3).x, world_t_world_footprintedgesi.at(3).y}}};

    boost::geometry::correct(footprint_poly);
    MultiPolygon swath_poly_prev = swath_poly;
    swath_poly.clear();
    boost::geometry::union_(footprint_poly, swath_poly_prev, swath_poly);
    boost::geometry::correct(swath_poly);
  }
  // Simplify joined structure
  MultiPolygon swath_poly_simpl;
  boost::geometry::simplify(swath_poly, swath_poly_simpl, 0.05);

  return swath_poly_simpl;
}

// To convert to polygon stamped message
vector<geometry_msgs::PolygonStamped> convertToPolygonStamped(const MultiPolygon& polygons, const std::string& frame,
                                                              const ros::Time& time) {
  int size = polygons.size();
  vector<geometry_msgs::PolygonStamped> poly_msg_vec;
  for (int i = 0; i < size; ++i) {
    geometry_msgs::PolygonStamped poly_msg;
    poly_msg.header.frame_id = frame;
    poly_msg.header.stamp = time;
    for (Polygon poly: polygons) {
      auto ring = poly.outer();
      for (Point pt: ring) {
        geometry_msgs::Point32 point32;
        point32.x = pt.x();
        point32.y = pt.y();
        point32.z = 0;
        poly_msg.polygon.points.push_back(point32);
      }
    }
    poly_msg_vec.push_back(poly_msg);
  }
  return poly_msg_vec;


}

} // namespace lonomy_path_controller