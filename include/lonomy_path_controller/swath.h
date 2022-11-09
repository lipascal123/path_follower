/*
 *  swath.h
 *
 *  Created on: 09.08.2022
 *
 *  Copyright 2022 Pascal Lieberherr (ETH Zurich)
 *                 [lipascal@ethz.ch]
 *  All rights reserved.
*/

/*
 *  swath.h
 *
 *  Functions in this file make use of the boost geometry library in order
 *  to implement the computation of the swath. The swath is the
 *  area which needs to be checked for collision. Based on a path
 *  and footprint the swath can be computed through unions of polygons.
 *
*/

#pragma once

// boost includes
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/union.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
// ros includes
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace lonomy_path_controller {

using std::vector;

// typedefs
typedef boost::geometry::model::d2::point_xy<double>     Point;
typedef boost::geometry::model::polygon<Point,false>     Polygon; // false indicates counterclockwise
typedef boost::geometry::ring_type<Polygon>::type        Ring;
typedef boost::geometry::model::multi_polygon<Polygon>   MultiPolygon;

// operator overloading for some boost::geometry objects
std::ostream& operator << (std::ostream& o, const Point& pt);

std::ostream& operator << (std::ostream& o, const Ring& ring);

std::ostream& operator << (std::ostream& o, const Polygon& poly);

std::ostream& operator << (std::ostream& o, const MultiPolygon & polys);

/*!
* Transform vector of footprint edges with T_world_pathpose
* @param bl_t_bl_footprintedges vector of footprint edges in base_link frame.
* @param T_world_pathpose the transformation to be applied
* @return transformed footprint edge points
*/
// TODO [lipascal] could this be done with e.g. eigen?
vector<geometry_msgs::Point> transformFootprintAtPathPose(const vector<geometry_msgs::Point>& bl_t_bl_footprintedges,
                                                          const geometry_msgs::Pose& T_world_pathpose);

/*!
* Computes the swath given a footprint and a path
* @param footprint that will be propagated along the path
* @param predicted_path the path along which the swath is computed
* @return A multi polygon representing the swath
*/
MultiPolygon computeSwath(const vector<geometry_msgs::Point>& footprint, const vector<geometry_msgs::Pose>& predicted_path);

/*!
* Conversion from boost geometry polygon to ros polygon msgs
* @param polygons to be converted into polygon msgs
* @param frame in which polygon edge points are defined
* @param time for the time stamp of the message
* @return vector of polygon stamped ros messages
*/
vector<geometry_msgs::PolygonStamped> convertToPolygonStamped(const MultiPolygon& polygons, const std::string& frame,
                                                              const ros::Time& time);

} // namespace lonomy_path_controller

