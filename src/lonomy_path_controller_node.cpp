/*
 * lonomy_path_controller.h
 *
 *  Created on: 31.03.2022
 *  
 *  Copyright 2021 Pascal Lieberherr (ETH Zurich)
 *                 [lipascal@ethz.ch]
 *  All rights reserved.
 * 
 */

#include <ros/ros.h>
#include "lonomy_path_controller/lonomy_path_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lonomy_path_controller_node"); //must be unique
    ros::NodeHandle node_handle("~");

    lonomy_path_controller::PathController lonomy_path_controller(node_handle);

    ros::spin(); // async spinner, http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
    return 0;
}