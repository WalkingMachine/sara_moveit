////
//// Created by philippe on 02/07/17.
////

#ifndef PROJECT_MOVE_ARM_ACTION_SERVER_H
#define PROJECT_MOVE_ARM_ACTION_SERVER_H


#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit_msgs/Grasp.h>
#include <sara_moveit/move.h>
#include <moveit/planning_interface/planning_interface.h>

bool move( sara_moveit::moveRequest &req, sara_moveit::moveResponse &resp );
int main(int argc, char **argv);


//moveit::planning_interface::MoveGroupInterface group("right_arm");
//class move_arm_action_server {
//
//};


#endif //PROJECT_MOVE_ARM_ACTION_SERVER_H
