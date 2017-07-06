////
//// Created by philippe on 02/07/17.
////

#include <sara_moveit/move_arm_server.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>


bool move( sara_moveit::moveRequest &req, sara_moveit::moveResponse &resp )
{
    try {
        moveit::planning_interface::MoveGroupInterface group(req.move_group);
        group.setPoseTarget(req.pose);
        return group.move();
    } catch ( __exception ex ){
        return false;
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "move_arm_server");

    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService( "move_arm", move );
    ROS_INFO("Ready to move.");
    ros::spin();

    return 0;
}