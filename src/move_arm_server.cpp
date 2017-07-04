////
//// Created by philippe on 02/07/17.
////
//
#include <sara_moveit/move_arm_server.h>
//
//
//bool move_to( geometry_msgs::PoseConstPtr &pose )
//{
//    std::vector<moveit_msgs::Grasp> grasps;
//
//    geometry_msgs::PoseStamped p;
//    p.header.frame_id = "base_footprint";
//    p.pose.position.x = 0.34;
//    p.pose.position.y = -0.7;
//    p.pose.position.z = 0.5;
//    p.pose.orientation.x = 0;
//    p.pose.orientation.y = 0;
//    p.pose.orientation.z = 0;
//    p.pose.orientation.w = 1;
//    moveit_msgs::Grasp g;
//    g.grasp_pose = p;
//
//    g.pre_grasp_approach.direction.vector.x = 1.0;
//    g.pre_grasp_approach.direction.header.frame_id = "r_wrist_roll_link";
//    g.pre_grasp_approach.min_distance = 0.2;
//    g.pre_grasp_approach.desired_distance = 0.4;
//
//    g.post_grasp_retreat.direction.header.frame_id = "base_footprint";
//    g.post_grasp_retreat.direction.vector.z = 1.0;
//    g.post_grasp_retreat.min_distance = 0.1;
//    g.post_grasp_retreat.desired_distance = 0.25;
//
//    openGripper(g.pre_grasp_posture);
//
//    closedGripper(g.grasp_posture);
//
//    grasps.push_back(g);
//    group.setSupportSurfaceName("table");
//    group.pick("part", grasps);
//
//
//
//    return true;
//}
//
int main(int argc, char **argv)
{
//
//    ros::init(argc, argv, "move_to_server");
//    ros::AsyncSpinner spinner(1);
//    spinner.start();
//
//    ros::NodeHandle nh;
//    ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
//    ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);
//
//    ros::WallDuration(1.0).sleep();
//
//
//    moveit::planning_interface::MoveGroupInterface group("right_arm");
//    group.setPlanningTime(45.0);
//
//    moveit_msgs::CollisionObject co;
//    co.header.stamp = ros::Time::now();
//    co.header.frame_id = "base_footprint";
//
//    // remove pole
//    co.id = "pole";
//    co.operation = moveit_msgs::CollisionObject::REMOVE;
//    pub_co.publish(co);
//
//    // add pole
//    co.operation = moveit_msgs::CollisionObject::ADD;
//    co.primitives.resize(1);
//    co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
//    co.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
//    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.3;
//    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
//    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
//    co.primitive_poses.resize(1);
//    co.primitive_poses[0].position.x = 0.7;
//    co.primitive_poses[0].position.y = -0.4;
//    co.primitive_poses[0].position.z = 0.85;
//    co.primitive_poses[0].orientation.w = 1.0;
//    pub_co.publish(co);
//
//    // remove table
//    co.id = "table";
//    co.operation = moveit_msgs::CollisionObject::REMOVE;
//    pub_co.publish(co);
//
//    // add table
//    co.operation = moveit_msgs::CollisionObject::ADD;
//    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.5;
//    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
//    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.35;
//    co.primitive_poses[0].position.x = 0.7;
//    co.primitive_poses[0].position.y = -0.2;
//    co.primitive_poses[0].position.z = 0.175;
//    pub_co.publish(co);
//
//    co.id = "part";
//    co.operation = moveit_msgs::CollisionObject::REMOVE;
//    pub_co.publish(co);
//
//    moveit_msgs::AttachedCollisionObject aco;
//    aco.object = co;
//    pub_aco.publish(aco);
//
//    co.operation = moveit_msgs::CollisionObject::ADD;
//    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.15;
//    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
//    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.3;
//
//    co.primitive_poses[0].position.x = 0.6;
//    co.primitive_poses[0].position.y = -0.7;
//    co.primitive_poses[0].position.z = 0.5;
//    pub_co.publish(co);
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//    ros::init(argc, argv, "sara_moveit_server");
//    ros::NodeHandle nh;
//
//    ros::ServiceServer service = nh.advertiseService("move_to_pose", move_to);
//    ROS_INFO("Ready to move.");
//    ros::spin();
//
    return 0;
}