////
//// Created by philippe on 02/07/17.
////

#include <sara_moveit/move_arm_server.h>
#include <agile_grasp/Grasps.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

// taken from the grasp selection package





bool grasp( agile_grasp::GraspsConstPtr msg ){
    moveit::planning_interface::MoveGroupInterface group("RightArm");

    int length = (int)msg->grasps.size();
    for ( int i=0; i<length; i++ ){
        geometry_msgs::Pose pose;

        GraspEigen grasp(msg->grasps[i]);
        auto quat = calculateHandOrientations( grasp );
        pose.orientation.x = quat.getX();
        pose.orientation.y = quat.getY();
        pose.orientation.z = quat.getZ();
        pose.position.x = msg->grasps[i].center.x;
        pose.position.y = msg->grasps[i].center.y;
        pose.position.z = msg->grasps[i].center.z;

        moveit_msgs::Grasp Grasp;
        Grasp.grasp_pose.pose = pose;
        Grasp.pre_grasp_approach.direction.vector = msg->grasps[i].approach;
        double x = msg->grasps[i].approach.x;
        double y = msg->grasps[i].approach.y;
        double z = msg->grasps[i].approach.z;

        double dist = sqrt( x*x+y*y+z*z );
        Grasp.pre_grasp_approach.desired_distance = (float)dist;
        Grasp.post_grasp_retreat.direction.vector = msg->grasps[i].approach;
        Grasp.post_grasp_retreat.desired_distance = (float)dist;

        group.pick( "object", Grasp);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
    }
    return true;
}





bool move( sara_moveit::moveRequest &req, sara_moveit::moveResponse &resp )
{
    try {
        moveit::planning_interface::MoveGroupInterface group(req.move_group);

        ROS_INFO("Stoping previous move execution");
        group.stop();
        group.clearPoseTargets();
        group.setPoseTarget(req.pose);
        group.setGoalTolerance( 0.01 );
        group.setGoalJointTolerance(0.01);
        group.setGoalPositionTolerance(0.01);
        ROS_INFO("Starting move execution");
        group.allowReplanning( true );
        group.setNumPlanningAttempts( 5 );
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        int attempt = 1;
        do{
            attempt ++;
            if ( attempt > 2 ){
                ROS_ERROR( "target couldn't be planned" );
                resp.success = 0;
                return true;
            }
        }while(group.plan( plan ).val != 1 );
        group.execute( plan );
        resp.success = 1;
        double dist = 0;
        auto Tpos = group.getPoseTarget().pose;
        do {
            auto Cpos = group.getCurrentPose().pose;
            double dx2 = (Cpos.position.x-Tpos.position.x)*(Cpos.position.x-Tpos.position.x);
            double dy2 = (Cpos.position.y-Tpos.position.y)*(Cpos.position.y-Tpos.position.y);
            double dz2 = (Cpos.position.z-Tpos.position.z)*(Cpos.position.z-Tpos.position.z);
            dist = sqrt( dx2+dy2+dz2 );
            double Cw = Cpos.orientation.w;
            double Tw = Tpos.orientation.w;
            dx2 = (Cpos.orientation.x/Cw-Tpos.orientation.x/Tw)*(Cpos.orientation.x/Cw-Tpos.orientation.x/Tw);
            dy2 = (Cpos.orientation.y/Cw-Tpos.orientation.y/Tw)*(Cpos.orientation.y/Cw-Tpos.orientation.y/Tw);
            dz2 = (Cpos.orientation.z/Cw-Tpos.orientation.z/Tw)*(Cpos.orientation.z/Cw-Tpos.orientation.z/Tw);
            dist += sqrt( dx2+dy2+dz2 );
            usleep(100000);
        }while ( dist > 0.05 );

        ROS_INFO("Move result: %d",resp.success);
    } catch ( __exception ex ){
        resp.success = 0;
    }
    return true;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "move_arm_server");

    ros::AsyncSpinner sp( 2 );
    sp.start();

    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService( "move_arm", move );
    ROS_INFO("Ready to move.");
    //ros::spin();
    while ( ros::ok()){}

    return 0;
}








tf::Quaternion calculateHandOrientations(const GraspEigen& grasp)
{
    // calculate first hand orientation
    Eigen::Matrix3d R = Eigen::MatrixXd::Zero(3, 3);
    R.col(0) = -1.0 * grasp.approach_;
    R.col(1) = grasp.axis_;
    R.col(2) << R.col(0).cross(R.col(1));

    // rotate by 180deg around the grasp approach vector to get the "opposite" hand orientation
    Eigen::Transform<double, 3, Eigen::Affine> T(Eigen::AngleAxis<double>(3.14159, grasp.approach_));

    // convert Eigen rotation matrices to TF quaternions and normalize them
    tf::Matrix3x3 TF1;
    tf::matrixEigenToTF(R, TF1);

    tf::Quaternion quat1;
//    TF1.getRotation(quat1);
    quat1.normalize();

    return quat1;
}