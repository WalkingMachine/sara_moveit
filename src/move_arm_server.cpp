////
//// Created by philippe on 02/07/17.
////

#include <sara_moveit/move_arm_server.h>

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