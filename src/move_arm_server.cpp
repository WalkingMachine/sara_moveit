////
//// Created by philippe on 02/07/17.
////

#include <sara_moveit/move_arm_server.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_planners_ompl/OMPLDynamicReconfigureConfig.h>

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
        double dist = 100;
        while ( dist > 0.05 ) {
            auto Tpos = group.getPoseTarget().pose.position;
            auto Cpos = group.getCurrentPose().pose.position;
            dist = sqrt((Tpos.x - Cpos.x) * (Tpos.x - Cpos.x)
                        + (Tpos.y - Cpos.y) * (Tpos.y - Cpos.y)
                        + (Tpos.z - Cpos.z) * (Tpos.z - Cpos.z));
            usleep(1000000);
        }

        //auto state = group.getCurrentState();
        //state->
        //resp.success = (char)((group.move().val == 1)?1:0);
        //group.stop();
        //resp.success = (char)((group.move().val == 1)?1:0);
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