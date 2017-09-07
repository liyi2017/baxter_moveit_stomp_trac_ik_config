#include <ros/ros.h>
#include <iostream>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>


using namespace std;

boost::shared_ptr<moveit::planning_interface::MoveGroup> group;


void testPlaceTarget(string namedTarget ,int sleepTime)
{
    ros::AsyncSpinner spinner(1);
    spinner.start();
    group->setMaxVelocityScalingFactor(0.5);
    group->setMaxAccelerationScalingFactor(0.5);
    group->setNamedTarget(namedTarget);       //group->setPoseTarget(goalPose);
    group->move();
    sleep(sleepTime);
    spinner.stop();
}

void testTargeByJointAngel(int sleepTime)
{
    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroup::Plan planner;
    bool is_success;
    vector<double> joint_angles(7);
/*    joint_angles[0]=13.16/180.0*M_PI;
    joint_angles[1]=-83.80/180.0*M_PI;
    joint_angles[2]=-97.89/180.0*M_PI;
    joint_angles[3]=-77.18/180.0*M_PI;
    joint_angles[4]=82.11/180.0*M_PI;
    joint_angles[5]=-34.23/180.0*M_PI;*/
    joint_angles[0]=-0.6699952259595108;
    joint_angles[1]=1.030009435085784;
    joint_angles[2]=0.4999997247485215;
    joint_angles[3]=-0.189968899785275;
    joint_angles[4]=1.9400238130755056;
    joint_angles[5]=0.08000397926829805;
    joint_angles[6]=-0.9999781166910306;
    group->setJointValueTarget(joint_angles);
    is_success=group->plan(planner);
    if(is_success)
    {  group->move(); }
     else
    {  cout<<"Move to place pose: Planning fail!"<<endl; }
    sleep(sleepTime);
    spinner.stop();
}

int main(int argc, char **argv)
{
     ros::init(argc,argv,"test");
     ros::NodeHandle nh;
     group.reset();
     group = boost::make_shared<moveit::planning_interface::MoveGroup>("right_arm");
     group->setPoseReferenceFrame("/base");
     group->setMaxVelocityScalingFactor(0.5);
     group->setMaxAccelerationScalingFactor(0.5);

     //use assistance set pose to run robot   --method 1
//     testPlaceTarget("look_for_targets",1);
     //testPlaceTarget("place_target",0.2);
     //testPlaceTarget("look_for_targets",0.2);

     //use joint angles to run robot          --method 2
     testTargeByJointAngel(0.2);



     return 0;
}
