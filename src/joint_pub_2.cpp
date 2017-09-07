#include <ros/ros.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();
 // ros::init(argc, argv, "baxter_right_arm_joint_pub");
 // ros::NodeHandle n;
 // ros::Publisher right_cmd_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1);
  // publish at at least 5 Hz, or else Baxter switches back to Position mode and holds position
 // ros::Rate loop_rate(100);
  moveit_msgs::DisplayTrajectory cmd;
  moveit::planning_interface::MoveGroup group("right_arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;
  // command in velocity mode

  // command joints in the order shown in baxter_interface
  cmd.names.push_back("right_s0");
  cmd.names.push_back("right_s1");
  cmd.names.push_back("right_e0");
  cmd.names.push_back("right_e1");
  cmd.names.push_back("right_w0");
  cmd.names.push_back("right_w1");
  cmd.names.push_back("right_w2");
  // set your calculated velocities
  cmd.command.resize(cmd.names.size());
    cmd.command[1] =-0.6699952259595108;
    cmd.command[2] =1.030009435085784;
    cmd.command[3] =0.4999997247485215;
    cmd.command[4] =-0.189968899785275;
    cmd.command[5] =1.9400238130755056;
    cmd.command[6] =0.08000397926829805;
    cmd.command[7] =-0.9999781166910306;
 
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
  /* Sleep to give Rviz time to visualize the plan. */
  ros::Rate loop_rate(10);

 while(ros::ok()){
    ROS_INFO("Visualizing plan 1 (again)");    
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher.publish(display_trajectory);
    ros::spinOnce();
    /* Sleep to give Rviz time to visualize the plan. */
    loop_rate.sleep();
  }
  
  // Moving to a pose goal
  // Uncomment below line when working with a real robot
  //group.move();

  //ros::shutdown();  
  return 0;
}
