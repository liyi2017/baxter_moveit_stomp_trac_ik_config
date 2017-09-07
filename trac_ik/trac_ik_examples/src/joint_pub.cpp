#include "ros/ros.h"
#include "baxter_core_msgs/JointCommand.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "baxter_right_arm_joint_pub");
  ros::NodeHandle n;
  ros::Publisher right_cmd_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1);
  // publish at at least 5 Hz, or else Baxter switches back to Position mode and holds position
  ros::Rate loop_rate(100);
  baxter_core_msgs::JointCommand cmd;
  // command in velocity mode
  cmd.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
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
    cmd.command[0] = -0.6699952259595108;
    cmd.command[1] = 1.030009435085784;
    cmd.command[2] = 0.4999997247485215;
    cmd.command[3] = -0.189968899785275;
    cmd.command[4] = 1.9400238130755056;
    cmd.command[5] = 0.08000397926829805;
    cmd.command[6] = -0.9999781166910306;

  std::cout<<cmd<<std::endl;
  while(ros::ok()){
    //update cmd.command commands here
    right_cmd_pub.publish(cmd);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
