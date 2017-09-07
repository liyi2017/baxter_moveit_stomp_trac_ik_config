#include </opt/ros/indigo/include/ros/ros.h>
#include </opt/ros/indigo/include/sensor_msgs/JointState.h>

int main(int argc, char **argv)
{
    //Initializen the ros
    ros::init(argc, argv, "baxter_right_arm_joint_pub");

    //Declare the node handle
    ros::NodeHandle node;

    //Decleare a joint state publisher
    ros::Publisher joint_pub = node.advertise<sensor_msgs::JointState>("/robot/limb/right/joint_command",10);

    //Define the joint state
    sensor_msgs::JointState joint_state;

    joint_state.name[0] = "right_e0";
    joint_state.name[1] = "right_e1";
    joint_state.name[2] = "right_s0";
    joint_state.name[3] = "right_s1";
    joint_state.name[4] = "right_w0";
    joint_state.name[5] = "right_w1";
    joint_state.name[6] = "right_w2";

    joint_state.position[0] = 0.0;
    joint_state.position[1] = 0.0;
    joint_state.position[2] = 0.0;
    joint_state.position[3] = 0.0;
    joint_state.position[4] = 0.0;
    joint_state.position[5] = 0.0;
    joint_state.position[6] = 0.0;

    //command the robot to move
   while(ros::ok()){
    //update cmd.command commands here
    joint_pub.publish(joint_state);
    ros::spinOnce();
 
   }
   return 0;
}
