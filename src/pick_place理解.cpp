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


 // 这个睡眠只允许Rviz来
  sleep(20.0);
  //Move Group 界面的建立：
  // //建立MoveGroup类的对象group，参数是right_arm
  moveit::planning_interface::MoveGroup group("right_arm");

 //使用PlanningSceneInterface类的对象，与世界沟通
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  //创立一个发布者发布消息
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
//定义了一个相应的类的对象，用于发布消息
  moveit_msgs::DisplayTrajectory display_trajectory;

  //打印这个机器人的参考框架的名称。 
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // 打印此组的末端效应器链接的名称。
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

 //设置一个目标位置姿态
  geometry_msgs::Pose target_pose1;//定义了一个Pose类的对象 target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.7;
  target_pose1.position.z = 1.0;
  group.setPoseTarget(target_pose1);//利用group的成员函数setPoseTarget,为group类的PoseTarget变量赋值。


  //现在我们利用ros进行规划，但这  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");一步只是规划，并没有真正的让他动
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);  //利用MoveGroup中的成员函数.plan()进行规划，成功返回true

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED"); //没成功打印FAILED   
  /* Sleep to give Rviz time to visualize the plan. *///睡眠5s,等待RVIZ去可视化
  sleep(5.0);

 //RVIZ可视化
  if (1)  //为啥要这么写？
  {
    ROS_INFO("Visualizing plan 1 (again)");    
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);//将my_plan对应的trajectory_赋值给display_trajectory
    display_publisher.publish(display_trajectory);  //将消息发出去
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);
  }
  
  //移动到一个姿势目标
 
   // *在使用真实的机器人时，取消注释下方的注释* /
  //  group.move（）

  //规划关节空间的目标点
  std::vector<double> group_variable_values;
  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);//将现在的关节位置向量赋值给向量group_variable_values
  

  //改变其中的一个进行规划
  group_variable_values[0] = -1.0;  
  group.setJointValueTarget(group_variable_values);
  success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(5.0);

//带有路径约束的规划
  moveit_msgs::OrientationConstraint ocm;  
  ocm.link_name = "r_wrist_roll_link";  
  ocm.header.frame_id = "base_link";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;
  
 //把它置为路径约束
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);  
  group.setPathConstraints(test_constraints);
//重新设定初始状态，可为什么要这么做？
  robot_state::RobotState start_state(*group.getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  const robot_state::JointModelGroup *joint_model_group =
                  start_state.getJointModelGroup(group.getName());//要看具体的实现细节,这里没看懂
  start_state.setFromIK(joint_model_group, start_pose2);
  group.setStartState(start_state);
  
//进行规划
  group.setPoseTarget(target_pose1);
  success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 3 (constraints) %s",success?"":"FAILED"); //这里如何实现RVIZ可视化？
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(10.0);

  // When done with the path constraint be sure to clear it.//清除路径约束
  group.clearPathConstraints();
//笛卡尔路径
//首先定义并waypoints
  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose target_pose3 = start_pose2;
  target_pose3.position.x += 0.2;
  target_pose3.position.z += 0.2;
  waypoints.push_back(target_pose3);  // up and out

  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3);  // left

  target_pose3.position.z -= 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // down and right (back to start)

//得到笛卡尔路径
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold //如何理解这个定义？
                                               trajectory);

  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);    //fraction 返回值的什么意思？    
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(15.0);

//首先添加物体的信息
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group.getPlanningFrame();//与group类建立联系

 //识别物体的id
  collision_object.id = "box1";

 //定义一个箱子，并且给出相应的外形信息
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;

//箱子的位置
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x =  0.6;
  box_pose.position.y = -0.4;
  box_pose.position.z =  1.2;

//将上述的信息赋值给collison_object;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

//建立一个向量，将元素压入
  std::vector<moveit_msgs::CollisionObject> collision_objects;  
  collision_objects.push_back(collision_object);  

  //将物体加入机器人的世界
  ROS_INFO("Add an object into the world");  
  planning_scene_interface.addCollisionObjects(collision_objects);
  
  /* Sleep so we have time to see the object in RViz */
  sleep(2.0);

//设置规划的时间
  group.setPlanningTime(10.0);


 //进行规划
  group.setStartState(*group.getCurrentState());
  group.setPoseTarget(target_pose1);
  success = group.plan(my_plan);


  ROS_INFO("Visualizing plan 5 (pose goal move around box) %s",
    success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(10.0);
  

 //现在，我们将碰撞对象附加到机器人。
  ROS_INFO("Attach the object to the robot");  
  group.attachObject(collision_object.id);  
  /* Sleep to give Rviz time to show the object attached (different color). */
  sleep(4.0);


  //现在，我们将碰撞对象与机器人分离。
  ROS_INFO("Detach the object from the robot");  
  group.detachObject(collision_object.id);  
  /* Sleep to give Rviz time to show the object detached. */
  sleep(4.0);


 //从世界中移除物体
  ROS_INFO("Remove the object from the world");  
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);  
  planning_scene_interface.removeCollisionObjects(object_ids);
  /* Sleep to give Rviz time to show the object is no longer there. */
  sleep(4.0);

//双手规划
  moveit::planning_interface::MoveGroup two_arms_group("arms");

  two_arms_group.setPoseTarget(target_pose1, "r_wrist_roll_link");

  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.w = 1.0;
  target_pose2.position.x = 0.7;
  target_pose2.position.y = 0.15;
  target_pose2.position.z = 1.0;

  two_arms_group.setPoseTarget(target_pose2, "l_wrist_roll_link");

  // Now, we can plan and visualize
  moveit::planning_interface::MoveGroup::Plan two_arms_plan;
  two_arms_group.plan(two_arms_plan);
  sleep(4.0);

// END_TUTORIAL

  ros::shutdown();  
  return 0;
}



在Rviz中，我们应该可以看到以下内容（每个步骤之间将延迟5-10秒）：
机器人将其右臂移动到向右前方的姿势目标。
机器人从1重复相同的动作。
机器人将右臂移动到右侧的联合目标。
机器人将其右臂移回到新的姿势目标，同时保持末端执行器水平。
机器人沿着所需的笛卡尔路径移动其右臂（三角形向上，向前，向左，向下+向后）。
一个框对象被添加到右臂右侧的环境中。乙
机器人将其右臂移动到姿势目标，避免与盒子碰撞。
物体附着在手腕上（其颜色将变为紫色/橙色/绿色）。
物体与腕部分离（其颜色将变回绿色）。
该对象从环境中删除。
