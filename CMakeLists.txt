cmake_minimum_required(VERSION 2.8.3)
project(baxter_moveit_stomp_trac_ik_config)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
             COMPONENTS
             moveit_core
             moveit_ros_planning
             moveit_ros_planning_interface
             pluginlib
             cmake_modules
             geometric_shapes
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pr2_moveit_tutorials
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
)
find_package(Eigen REQUIRED)

###########
## Build ##
###########  

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(baxter_moveit_stomp_trac_ik_tutorial src/baxter_moveit_stomp_trac_ik_tutorial.cpp)
target_link_libraries(baxter_moveit_stomp_trac_ik_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(baxter_moveit_stomp_trac_ik src/baxter_moveit_stomp_trac_ik.cpp)
target_link_libraries(baxter_moveit_stomp_trac_ik ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(baxter_moveit_stomp_trac_ik_controller src/baxter_moveit_stomp_trac_ik_controller.cpp)
target_link_libraries(baxter_moveit_stomp_trac_ik_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(give_goal_and_move src/give_goal_and_move.cpp)
target_link_libraries(give_goal_and_move ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(joint_angles src/joint_angles.cpp)
target_link_libraries(joint_angles ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS joint_angles DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(baxter_moveit_pick_and_place src/baxter_moveit_pick_and_place.cpp)
target_link_libraries(baxter_moveit_pick_and_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS baxter_moveit_pick_and_place DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(pick_place src/pick_place.cpp)
target_link_libraries(pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS pick_place DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(motion_planning src/motion_planning.cpp)
target_link_libraries(motion_planning ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS motion_planning DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
