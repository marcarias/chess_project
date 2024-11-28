cmake_minimum_required(VERSION 2.8.3)
project(chesslab_setup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  tf2
  tf2_ros
  robotiq_2f_model
  ur_description
  ur_e_description
  roslib
  ur3ik
  downward_ros
  #kautham
)

find_package(
   kautham REQUIRED
)

if(KAUTHAM_INCLUDE_DIR)
    message(STATUS "Kautham found and its include dir is $(KAUTHAM_INCLUDE_DIR")
endif()

add_service_files(
  FILES
  setrobconf.srv
  setobjpose.srv
  attachobs2robot.srv
  dettachobs.srv
  ffplan.srv
  ik.srv
)

add_message_files(
  FILES
  chessmap.msg
  iksol.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_basics_pkg
   CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime robotiq_2f_model ur_description ur_e_description ur3ik downward_ros #kautham
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(chesslab_setup_node src/chesslab_setup_node.cpp)
add_executable(demo_attach src/demo_attach.cpp)
add_executable(demo_detach src/demo_detach.cpp)
add_executable(demo_movepiece src/demo_movepiece.cpp)
add_executable(demo_collision src/demo_collision.cpp)
add_executable(demo_planmotions src/demo_planmotions.cpp)
add_executable(demo_ik src/demo_ik.cpp)

add_dependencies(chesslab_setup_node ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_attach ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_detach ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_movepiece ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_collision ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_planmotions ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_ik ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(chesslab_setup_node ${catkin_LIBRARIES})
target_link_libraries(demo_attach ${catkin_LIBRARIES})
target_link_libraries(demo_detach ${catkin_LIBRARIES})
target_link_libraries(demo_movepiece ${catkin_LIBRARIES})
target_link_libraries(demo_collision ${catkin_LIBRARIES})
target_link_libraries(demo_planmotions ${catkin_LIBRARIES})
target_link_libraries(demo_ik ${catkin_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############
