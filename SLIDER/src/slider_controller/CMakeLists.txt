cmake_minimum_required(VERSION 2.8.3)
project(slider_controller)

find_package(gazebo REQUIRED)
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_filters
  message_generation
  eigen_conversions
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

find_library(PINOCCHIO_LIBRARY NAMES pinocchio)
find_library(HPP_FCL_LIBRARY NAMES hpp-fcl)


add_message_files(
  FILES
  FloatArray.msg
  #Controller_paramaters.msg
)

add_service_files(
FILES
Controller_paramaters.srv
Weight_Ref.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime 
  DEPENDS pinocchio 
)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${rospy_INCLUDE_DIRS}
  ${ros_lib_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
#  scripts/DQN.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  "/usr/include/eigen3/"
  "/usr/local/include"
  "/opt/openrobots/include/"
  "/usr/include/python2.7"
) 

SET(THIS_COM "${PROJECT_SOURCE_DIR}")

include_directories(SYSTEM ${THIS_COM}/third-party/ParamHandler/)

add_subdirectory(third-party)

message(IMPORTANT MESSAGE "${pinocchio_CFLAGS_OTHER}")

FILE(GLOB folder_source ./src/*.cc)
FILE(GLOB folder_header ./src/*.h)

SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "-std=c++11 -pthread")
SET(GCC_COVERAGE_COMPILE_FLAGS " -O3 -std=c++11") # -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR -DBOOST_MPL_LIMIT_LIST_SIZE=30 -DPINOCCHIO_WITH_URDFDOM -std=c++11 -lprofilingS -p -lp -g -p -O3 -fopenmp
add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})

LINK_LIBRARIES(
boost_system
urdfdom_model
)

add_executable(slider_controller src/slider_controller.cc ${folder_source} ${folder_header})
target_compile_definitions(slider_controller PRIVATE ${pinocchio_CFLAGS_OTHER})

target_link_libraries(slider_controller PRIVATE libqpOASES.so ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${LIBRARIES} ${pinocchio_LIBRARIES} dynacore_param_handler) # pinocchio::pinocchio


#add_executable(slider_controller_jump src/slider_controller_jump.cc ${folder_source} ${folder_header})
#target_compile_definitions(slider_controller_jump PRIVATE ${pinocchio_CFLAGS_OTHER})

#target_link_libraries(slider_controller_jump PRIVATE libqpOASES.so ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${LIBRARIES} ${pinocchio_LIBRARIES} dynacore_param_handler) # pinocchio::pinocchioadd_executable(slider_planner src/step_planner_main.cpp src/step_planner.cpp src/step_planner_data.cpp)
#target_link_libraries(slider_planner PRIVATE ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${LIBRARIES})

#add_executable(slider_planner src/MPC/motion_planner112.cc src/MPC/c_load.cc src/MPC/planner_xy.cc src/MPC/planner_z.cc)

add_executable(slider_planner src/MPC/motion_planner.cc src/MPC/dynamics.cc)
target_link_libraries(slider_planner PRIVATE libqpOASES.so ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${LIBRARIES})

#add_executable(parameter_selftuning scripts/DQN.py)
#target_link_libraries(parameter_selftuning PRIVATE ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${LIBRARIES})
 
