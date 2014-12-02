cmake_minimum_required(VERSION 2.8.3)
project(gaussian_mixture_model)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  cmake_modules
)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${Boost_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  DIRECTORY msg
    FILES
    Gaussian.msg
    GaussianMixture.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gaussian_mixture_model
  CATKIN_DEPENDS message_runtime roscpp std_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(gaussian_mixture_model
  src/gmm.cpp
  )

add_executable(gaussian_mixture_model_node src/gmm_node.cpp)

add_dependencies(gaussian_mixture_model_node gaussian_mixture_model_generate_messages_cpp)

target_link_libraries(gaussian_mixture_model
  ${catkin_LIBRARIES}
  )

target_link_libraries(gaussian_mixture_model_node
  gaussian_mixture_model
  ${catkin_LIBRARIES}
  )

add_executable(gaussian_mixture_model_rviz_converter src/gmm_rviz_converter.cpp)

add_dependencies(gaussian_mixture_model_rviz_converter gaussian_mixture_model_generate_messages_cpp)

target_link_libraries(gaussian_mixture_model_rviz_converter
  ${catkin_LIBRARIES}
  )

add_executable(gaussian_mixture_model_trajectory_converter src/trajectory_converter.cpp)

add_dependencies(gaussian_mixture_model_trajectory_converter gaussian_mixture_model_generate_messages_cpp)

target_link_libraries(gaussian_mixture_model_trajectory_converter
  ${catkin_LIBRARIES}
  )

#############
## Install ##
#############


