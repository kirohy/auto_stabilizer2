cmake_minimum_required(VERSION 3.0.2)
project(auto_stabilizer_choreonoid_sample)

find_package(catkin REQUIRED COMPONENTS
  jvrc_models
  hrpsys_choreonoid_tutorials
  choreonoid_ros_ext_plugin)

catkin_package()

configure_file(${PROJECT_SOURCE_DIR}/config/JAXON_JVRC.cnoid.in ${PROJECT_SOURCE_DIR}/config/JAXON_JVRC.cnoid @ONLY)
