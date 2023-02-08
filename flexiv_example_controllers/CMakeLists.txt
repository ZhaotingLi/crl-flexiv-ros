cmake_minimum_required(VERSION 3.5)
project(flexiv_example_controllers)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  flexiv_msgs
  hardware_interface
  joint_limits_interface
  pluginlib
  realtime_tools
  roscpp
  std_msgs
  urdf
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES flexiv_example_controllers
  CATKIN_DEPENDS
    controller_interface
    controller_manager
    flexiv_msgs
    hardware_interface
    joint_limits_interface
    pluginlib
    realtime_tools
    roscpp
    std_msgs
    urdf
)


add_library(${PROJECT_NAME}
  SHARED
  src/joint_position_example_controller.cpp
)


# add_dependencies(${PROJECT_NAME}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# #   ${PROJECT_NAME}_generate_messages_cpp
# #   ${PROJECT_NAME}_gencpp
# #   ${PROJECT_NAME}_gencfg
# )


target_include_directories(${PROJECT_NAME} PUBLIC include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
#   ${CMAKE_CURRENT_SOURCE_DIR}/rdk/include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${catkin_LIBRARIES}
)

# add_dependencies(${PROJECT_NAME}
#   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )

# prevent pluginlib from using boost
# target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
# pluginlib_export_plugin_description_file(controller_interface franka_example_controllers_plugin.xml)

# INSTALL
install(
  TARGETS
  flexiv_example_controllers
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executable
install(
  TARGETS
  flexiv_example_controllers
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
