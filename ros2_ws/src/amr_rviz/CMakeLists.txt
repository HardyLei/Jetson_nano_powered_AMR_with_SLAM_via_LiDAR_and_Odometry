cmake_minimum_required(VERSION 3.8)
project(amr_rviz)

find_package(ament_cmake REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(explore_lite_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(amr_rviz_panel SHARED src/exploration_panel.cpp)
set_target_properties(amr_rviz_panel PROPERTIES AUTOMOC ON)
target_compile_features(amr_rviz_panel PRIVATE cxx_std_17)
target_link_libraries(amr_rviz_panel Qt5::Widgets)
ament_target_dependencies(amr_rviz_panel
  amr_interfaces explore_lite_msgs pluginlib rclcpp rviz_common std_msgs)

pluginlib_export_plugin_description_file(rviz_common panels.xml)
install(TARGETS amr_rviz_panel LIBRARY DESTINATION lib)
ament_package()
