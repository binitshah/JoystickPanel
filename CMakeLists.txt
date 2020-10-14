cmake_minimum_required(VERSION 3.5)
project(joystick_panel)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)

link_directories(${ament_cmake_LIBRARY_DIRS})

add_definitions(-DQT_NO_KEYWORDS)

add_library(${PROJECT_NAME} SHARED
  include/${PROJECT_NAME}/joystick_widget.hpp
  include/${PROJECT_NAME}/joystick_panel.hpp
  src/joystick_widget.cpp
  src/joystick_panel.cpp
  ${MOC_FILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${rvizCommon_DIR}
  ${ament_cmake_INCLUDE_DIRS}
  ${rviz2_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
  ${FREETYPE_INCLUDE_DIRS}
  ${Qt5_INCLUDE_DIRS}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  rclcpp
  rviz_common
  pluginlib
  geometry_msgs
)

ament_export_include_directories(${INCLUDE_DIRS} ${ament_cmake_INCLUDE_DIRS} include)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Qt5
  rviz_common
  geometry_msgs
  rclcpp
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
