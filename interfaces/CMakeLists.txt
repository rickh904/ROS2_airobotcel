cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 1. VIND DE BENODIGDE ROS2 PACKAGES ---
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)  # Nodig voor Point en Quaternion

# --- 2. DE AUTOMATISCHE INTERFACE GENERATOR AANKOPPELEN ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/AutoSort.action"
  "action/GoHome.action"
  "action/ManipulatorTask.action"
  "srv/CoordRobot.srv"                 # <--- TOEGEVOEGD: Jouw nieuwe servicebestand
  DEPENDENCIES
    std_msgs
    geometry_msgs                      # Zorgt dat de types herkend worden
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
