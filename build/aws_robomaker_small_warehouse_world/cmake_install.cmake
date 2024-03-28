# Install script for directory: /home/winter24/gcamp_ros2_ws/zm_robot-ros2/aws-robomaker-small-warehouse-world

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/winter24/gcamp_ros2_ws/install/aws_robomaker_small_warehouse_world")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aws_robomaker_small_warehouse_world" TYPE DIRECTORY FILES
    "/home/winter24/gcamp_ros2_ws/zm_robot-ros2/aws-robomaker-small-warehouse-world/launch"
    "/home/winter24/gcamp_ros2_ws/zm_robot-ros2/aws-robomaker-small-warehouse-world/models"
    "/home/winter24/gcamp_ros2_ws/zm_robot-ros2/aws-robomaker-small-warehouse-world/worlds"
    "/home/winter24/gcamp_ros2_ws/zm_robot-ros2/aws-robomaker-small-warehouse-world/maps"
    "/home/winter24/gcamp_ros2_ws/zm_robot-ros2/aws-robomaker-small-warehouse-world/rviz"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aws_robomaker_small_warehouse_world/environment" TYPE FILE FILES "/home/winter24/gcamp_ros2_ws/build/aws_robomaker_small_warehouse_world/ament_cmake_environment_hooks/aws_robomaker_small_warehouse_world.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/winter24/gcamp_ros2_ws/build/aws_robomaker_small_warehouse_world/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/aws_robomaker_small_warehouse_world")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/winter24/gcamp_ros2_ws/build/aws_robomaker_small_warehouse_world/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/aws_robomaker_small_warehouse_world")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aws_robomaker_small_warehouse_world/environment" TYPE FILE FILES "/opt/ros/foxy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aws_robomaker_small_warehouse_world/environment" TYPE FILE FILES "/home/winter24/gcamp_ros2_ws/build/aws_robomaker_small_warehouse_world/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aws_robomaker_small_warehouse_world/environment" TYPE FILE FILES "/opt/ros/foxy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aws_robomaker_small_warehouse_world/environment" TYPE FILE FILES "/home/winter24/gcamp_ros2_ws/build/aws_robomaker_small_warehouse_world/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aws_robomaker_small_warehouse_world" TYPE FILE FILES "/home/winter24/gcamp_ros2_ws/build/aws_robomaker_small_warehouse_world/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aws_robomaker_small_warehouse_world" TYPE FILE FILES "/home/winter24/gcamp_ros2_ws/build/aws_robomaker_small_warehouse_world/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aws_robomaker_small_warehouse_world" TYPE FILE FILES "/home/winter24/gcamp_ros2_ws/build/aws_robomaker_small_warehouse_world/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aws_robomaker_small_warehouse_world" TYPE FILE FILES "/home/winter24/gcamp_ros2_ws/build/aws_robomaker_small_warehouse_world/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aws_robomaker_small_warehouse_world" TYPE FILE FILES "/home/winter24/gcamp_ros2_ws/build/aws_robomaker_small_warehouse_world/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/winter24/gcamp_ros2_ws/build/aws_robomaker_small_warehouse_world/ament_cmake_index/share/ament_index/resource_index/packages/aws_robomaker_small_warehouse_world")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aws_robomaker_small_warehouse_world/cmake" TYPE FILE FILES "/home/winter24/gcamp_ros2_ws/build/aws_robomaker_small_warehouse_world/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aws_robomaker_small_warehouse_world/cmake" TYPE FILE FILES
    "/home/winter24/gcamp_ros2_ws/build/aws_robomaker_small_warehouse_world/ament_cmake_core/aws_robomaker_small_warehouse_worldConfig.cmake"
    "/home/winter24/gcamp_ros2_ws/build/aws_robomaker_small_warehouse_world/ament_cmake_core/aws_robomaker_small_warehouse_worldConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aws_robomaker_small_warehouse_world" TYPE FILE FILES "/home/winter24/gcamp_ros2_ws/zm_robot-ros2/aws-robomaker-small-warehouse-world/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/winter24/gcamp_ros2_ws/build/aws_robomaker_small_warehouse_world/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
