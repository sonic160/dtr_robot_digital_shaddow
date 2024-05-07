# Install script for directory: /home/zhiguo/github_repo/dtr_robot_digital_shaddow/pc_side/catkin_ws/src/cm_listener

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zhiguo/github_repo/dtr_robot_digital_shaddow/pc_side/catkin_ws/install")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cm_listener/msg" TYPE FILE FILES "/home/zhiguo/github_repo/dtr_robot_digital_shaddow/pc_side/catkin_ws/src/cm_listener/msg/msg_cm.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cm_listener/cmake" TYPE FILE FILES "/home/zhiguo/github_repo/dtr_robot_digital_shaddow/pc_side/catkin_ws/build/cm_listener/catkin_generated/installspace/cm_listener-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/zhiguo/github_repo/dtr_robot_digital_shaddow/pc_side/catkin_ws/devel/include/cm_listener")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/zhiguo/github_repo/dtr_robot_digital_shaddow/pc_side/catkin_ws/devel/share/roseus/ros/cm_listener")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/zhiguo/github_repo/dtr_robot_digital_shaddow/pc_side/catkin_ws/devel/share/common-lisp/ros/cm_listener")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/zhiguo/github_repo/dtr_robot_digital_shaddow/pc_side/catkin_ws/devel/share/gennodejs/ros/cm_listener")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/zhiguo/github_repo/dtr_robot_digital_shaddow/pc_side/catkin_ws/devel/lib/python2.7/dist-packages/cm_listener")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/zhiguo/github_repo/dtr_robot_digital_shaddow/pc_side/catkin_ws/devel/lib/python2.7/dist-packages/cm_listener")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zhiguo/github_repo/dtr_robot_digital_shaddow/pc_side/catkin_ws/build/cm_listener/catkin_generated/installspace/cm_listener.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cm_listener/cmake" TYPE FILE FILES "/home/zhiguo/github_repo/dtr_robot_digital_shaddow/pc_side/catkin_ws/build/cm_listener/catkin_generated/installspace/cm_listener-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cm_listener/cmake" TYPE FILE FILES
    "/home/zhiguo/github_repo/dtr_robot_digital_shaddow/pc_side/catkin_ws/build/cm_listener/catkin_generated/installspace/cm_listenerConfig.cmake"
    "/home/zhiguo/github_repo/dtr_robot_digital_shaddow/pc_side/catkin_ws/build/cm_listener/catkin_generated/installspace/cm_listenerConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cm_listener" TYPE FILE FILES "/home/zhiguo/github_repo/dtr_robot_digital_shaddow/pc_side/catkin_ws/src/cm_listener/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cm_listener" TYPE PROGRAM FILES "/home/zhiguo/github_repo/dtr_robot_digital_shaddow/pc_side/catkin_ws/build/cm_listener/catkin_generated/installspace/cm_listener.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cm_listener" TYPE PROGRAM FILES "/home/zhiguo/github_repo/dtr_robot_digital_shaddow/pc_side/catkin_ws/build/cm_listener/catkin_generated/installspace/cm_listener_websocket.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cm_listener" TYPE PROGRAM FILES "/home/zhiguo/github_repo/dtr_robot_digital_shaddow/pc_side/catkin_ws/build/cm_listener/catkin_generated/installspace/cm_listen_position.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cm_listener" TYPE PROGRAM FILES "/home/zhiguo/github_repo/dtr_robot_digital_shaddow/pc_side/catkin_ws/build/cm_listener/catkin_generated/installspace/cm_listen_trajectory.py")
endif()

