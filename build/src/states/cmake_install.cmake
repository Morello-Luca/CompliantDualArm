# Install script for directory: /home/vscode/workspace/sandbox/controllers/FSM-DualManipulation/src/states

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
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

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Idle.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Idle.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Idle.so"
         RPATH "/home/vscode/workspace/install/lib/mc_controller/fsm/states:/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Idle.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states" TYPE SHARED_LIBRARY FILES "/home/vscode/workspace/sandbox/controllers/FSM-DualManipulation/build/src/states/Idle.so")
  if(EXISTS "$ENV{DESTDIR}/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Idle.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Idle.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Idle.so"
         OLD_RPATH "/home/vscode/workspace/sandbox/controllers/FSM-DualManipulation/build/src:/home/vscode/workspace/install/lib:/opt/ros/humble/lib:::::::"
         NEW_RPATH "/home/vscode/workspace/install/lib/mc_controller/fsm/states:/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Idle.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Independent.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Independent.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Independent.so"
         RPATH "/home/vscode/workspace/install/lib/mc_controller/fsm/states:/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Independent.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states" TYPE SHARED_LIBRARY FILES "/home/vscode/workspace/sandbox/controllers/FSM-DualManipulation/build/src/states/Independent.so")
  if(EXISTS "$ENV{DESTDIR}/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Independent.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Independent.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Independent.so"
         OLD_RPATH "/home/vscode/workspace/sandbox/controllers/FSM-DualManipulation/build/src:/home/vscode/workspace/install/lib:/opt/ros/humble/lib:::::::"
         NEW_RPATH "/home/vscode/workspace/install/lib/mc_controller/fsm/states:/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Independent.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Collaborative.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Collaborative.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Collaborative.so"
         RPATH "/home/vscode/workspace/install/lib/mc_controller/fsm/states:/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Collaborative.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states" TYPE SHARED_LIBRARY FILES "/home/vscode/workspace/sandbox/controllers/FSM-DualManipulation/build/src/states/Collaborative.so")
  if(EXISTS "$ENV{DESTDIR}/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Collaborative.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Collaborative.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Collaborative.so"
         OLD_RPATH "/home/vscode/workspace/sandbox/controllers/FSM-DualManipulation/build/src:/home/vscode/workspace/install/lib:/opt/ros/humble/lib:::::::"
         NEW_RPATH "/home/vscode/workspace/install/lib/mc_controller/fsm/states:/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/Collaborative.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states/data")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/vscode/workspace/install/lib/mc_controller/FSMDualManipulation/states" TYPE DIRECTORY FILES "/home/vscode/workspace/sandbox/controllers/FSM-DualManipulation/src/states/data" FILES_MATCHING REGEX ".*(.json$|.yml$|.yaml$)")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/vscode/workspace/sandbox/controllers/FSM-DualManipulation/build/src/states/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
