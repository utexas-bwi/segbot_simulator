# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(segbot_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros gazebo_plugins image_transport cv_bridge message_generation)
find_package(Boost REQUIRED COMPONENTS thread)

# Start custom Gazebo CMake stuff
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO_SYSTEM REQUIRED gazebo)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

set(cxx_flags)
foreach (item ${GAZEBO_SYSTEM_CFLAGS})
  set(cxx_flags "${cxx_flags} ${item}")
endforeach ()
message(STATUS "segbot_gazebo_plugins: using cxx_flags: ${cxx_flags}")

set(ld_flags)
foreach (item ${GAZEBO_SYSTEM_LDFLAGS})
  set(ld_flags "${ld_flags} ${item}")
endforeach ()
message(STATUS "segbot_gazebo_plugins: using ld_flags: ${ld_flags}")

pkg_check_modules(OGRE OGRE)

execute_process(COMMAND
  pkg-config --variable=plugindir OGRE
  OUTPUT_VARIABLE OGRE_PLUGIN_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# End custom gazebo cmake stuff

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_SYSTEM_INCLUDE_DIRS} 
  ${OGRE_INCLUDE_DIRS}
)

add_service_files(
  FILES
  UpdatePluginState.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

add_library(object_controller_plugin src/object_controller_plugin.cc)
set_target_properties(object_controller_plugin PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(object_controller_plugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(object_controller_plugin 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GAZEBO_SYSTEM_LIBRARIES} 
)

add_library(gazebo_ros_video src/gazebo_ros_video.cc)
set_target_properties(gazebo_ros_video PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_video PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_video 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GAZEBO_SYSTEM_LIBRARIES} 
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp
    gazebo_ros 
    gazebo_plugins 
    image_transport 
    cv_bridge
    message_runtime
  INCLUDE_DIRS 
    include
  LIBRARIES
    object_controller_plugin
    gazebo_ros_video
)

install(TARGETS 
  object_controller_plugin  
  gazebo_ros_video
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY include 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
