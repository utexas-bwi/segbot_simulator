# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(segbot_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros image_transport cv_bridge message_generation)
find_package(Boost REQUIRED COMPONENTS thread)

# Start custom Gazebo CMake stuff
find_package(GAZEBO REQUIRED)
find_package(SDF REQUIRED)

# http://stackoverflow.com/questions/7172670/best-shortest-way-to-join-a-list-in-cmake
function(JOIN VALUES GLUE OUTPUT)
  string (REGEX REPLACE "([^\\]|^);" "\\1${GLUE}" _TMP_STR "${VALUES}")
  string (REGEX REPLACE "[\\](.)" "\\1" _TMP_STR "${_TMP_STR}") #fixes escaping
  set (${OUTPUT} "${_TMP_STR}" PARENT_SCOPE)
endfunction()
JOIN("${GAZEBO_LDFLAGS}" " " LD_FLAGS)

#message(STATUS "GAZEBO_LDFLAGS: ${GAZEBO_LDFLAGS}")
message(STATUS "LD_FLAGS: ${LD_FLAGS}")

find_package(PkgConfig)
pkg_check_modules(OGRE OGRE)

#execute_process(COMMAND
#  pkg-config --variable=plugindir OGRE
#  OUTPUT_VARIABLE OGRE_PLUGIN_PATH

#OUTPUT_STRIP_TRAILING_WHITESPACE
#)

# End custom gazebo cmake stuff

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS} 
  ${OGRE_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
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
add_dependencies(object_controller_plugin segbot_gazebo_plugins_gencpp)
set_target_properties(object_controller_plugin PROPERTIES LINK_FLAGS "${LD_FLAGS}")
target_link_libraries(object_controller_plugin 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES} 
  ${SDFormat_LIBRARIES}
)

add_library(gazebo_ros_video src/gazebo_ros_video.cc)
set_target_properties(gazebo_ros_video PROPERTIES LINK_FLAGS "${LD_FLAGS}")
target_link_libraries(gazebo_ros_video 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES} 
  ${SDFormat_LIBRARIES}
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp
    gazebo_ros 
    image_transport 
    cv_bridge
    message_runtime
  INCLUDE_DIRS 
    include
  LIBRARIES
    object_controller_plugin
    gazebo_ros_video
)

# Install targets

install(TARGETS 
  object_controller_plugin  
  gazebo_ros_video
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY include/segbot_gazebo_plugins/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
