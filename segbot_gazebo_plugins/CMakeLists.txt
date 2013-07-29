# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(segbot_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS 
  gazebo_ros 
  roscpp 
  )

catkin_package(
  CATKIN_DEPENDS 
    gazebo_ros 
    roscpp
  INCLUDE_DIRS 
    include
  LIBRARIES
  # gazebo_ros_model_controller
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)
find_package(SDF REQUIRED)

# http://stackoverflow.com/questions/7172670/best-shortest-way-to-join-a-list-in-cmake
function(JOIN VALUES GLUE OUTPUT)
  string (REGEX REPLACE "([^\\]|^);" "\\1${GLUE}" _TMP_STR "${VALUES}")
  string (REGEX REPLACE "[\\](.)" "\\1" _TMP_STR "${_TMP_STR}") #fixes escaping
  set (${OUTPUT} "${_TMP_STR}" PARENT_SCOPE)
endfunction()
JOIN("${GAZEBO_LDFLAGS}" " " LD_FLAGS)

#message(STATUS "GAZEBO_LDFLAGS: ${GAZEBO_LDFLAGS}")
message(STATUS "segbot_gazebo_plugins: using LD_FLAGS = \"${LD_FLAGS}\"")

find_package(PkgConfig)
pkg_check_modules(OGRE OGRE)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS} 
  ${OGRE_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)

## add_library(gazebo_ros_model_controller src/gazebo_ros_model_controller.cpp)
## add_dependencies(gazebo_ros_model_controller segbot_gazebo_plugins_gencpp)
## set_target_properties(gazebo_ros_model_controller PROPERTIES LINK_FLAGS "${LD_FLAGS}")
## target_link_libraries(gazebo_ros_model_controller 
##   ${Boost_LIBRARIES}
##   ${catkin_LIBRARIES}
##   ${GAZEBO_LIBRARIES} 
##   ${SDFormat_LIBRARIES}
## )

# Install targets
install(TARGETS 
  # gazeboi_ros_model_controller  
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY include/segbot_gazebo_plugins/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
