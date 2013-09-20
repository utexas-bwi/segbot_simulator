# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(segbot_gazebo)
find_package(catkin REQUIRED)
catkin_package()

# Install all supporting directories
foreach(dir config launch maps Media worlds)
  install(DIRECTORY ${dir}/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# unit tests are enabled selectively
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  #roslaunch_add_file_check(launch/multiple_segbots.launch)   # arg exception
  #roslaunch_add_file_check(launch/segbot_mobile_base.launch) # arg exception
  #roslaunch_add_file_check(launch/segbot_navigation.launch)  # arg exception
  #roslaunch_add_file_check(launch/segbot_test_world.launch)  # arg exception
  roslaunch_add_file_check(launch/segbot_test_world_map_server.launch)
endif (CATKIN_ENABLE_TESTING)
