#include <ros/ros.h>

#include <segbot_simulation_apps/door_handler.h>
#include <segbot_simulation_apps/DoorHandlerInterface.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "gazebo_door_handler");
  segbot_simulation_apps::DoorHandler gh;
  ros::NodeHandle nh;
  int count = 0;
  ros::Rate r(1.0);
  while (ros::ok()) {
    ROS_INFO_STREAM("tick " << count);
    if (count == 10) gh.closeAllDoors();
    ros::spinOnce();
    ++count;
    r.sleep();
  }
  return 0;
}
