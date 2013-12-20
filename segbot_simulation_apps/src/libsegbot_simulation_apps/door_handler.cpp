#include <boost/algorithm/string/join.hpp>
#include <boost/regex.hpp>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <stdexcept>
#include <tf/transform_datatypes.h>

#include <bwi_tools/point.h>
#include <segbot_simulation_apps/door_handler.h>

namespace segbot_simulation_apps {

  DoorHandler::DoorHandler() {

    ros::NodeHandle nh, private_nh("~");

    std::vector<std::string> unavailable_parameters;
    std::string door_file;
    if (!(private_nh.getParam("door_file", door_file))) {
      unavailable_parameters.push_back("door_file");
    }
    if (!(private_nh.getParam("obstacle_urdf", obstacle_urdf_))) {
      unavailable_parameters.push_back("obstacle_urdf");
    }
    if (!(private_nh.getParam("door_urdf", door_urdf_))) {
      unavailable_parameters.push_back("door_urdf");
    }

    if (unavailable_parameters.size() != 0) {
      std::string message = "Following neccessary params not available: " +
        boost::algorithm::join(unavailable_parameters, ", ");
      ROS_INFO_STREAM(message);
      throw std::runtime_error(message);
    }

    readDoorFile(door_file, doors_);

    get_gazebo_model_client_ =
      nh.serviceClient<gazebo_msgs::GetModelState>(
          "/gazebo/get_model_state");
    bool gazebo_available = 
      get_gazebo_model_client_.waitForExistence(ros::Duration(30));

    if (!gazebo_available) {
      ROS_FATAL_STREAM("ClingoGazeboHandler: Gazebo is NOT AVAILABLE");
      throw 
        std::runtime_error("ClingoGazeboHandler: Gazebo is NOT AVAILABLE");
    }

    set_gazebo_model_client_ =
      nh.serviceClient<gazebo_msgs::SetModelState>(
          "/gazebo/set_model_state");
    set_gazebo_model_client_.waitForExistence();

    spawn_model_client_ =
      nh.serviceClient<gazebo_msgs::SpawnModel>(
          "/gazebo/spawn_urdf_model");
    set_gazebo_model_client_.waitForExistence();

    // Spawn all necessary doors
    door_open_status_.resize(doors_.size());
    for (unsigned i = 0; i < doors_.size(); ++i) {
      spawnObject(true, i);
      door_open_status_[i] = false;
    }

    // Don't use obstacles for now
    // Spawn 30 obstacle objects
    num_obstacles_ = 0;
    // for (unsigned i = 0; i < 30; ++i) {
    //   spawnObject(false, i);
    // }

  }

  geometry_msgs::Pose DoorHandler::getDefaultLocation(bool is_door, int index) {
    geometry_msgs::Pose retval;
    retval.position.y = 500.0f + index * 2;
    retval.position.z = 0.0f;
    retval.orientation.x = 0.0f;
    retval.orientation.y = 0.0f;
    retval.orientation.z = 0.0f;
    retval.orientation.w = 1.0f;
    if (is_door) {
      retval.position.x = 500.0f;
    } else {
      retval.position.x = 600.0f;
    }
    return retval;
  }

  float DoorHandler::getDoorWidth(int index) {
    return (0.75f/0.9f) * doors_[index].width;
  }

  geometry_msgs::Pose DoorHandler::getDoorLocation(int index) {
    geometry_msgs::Pose retval;

    bwi::Point2f door_center = 0.5 *
      (doors_[index].approach_points[0] +
       doors_[index].approach_points[1]);
    retval.position.x = door_center.x;
    retval.position.y = door_center.y;
    retval.position.z = 0;

    bwi::Point2f diff = 
      (doors_[index].approach_points[0] -
       doors_[index].approach_points[1]);
    float door_yaw = atan2f(diff.y, diff.x);
    retval.orientation = tf::createQuaternionMsgFromYaw(door_yaw);

    return retval;
  }

  bool DoorHandler::openDoor(const std::string& door) {
    size_t idx = resolveDoor(door);
    if (idx == bwi_planning_common::NO_DOOR_IDX) {
      return false;
    }
    return openDoor(idx); 
  }

  bool DoorHandler::openDoor(int index) {
    if (index >= doors_.size()) {
      return false;
    }
    if (door_open_status_[index]) 
      return true;
    std::string prefix = "auto_door_";
    std::string model_name = prefix +
      boost::lexical_cast<std::string>(index);
    geometry_msgs::Pose pose = getDefaultLocation(true, index);
    bool success = teleportEntity(model_name, pose);
    door_open_status_[index] = true;
    return success;
  }

  void DoorHandler::openAllDoors() {
    for (unsigned i = 0; i < doors_.size(); ++i) {
      openDoor(i);
    }
  }

  bool DoorHandler::closeDoor(const std::string& door) {
    size_t idx = resolveDoor(door);
    if (idx == bwi_planning_common::NO_DOOR_IDX) {
      return false;
    }
    return closeDoor(idx); 
  }

  bool DoorHandler::closeDoor(int index) {
    if (index >= doors_.size()) {
      return false;
    }
    if (!door_open_status_[index]) 
      return true;
    ROS_INFO_STREAM("Closing door " << index);
    std::string prefix = "auto_door_";
    std::string model_name = prefix +
      boost::lexical_cast<std::string>(index);
    geometry_msgs::Pose pose = getDoorLocation(index);
    bool success = teleportEntity(model_name, pose);
    door_open_status_[index] = false;
    return success;
  }

  void DoorHandler::closeAllDoors() {
    ROS_INFO_STREAM("Closing all doors");
    for (unsigned i = 0; i < doors_.size(); ++i) {
      closeDoor(i);
    }
  }


  bool DoorHandler::isDoorOpen(const std::string& door) {
    size_t idx = resolveDoor(door);
    if (idx == bwi_planning_common::NO_DOOR_IDX) {
      return false;
    }
    return isDoorOpen(idx); 
  }

  bool DoorHandler::isDoorOpen(int index) {
    if (index >= doors_.size()) {
      return false;
    }
    return door_open_status_[index];
  }

  void DoorHandler::closeAllDoorsFarAwayFromPoint(
      const geometry_msgs::Pose& point, float distance) {
    for (unsigned i = 0; i < doors_.size(); ++i) {
      if (!door_open_status_[i])
        continue;
      bool is_door_near = 
        checkClosePoses(point, getDoorLocation(i), distance, false);
      if (!is_door_near) 
        closeDoor(i);
    }
  }

  bool DoorHandler::checkClosePoses(const geometry_msgs::Pose& p1,
      const geometry_msgs::Pose& p2, float threshold,
      bool check_yaw) {
    float dist_diff = 
      sqrtf(pow((p1.position.x - p2.position.x), 2) +
          pow((p1.position.y - p2.position.y), 2));
    if (dist_diff > threshold) {
      return false;
    }
    double yaw1 = tf::getYaw(p1.orientation);
    double yaw2 = tf::getYaw(p2.orientation);
    if (check_yaw && fabs(yaw1 - yaw2) > 0.1) {
      return false;
    }
    return true;
  }

  bool DoorHandler::teleportEntity(const std::string& entity,
      const geometry_msgs::Pose& pose) {

    int count = 0;
    int attempts = 5;
    bool location_verified = false;
    while (count < attempts and !location_verified) {
      gazebo_msgs::GetModelState get_srv;
      get_srv.request.model_name = entity;
      get_gazebo_model_client_.call(get_srv);
      location_verified = checkClosePoses(get_srv.response.pose, pose);
      if (!location_verified) {
        gazebo_msgs::SetModelState set_srv;
        set_srv.request.model_state.model_name = entity;
        set_srv.request.model_state.pose = pose;
        set_gazebo_model_client_.call(set_srv);
        if (!set_srv.response.success) {
          ROS_WARN_STREAM("SetModelState service call failed for " << entity
              << " to " << pose);
        }
      }
      ++count;
    }
    if (!location_verified) {
      ROS_ERROR_STREAM("Unable to teleport " << entity << " to " << pose
          << " despite " << attempts << " attempts.");
      return false;
    }
    return true;
  }

  void DoorHandler::spawnObject(bool is_door, int index) {

    gazebo_msgs::SpawnModel spawn;
    std::string prefix;
    if (is_door) {
      prefix = "auto_door_";
      spawn.request.model_xml = boost::regex_replace(door_urdf_, 
          boost::regex("@WIDTH@"),
          boost::lexical_cast<std::string>(getDoorWidth(index)));
      spawn.request.initial_pose = getDoorLocation(index);
    } else {
      prefix = "auto_obs_";
      index = num_obstacles_;
      spawn.request.model_xml = obstacle_urdf_;
      spawn.request.initial_pose = getDefaultLocation(false, index);
    }

    spawn.request.model_name = prefix +
      boost::lexical_cast<std::string>(index);

    if (spawn_model_client_.call(spawn)) {
      if (spawn.response.success) {
        ++num_obstacles_;
        return;
      }
      ROS_WARN_STREAM("Received error message while spawning object: " <<
          spawn.response.status_message);
    }

    ROS_ERROR_STREAM("Unable to spawn: " << spawn.request.model_name);
  }      

  size_t DoorHandler::resolveDoor(const std::string& door) {
    
    for (size_t i = 0; i < doors_.size(); ++i) {
      if (doors_[i].name == door) {
        return i;
      }
    }

    return bwi_planning_common::NO_DOOR_IDX;
  }

}
