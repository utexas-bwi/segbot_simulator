/*
    Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <algorithm>
#include <assert.h>

#include <bwi_gazebo_plugins/segbotdrive_plugin.h>

#include <common/common.h>
#include <math/gzmath.h>
#include <physics/physics.h>
#include <sdf/sdf.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

namespace gazebo
{

enum
{
  RIGHT,
  LEFT,
};

// Constructor
SegbotDrivePlugin::SegbotDrivePlugin()
{
}

// Destructor
SegbotDrivePlugin::~SegbotDrivePlugin()
{
  delete rosnode_;
  delete transform_broadcaster_;
}

// Load the controller
void SegbotDrivePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->parent = _parent;
  this->world = _parent->GetWorld();

  gzdbg << "plugin parent sensor name: " << parent->GetName() << "\n";

  if (!this->parent) { gzthrow("Differential_Position2d controller requires a Model as its parent"); }

  this->robotNamespace = "";
  if (_sdf->HasElement("robotNamespace")) {
    this->robotNamespace = _sdf->GetElement("robotNamespace")->GetValueString() + "/";
  }

  if (!_sdf->HasElement("leftJoint")) {
    ROS_WARN("Differential Drive plugin missing <leftJoint>, defaults to left_joint");
    this->leftJointName = "left_joint";
  } else {
    this->leftJointName = _sdf->GetElement("leftJoint")->GetValueString();
  }

  if (!_sdf->HasElement("rightJoint")) {
    ROS_WARN("Differential Drive plugin missing <rightJoint>, defaults to right_joint");
    this->rightJointName = "right_joint";
  } else {
    this->rightJointName = _sdf->GetElement("rightJoint")->GetValueString();
  }

  if (!_sdf->HasElement("wheelSeparation")) {
    ROS_WARN("Differential Drive plugin missing <wheelSeparation>, defaults to 0.34");
    this->wheelSeparation = 0.34;
  } else {
    this->wheelSeparation = _sdf->GetElement("wheelSeparation")->GetValueDouble();
  }

  if (!_sdf->HasElement("wheelDiameter")) {
    ROS_WARN("Differential Drive plugin missing <wheelDiameter>, defaults to 0.15");
    this->wheelDiameter = 0.15;
  } else {
    this->wheelDiameter = _sdf->GetElement("wheelDiameter")->GetValueDouble();
  }

  if (!_sdf->HasElement("torque")) {
    ROS_WARN("Differential Drive plugin missing <torque>, defaults to 5.0");
    this->torque = 5.0;
  } else {
    this->torque = _sdf->GetElement("torque")->GetValueDouble();
  }

  if (!_sdf->HasElement("topicName")) {
    ROS_WARN("Differential Drive plugin missing <topicName>, defaults to cmd_vel");
    this->topicName = "cmd_vel";
  } else {
    this->topicName = _sdf->GetElement("topicName")->GetValueString();
  }

  if (!_sdf->HasElement("updateRate")) {
    ROS_WARN("Differential Drive plugin missing <updateRate>, defaults to 100.0");
    this->updateRate = 100.0;
  } else {
    this->updateRate = _sdf->GetElement("updateRate")->GetValueDouble();
  }

  // In the simple model:
  //  - the collision model of the robot is a cylinder.
  //  - the wheels are 1mm above the ground, 
  //  - The robot is manually moved in world space
  //  - speedup factor of about 10x over the full model
  if (!_sdf->HasElement("useSimpleModel")) {
    ROS_WARN("Differential Drive plugin missing <useSimpleModel>, defaults to false");
    this->useSimpleModel = false;
  } else {
    std::string value = _sdf->GetElement("useSimpleModel")->GetValueString();
    this->useSimpleModel = value == "true" || value == "1";
  }

  if (this->useSimpleModel) { // Get some additional parameters
    if (!_sdf->HasElement("simpleMapTopic")) {
      ROS_WARN("Differential Drive plugin missing <simpleMapTopic>, defaults to static_map");
      this->simpleMapTopic = "map";
    } else {
      this->simpleMapTopic = _sdf->GetElement("simpleMapTopic")->GetValueString();
    }
    if (!_sdf->HasElement("simpleModelRadius")) {
      ROS_WARN("Differential Drive plugin missing <simpleModelRadius>, defaults to 0.5");
      this->simpleModelRadius = 0.5;
    } else {
      this->simpleModelRadius = _sdf->GetElement("simpleModelRadius")->GetValueDouble();
    }
    if (!_sdf->HasElement("simpleRobotPadding")) {
      ROS_WARN("Differential Drive plugin missing <simpleRobotPadding>, defaults to 0.05");
      this->simpleRobotPadding = 0.05;
    } else {
      this->simpleRobotPadding = _sdf->GetElement("simpleRobotPadding")->GetValueDouble();
    }
    this->circumscribed_robot_distance_ = this->simpleRobotPadding + this->simpleModelRadius;
  }

  // Initialize update rate stuff
  if (this->updateRate > 0.0) {
    this->update_period_ = 1.0 / this->updateRate;
  } else {
    this->update_period_ = 0.0;
  }
  last_update_time_ = this->world->GetSimTime();

  // Initialize velocity stuff
  wheelSpeed[RIGHT] = 0;
  wheelSpeed[LEFT] = 0;
  last_odom_pose_ = this->parent->GetWorldPose();

  x_ = 0;
  rot_ = 0;
  alive_ = true;

  joints[LEFT] = this->parent->GetJoint(leftJointName);
  joints[RIGHT] = this->parent->GetJoint(rightJointName);

  joints[LEFT]->SetMaxForce(0, torque);
  joints[RIGHT]->SetMaxForce(0, torque);

  if (!joints[LEFT])  { gzthrow("The controller couldn't get left hinge joint"); }
  if (!joints[RIGHT]) { gzthrow("The controller couldn't get right hinge joint"); }

  // Initialize the ROS node and subscribe to cmd_vel
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "segbotdrive_plugin", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle(this->robotNamespace);

  ROS_INFO("starting segbotdrive plugin in ns: %s", this->robotNamespace.c_str());


  tf_prefix_ = tf::getPrefixParam(*rosnode_);
  transform_broadcaster_ = new tf::TransformBroadcaster();

  // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(topicName, 1,
                                                          boost::bind(&SegbotDrivePlugin::cmdVelCallback, this, _1),
                                                          ros::VoidPtr(), &queue_);

  sub_ = rosnode_->subscribe(so);

  if (useSimpleModel) {
    ROS_INFO("Subscribing to %s", simpleMapTopic.c_str());
    ros::SubscribeOptions so2 =
        ros::SubscribeOptions::create<nav_msgs::OccupancyGrid>(simpleMapTopic, 1,
                                                            boost::bind(&SegbotDrivePlugin::getSimpleMap, this, _1),
                                                            ros::VoidPtr(), &queue_);
    sub2_ = rosnode_->subscribe(so2);
  }
  map_available_ = false;

  pub_ = rosnode_->advertise<nav_msgs::Odometry>("odom", 1);
  pub2_ = rosnode_->advertise<nav_msgs::OccupancyGrid>("expanded_map", 1, true);

  // start custom queue for diff drive
  this->callback_queue_thread_ = boost::thread(boost::bind(&SegbotDrivePlugin::QueueThread, this));

  // listen to the update event (broadcast every simulation iteration)
  this->updateConnection = event::Events::ConnectWorldUpdateStart(boost::bind(&SegbotDrivePlugin::UpdateChild, this));

}

// Update the controller
void SegbotDrivePlugin::UpdateChild()
{
  common::Time current_time = this->world->GetSimTime();
  double seconds_since_last_update = (current_time - last_update_time_).Double();
  if (seconds_since_last_update > update_period_) {

    writePositionData(seconds_since_last_update);
    publishOdometry(seconds_since_last_update);

    // Update robot in case new velocities have been requested
    getWheelVelocities();
    joints[LEFT]->SetVelocity(0, wheelSpeed[LEFT] / wheelDiameter);
    joints[RIGHT]->SetVelocity(0, wheelSpeed[RIGHT] / wheelDiameter);

    last_update_time_+= common::Time(update_period_);

  }
}

// Finalize the controller
void SegbotDrivePlugin::FiniChild()
{
  alive_ = false;
  queue_.clear();
  queue_.disable();
  rosnode_->shutdown();
  callback_queue_thread_.join();
}

void SegbotDrivePlugin::getWheelVelocities()
{
  boost::mutex::scoped_lock scoped_lock(lock);

  double vr = x_;
  double va = rot_;
  last_angular_vel_ = va;

  wheelSpeed[LEFT] = vr + va * wheelSeparation / 2.0;
  wheelSpeed[RIGHT] = vr - va * wheelSeparation / 2.0;
}

void SegbotDrivePlugin::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
  boost::mutex::scoped_lock scoped_lock(lock);
  x_ = cmd_msg->linear.x;
  rot_ = cmd_msg->angular.z;
}

void SegbotDrivePlugin::QueueThread()
{
  static const double timeout = 0.01;

  while (alive_ && rosnode_->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void SegbotDrivePlugin::getSimpleMap(const nav_msgs::OccupancyGrid::ConstPtr& map) {

  simple_map_.header = map->header;

  simple_map_.info.map_load_time = map->info.map_load_time;
  simple_map_.info.resolution = map->info.resolution;
  simple_map_.info.width = map->info.width;
  simple_map_.info.height = map->info.height;

  // Get the map origin in global space
  // useful for multi_level_map
  tf::TransformListener listener;
  geometry_msgs::PoseStamped pose_in, pose_out;
  pose_in.header = map->header;
  pose_in.pose = map->info.origin;

  if (listener.waitForTransform(map->header.frame_id, "/map", ros::Time(0), ros::Duration(5.0))) {
    ROS_INFO("Transformation for simple map acquired");
    listener.transformPose("/map", ros::Time(0), pose_in, pose_in.header.frame_id, pose_out);
    simple_map_.info.origin = pose_out.pose;
  } else {
    ROS_ERROR("Unable to get transformation from /map to %s.", map->header.frame_id.c_str());
    simple_map_.info.origin = pose_in.pose;
  }

  // expand the map out based on the circumscribed robot distance
  int expand_pixels = ceil(circumscribed_robot_distance_ / map->info.resolution);
  simple_map_.data.resize(map->info.height * map->info.width);
  for (int i = 0; i < (int)map->info.height; ++i) {
    for (int j = 0; j < (int)map->info.width; ++j) {
      int low_i = (i - expand_pixels < 0) ? 0 : i - expand_pixels;
      int high_i = (i + expand_pixels >= (int)map->info.height) ? 
          map->info.height - 1 : i + expand_pixels;
      int max = 0;
      for (int k = low_i; k <= high_i; ++k) {
        int diff_j = floor(sqrtf(expand_pixels * expand_pixels - (i - k) * (i - k)));
        int low_j = (j - diff_j < 0) ? 0 : j - diff_j;
        int high_j = (j + diff_j >= (int)map->info.width) ? 
            map->info.width - 1 : j + diff_j;
        for (int l = low_j; l <= high_j; ++l) {
          if (map->data[k * map->info.width + l] > max) {
            max = map->data[k * map->info.width + l];
          }
        }
      }
      simple_map_.data[i * map->info.width + j] = max;
    }
  }

  ROS_INFO("Simple Map Acquired");
  pub2_.publish(simple_map_);
  map_available_ = true;
}

void SegbotDrivePlugin::publishOdometry(double step_time)
{
  ros::Time current_time = ros::Time::now();
  std::string odom_frame = tf::resolve(tf_prefix_, "odom");
  std::string base_footprint_frame = tf::resolve(tf_prefix_, "base_footprint");

  // getting data for base_footprint to odom transform
  math::Pose pose = this->parent->GetState().GetPose();

  tf::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
  tf::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);

  tf::Transform base_footprint_to_odom(qt, vt);
  transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom,
                                                             current_time,
                                                             odom_frame,
                                                             base_footprint_frame));

  // publish odom topic
  odom_.pose.pose.position.x = pose.pos.x;
  odom_.pose.pose.position.y = pose.pos.y;

  odom_.pose.pose.orientation.x = pose.rot.x;
  odom_.pose.pose.orientation.y = pose.rot.y;
  odom_.pose.pose.orientation.z = pose.rot.z;
  odom_.pose.pose.orientation.w = pose.rot.w;
  odom_.pose.covariance[0] = 0.00001;
  odom_.pose.covariance[7] = 0.00001;
  odom_.pose.covariance[14] = 1000000000000.0;
  odom_.pose.covariance[21] = 1000000000000.0;
  odom_.pose.covariance[28] = 1000000000000.0;
  odom_.pose.covariance[35] = 0.001;

  // get velocity in /odom frame
  math::Vector3 linear;
  if (!this->useSimpleModel) {
    linear = this->parent->GetWorldLinearVel();
    odom_.twist.twist.angular.z = this->parent->GetWorldAngularVel().z;
  } else {
    // Getting values from the worlds model in gazebo instead of supplied
    // velocites as a simple means of error correction
    linear.x = (pose.pos.x - last_odom_pose_.pos.x) / step_time;
    linear.y = (pose.pos.y - last_odom_pose_.pos.y) / step_time;
    if (last_angular_vel_ > M_PI / step_time) { // we cannot calculate the angular velocity correctly
      odom_.twist.twist.angular.z = last_angular_vel_;
    } else {
      float last_yaw = last_odom_pose_.rot.GetYaw();
      float current_yaw = pose.rot.GetYaw();
      while (current_yaw < last_yaw - M_PI) current_yaw += 2*M_PI;
      while (current_yaw > last_yaw + M_PI) current_yaw -= 2*M_PI;
      float angular_diff = current_yaw - last_yaw;
      odom_.twist.twist.angular.z = angular_diff / step_time;
    }
    last_odom_pose_ = pose;
  }

  // convert velocity to child_frame_id (aka base_footprint)
  float yaw = pose.rot.GetYaw();
  odom_.twist.twist.linear.x = cosf(yaw) * linear.x + sinf(yaw) * linear.y;
  odom_.twist.twist.linear.y = cosf(yaw) * linear.y - sinf(yaw) * linear.x;

  odom_.header.stamp = current_time;
  odom_.header.frame_id = odom_frame;
  odom_.child_frame_id = base_footprint_frame;

  pub_.publish(odom_);
}

// Update the data in the interface
void SegbotDrivePlugin::writePositionData(double step_time)
{
  // move the simple model manually
  if (this->useSimpleModel && map_available_) {

    double wd, ws;
    double d1, d2;
    double dr, da;

    wd = wheelDiameter;
    ws = wheelSeparation;

    // Distance travelled by front wheels
    d1 = step_time * wd * joints[LEFT]->GetVelocity(0);
    d2 = step_time * wd * joints[RIGHT]->GetVelocity(0);

    dr = (d1 + d2) / 2;
    da = (d1 - d2) / ws;

    math::Pose orig_pose = this->parent->GetWorldPose();
    math::Pose new_pose = orig_pose;
    new_pose.pos.x = orig_pose.pos.x + dr * cos(orig_pose.rot.GetYaw());
    new_pose.pos.y = orig_pose.pos.y + dr * sin(orig_pose.rot.GetYaw());
    new_pose.rot.SetFromEuler(math::Vector3(0,0,orig_pose.rot.GetYaw() + da));

    // Check if the new pose can be allowed
    int x_pxl = (new_pose.pos.x - simple_map_.info.origin.position.x) / simple_map_.info.resolution;
    int y_pxl = (new_pose.pos.y - simple_map_.info.origin.position.y) / simple_map_.info.resolution;

    if (x_pxl < 0 || x_pxl >= (int)simple_map_.info.width ||
        y_pxl < 0 || y_pxl >= (int)simple_map_.info.height ||
        (simple_map_.data[(simple_map_.info.height - y_pxl - 1)*simple_map_.info.width + x_pxl] < 50)) {
      this->parent->SetWorldPose(new_pose);
      simple_map_block_time_ = 0;
    } else {
      simple_map_block_time_ += step_time;
      if (simple_map_block_time_ > 2.0) {
        ROS_WARN("Simple Segbot plugin is preventing running into a wall. If you think this is an error, then check the map");
        simple_map_block_time_ = 0;
      }
    }
  }

}

GZ_REGISTER_MODEL_PLUGIN(SegbotDrivePlugin)
}

