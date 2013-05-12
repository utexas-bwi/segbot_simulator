/*
 * A differential drive plugin for the segbot. Based on the diffdrive plugin 
 * developed for the erratic robot (see copyright notice below). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins
 *
 * Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/

#ifndef DIFFDRIVE_PLUGIN_HH
#define DIFFDRIVE_PLUGIN_HH

#include <map>

#include <common/common.hh>
#include <physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo
{
class Joint;
class Entity;

class SegbotDrivePlugin : public ModelPlugin
{
  public: SegbotDrivePlugin();
  public: ~SegbotDrivePlugin();
  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  protected: virtual void UpdateChild();
  protected: virtual void FiniChild();

private:
  void writePositionData(double step_time);
  void publishOdometry(double step_time);
  void getWheelVelocities();

  physics::WorldPtr world;
  physics::ModelPtr parent;
  event::ConnectionPtr updateConnection;

  std::string leftJointName;
  std::string rightJointName;

  double wheelSeparation;
  double wheelDiameter;
  double torque;
  double wheelSpeed[2];
  math::Pose last_odom_pose_;
  double last_angular_vel_;

  physics::JointPtr joints[2];
  physics::PhysicsEnginePtr physicsEngine;

  // ROS STUFF
  ros::NodeHandle* rosnode_;
  ros::Publisher pub_, pub2_;
  ros::Subscriber sub_, sub2_;
  tf::TransformBroadcaster *transform_broadcaster_;
  nav_msgs::Odometry odom_;
  std::string tf_prefix_;

  boost::mutex lock;

  std::string robotNamespace;
  std::string topicName;
  bool useSimpleModel;

  // Custom Callback Queue
  ros::CallbackQueue queue_;
  boost::thread callback_queue_thread_;
  void QueueThread();

  // DiffDrive stuff
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

  double x_;
  double rot_;
  bool alive_;

  // Update Rate
  double updateRate;
  double update_period_;
  common::Time last_update_time_;

  // Simple map stuff
  std::string simpleMapTopic;
  double simpleModelRadius;
  double simpleRobotPadding;
  double circumscribed_robot_distance_;
  nav_msgs::OccupancyGrid simple_map_;
  bool map_available_;
  void getSimpleMap(const nav_msgs::OccupancyGrid::ConstPtr& map);
  double simple_map_block_time_;

};

}

#endif

