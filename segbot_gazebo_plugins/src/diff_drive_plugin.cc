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

    This plugin is based on the original DiffDrivePlugin in
    erratic_gazebo_plugins. Improved and modified by Piyush Khandelwal 
    (piyushk@gmail.com). The original copyright notice can be found above.
*/


#include <algorithm>
#include <assert.h>

#include <segbot_gazebo_plugins/diff_drive_plugin.h>

#include <math/gzmath.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

namespace gazebo {

  enum {
    RIGHT,
    LEFT,
  };

  DiffDrivePlugin::DiffDrivePlugin() {}

  // Destructor
  DiffDrivePlugin::~DiffDrivePlugin() {
    delete rosnode_;
    delete transform_broadcaster_;
  }

  // Load the controller
  void DiffDrivePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

    this->parent = _parent;
    this->world = _parent->GetWorld();

    this->robotNamespace = "";
    if (!_sdf->HasElement("robotNamespace")) {
      ROS_INFO("DDPlugin missing <robotNamespace>, defaults to \"%s\"", 
          this->robotNamespace.c_str());
    } else {
      this->robotNamespace = 
        _sdf->GetElement("robotNamespace")->GetValueString() + "/";
    }

    this->leftJointName = "left_joint";
    if (!_sdf->HasElement("leftJoint")) {
      ROS_WARN("DDPlugin (%s) missing <leftJoint>, defaults to \"%s\"",
          this->robotNamespace.c_str(), this->leftJointName.c_str());
    } else {
      this->leftJointName = _sdf->GetElement("leftJoint")->GetValueString();
    }

    this->rightJointName = "right_joint";
    if (!_sdf->HasElement("rightJoint")) {
      ROS_WARN("DDPlugin (%s) missing <rightJoint>, defaults to \"%s\"",
          this->robotNamespace.c_str(), this->rightJointName.c_str());
    } else {
      this->rightJointName = _sdf->GetElement("rightJoint")->GetValueString();
    }

    this->wheelSeparation = 0.34;
    if (!_sdf->HasElement("wheelSeparation")) {
      ROS_WARN("DDPlugin (%s) missing <wheelSeparation>, defaults to %f",
          this->robotNamespace.c_str(), this->wheelSeparation);
    } else {
      this->wheelSeparation = 
        _sdf->GetElement("wheelSeparation")->GetValueDouble();
    }

    this->wheelDiameter = 0.15;
    if (!_sdf->HasElement("wheelDiameter")) {
      ROS_WARN("DDPlugin (%s) missing <wheelDiameter>, defaults to %f",
          this->robotNamespace.c_str(), this->wheelDiameter);
    } else {
      this->wheelDiameter = _sdf->GetElement("wheelDiameter")->GetValueDouble();
    }

    this->torque = 5.0;
    if (!_sdf->HasElement("torque")) {
      ROS_WARN("DDPlugin (%s) missing <torque>, defaults to %f",
          this->robotNamespace.c_str(), this->torque);
    } else {
      this->torque = _sdf->GetElement("torque")->GetValueDouble();
    }

    this->topicName = "cmd_vel";
    if (!_sdf->HasElement("topicName")) {
      ROS_WARN("DDPlugin (%s) missing <topicName>, defaults to \"%s\"",
          this->robotNamespace.c_str(), this->topicName.c_str());
    } else {
      this->topicName = _sdf->GetElement("topicName")->GetValueString();
    }

    this->updateRate = 100.0;
    if (!_sdf->HasElement("updateRate")) {
      ROS_WARN("DDPlugin (%s) missing <updateRate>, defaults to %f",
          this->robotNamespace.c_str(), this->updateRate);
    } else {
      this->updateRate = _sdf->GetElement("updateRate")->GetValueDouble();
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

    if (!joints[LEFT]) { 
      char error[200];
      snprintf(error, 200, 
          "DDPlugin (%s) couldn't get left hinge joint named \"%s\"", 
          this->robotNamespace.c_str(), this->leftJointName.c_str());
      gzthrow(error);
    }
    if (!joints[RIGHT]) { 
      char error[200];
      snprintf(error, 200, 
          "DDPlugin (%s) couldn't get right hinge joint named \"%s\"", 
          this->robotNamespace.c_str(), this->rightJointName.c_str());
      gzthrow(error);
    }

    // Initialize the ROS node and subscribe to cmd_vel
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "diff_drive_plugin", 
        ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    rosnode_ = new ros::NodeHandle(this->robotNamespace);

    ROS_INFO("Starting DDPlugin (%s)!", this->robotNamespace.c_str());

    tf_prefix_ = tf::getPrefixParam(*rosnode_);
    transform_broadcaster_ = new tf::TransformBroadcaster();

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(topicName, 1,
          boost::bind(&DiffDrivePlugin::cmdVelCallback, this, _1),
          ros::VoidPtr(), &queue_);

    sub_ = rosnode_->subscribe(so);

    pub_ = rosnode_->advertise<nav_msgs::Odometry>("odom", 1);

    // start custom queue for diff drive
    this->callback_queue_thread_ = 
      boost::thread(boost::bind(&DiffDrivePlugin::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    this->updateConnection = 
      event::Events::ConnectWorldUpdateStart(
          boost::bind(&DiffDrivePlugin::UpdateChild, this));

  }

  // Update the controller
  void DiffDrivePlugin::UpdateChild() {
    common::Time current_time = this->world->GetSimTime();
    double seconds_since_last_update = 
      (current_time - last_update_time_).Double();
    if (seconds_since_last_update > update_period_) {

      publishOdometry(seconds_since_last_update);

      // Update robot in case new velocities have been requested
      getWheelVelocities();
      joints[LEFT]->SetVelocity(0, wheelSpeed[LEFT] / wheelDiameter);
      joints[RIGHT]->SetVelocity(0, wheelSpeed[RIGHT] / wheelDiameter);

      last_update_time_+= common::Time(update_period_);

    }
  }

  // Finalize the controller
  void DiffDrivePlugin::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void DiffDrivePlugin::getWheelVelocities() {
    boost::mutex::scoped_lock scoped_lock(lock);

    double vr = x_;
    double va = rot_;
    last_angular_vel_ = va;

    wheelSpeed[LEFT] = vr + va * wheelSeparation / 2.0;
    wheelSpeed[RIGHT] = vr - va * wheelSeparation / 2.0;
  }

  void DiffDrivePlugin::cmdVelCallback(
      const geometry_msgs::Twist::ConstPtr& cmd_msg) {

    boost::mutex::scoped_lock scoped_lock(lock);
    x_ = cmd_msg->linear.x;
    rot_ = cmd_msg->angular.z;
  }

  void DiffDrivePlugin::QueueThread() {
    static const double timeout = 0.01;

    while (alive_ && rosnode_->ok()) {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  void DiffDrivePlugin::publishOdometry(double step_time) {
    ros::Time current_time = ros::Time::now();
    std::string odom_frame = 
      tf::resolve(tf_prefix_, "odom");
    std::string base_footprint_frame = 
      tf::resolve(tf_prefix_, "base_footprint");

    // getting data for base_footprint to odom transform
    math::Pose pose = this->parent->GetWorldPose();

    tf::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
    tf::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);

    tf::Transform base_footprint_to_odom(qt, vt);
    transform_broadcaster_->sendTransform(
        tf::StampedTransform(base_footprint_to_odom, current_time, 
            odom_frame, base_footprint_frame));

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
    linear = this->parent->GetWorldLinearVel();
    odom_.twist.twist.angular.z = this->parent->GetWorldAngularVel().z;

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = pose.rot.GetYaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.x + sinf(yaw) * linear.y;
    odom_.twist.twist.linear.y = cosf(yaw) * linear.y - sinf(yaw) * linear.x;

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    pub_.publish(odom_);
  }

  GZ_REGISTER_MODEL_PLUGIN(DiffDrivePlugin)
}

