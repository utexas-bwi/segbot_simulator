/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: 3D position interface.
 * Author: Sachin Chitta and John Hsu
 * Date: 10 June 2008
 * SVN: $Id$
 */
#ifndef GAZEBO_ROS_VIDEO_H
#define GAZEBO_ROS_VIDEO_H

#include <boost/thread/mutex.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/transport/TransportTypes.hh>

namespace gazebo {

  class VideoVisual : public rendering::Visual {
    public: 
      VideoVisual(
          const std::string &name, rendering::VisualPtr parent, 
          int height, int width);
      virtual ~VideoVisual();
      void render(const cv::Mat& image);
    private:
      Ogre::TexturePtr texture_;
      int height_;
      int width_;
  }; 

  class GazeboRosVideo : public VisualPlugin {
    public: 
    
      GazeboRosVideo();
      virtual ~GazeboRosVideo();

      void Load(rendering::VisualPtr parent, sdf::ElementPtr sdf);
      void processImage(const sensor_msgs::ImageConstPtr &msg);

    protected:

      virtual void UpdateChild();

      // Pointer to the model
      rendering::VisualPtr model_;
      // Pointer to the update event connection
      event::ConnectionPtr update_connection_;

      boost::shared_ptr<VideoVisual> video_visual_;

      cv_bridge::CvImagePtr image_;
      boost::mutex m_image_;
      bool new_image_available_;

      // ROS Stuff
      boost::shared_ptr<ros::NodeHandle> rosnode_;
      ros::Subscriber camera_subscriber_;
      std::string robot_namespace_;
      std::string topic_name_;

      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

  };

}

#endif

