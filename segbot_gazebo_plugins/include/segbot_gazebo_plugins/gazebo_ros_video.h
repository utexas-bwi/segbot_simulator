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

#include <ros/ros.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/Image.h>

#include <gazebo/rendering/rendering.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

namespace gazebo {

  class VideoVisual : public rendering::Visual {
    public: 
      /// \brief Constructor
      /// \param[in] _name Name of the video visual.
      /// \param[in] _parent Parent of the video visual.
      VideoVisual(const std::string &_name, rendering::VisualPtr _parent, 
          int h, int w) :
        rendering::Visual(_name, _parent), height(h), width(w) {

          this->texture = Ogre::TextureManager::getSingleton().createManual(
              _name + "__VideoTexture__",
              Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
              Ogre::TEX_TYPE_2D,
              this->width, this->height,
              0,
              Ogre::PF_BYTE_BGR,
              Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

          Ogre::MaterialPtr material =
            Ogre::MaterialManager::getSingleton().create(
                _name + "__VideoMaterial__", "General");
          material->getTechnique(0)->getPass(0)->createTextureUnitState(
              _name + "__VideoTexture__");
          material->setReceiveShadows(false);

          double factor = 1.0;

          Ogre::ManualObject mo(_name + "__VideoObject__");
          mo.begin(_name + "__VideoMaterial__",
              Ogre::RenderOperation::OT_TRIANGLE_LIST);

          mo.position(-factor / 2, factor / 2, 0.51);
          mo.textureCoord(0, 0);

          mo.position(factor / 2, factor / 2, 0.51);
          mo.textureCoord(1, 0);

          mo.position(factor / 2, -factor / 2, 0.51);
          mo.textureCoord(1, 1);

          mo.position(-factor / 2, -factor / 2, 0.51);
          mo.textureCoord(0, 1);

          mo.triangle(0, 3, 2);
          mo.triangle(2, 1, 0);
          mo.end();

          mo.convertToMesh(_name + "__VideoMesh__");

          Ogre::MovableObject *obj = (Ogre::MovableObject*)
            this->sceneNode->getCreator()->createEntity(
                _name + "__VideoEntity__",
                _name + "__VideoMesh__");
          obj->setCastShadows(false);
          this->AttachObject(obj);

        }

      /// \brief Destructor
      virtual ~VideoVisual() {}

      /// \brief PreRender event callback.
      void render(const cv::Mat& image) {

        // Get the pixel buffer
        Ogre::HardwarePixelBufferSharedPtr pixelBuffer = 
          this->texture->getBuffer();
        
        // Lock the pixel buffer and get a pixel box
        pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
        const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
        uint8_t* pDest = static_cast<uint8_t*>(pixelBox.data);

        bool unusedAlpha = Ogre::PixelUtil::getNumElemBytes(
            this->texture->getFormat()) > 3 ? true : false;

        // If OGRE actually created a texture with no alpha channel, then we
        // can use memcpy
        if (!unusedAlpha) {
          memcpy(pDest, image.data, this->height*this->width*3);
        } else {
          int index;
          for (int j = 0; j < this->height; ++j) {
            for (int i = 0; i < this->width; ++i) {
              index = j*(this->width*3) + (i*3);
              *pDest++ = image.data[index + 0];  // B
              *pDest++ = image.data[index + 1];  // G
              *pDest++ = image.data[index + 2];  // R
              *pDest++ = 255;  // Alpha
            }
          }
        }

        // Unlock the pixel buffer
        pixelBuffer->unlock();
      }

    private:

      /// \brief Texture to draw the video onto.
      Ogre::TexturePtr texture;

      /// \brief Width and height of the video.
      int height,width;
  }; 

  class GazeboRosVideo : public VisualPlugin {
    public: 
    
      /// \brief Constructor
      GazeboRosVideo();

      /// \brief Destructor
      virtual ~GazeboRosVideo();

      /// \brief Load the controller
      void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf );

      void processImage(const sensor_msgs::ImageConstPtr &msg);

    protected:

      /// \brief Update the controller
      virtual void UpdateChild();

      // Pointer to the model
      rendering::VisualPtr model;
      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;

      boost::shared_ptr<VideoVisual> video_visual_;

      cv_bridge::CvImagePtr image_;
      boost::mutex m_image_;
      bool new_image_available_;

      // ROS Stuff
      boost::shared_ptr<ros::NodeHandle> rosnode_;
      boost::shared_ptr<image_transport::ImageTransport> it_;
      image_transport::Subscriber camera_subscriber_;
      int height;
      int width;
      std::string modelNamespace;
      std::string topicName;

      void QueueThread();
      boost::thread callback_queue_thread_;

  };

}

#endif

