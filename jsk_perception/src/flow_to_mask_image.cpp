// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "jsk_perception/flow_to_mask_image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <algorithm>
#include <math.h> 

#define PI 3.141592

namespace jsk_perception
{
  void FlowToMaskImage::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("diff_threshold", diff_threshold_, 1.0);
    pnh_->param("draw_rect_size", draw_rect_size_, 3);
    pub_mask_image_ = advertise<sensor_msgs::Image>(*pnh_, "mask", 1);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&FlowToMaskImage::configCallback, this, _1, _2);
    srv_->setCallback (f);
  }

  void FlowToMaskImage::configCallback(Config &new_config, uint32_t level)
  {
  }


  void FlowToMaskImage::subscribe()
  {
    //    sub_image_ = pnh_->subscribe("input", 1, &FlowToMaskImage::toMask, this);
    sub_camera_info_ = pnh_->subscribe("input/camera_info", 1, &FlowToMaskImage::cameraInfoCallback, this);
  }

  void FlowToMaskImage::unsubscribe()
  {
    sub_image_.shutdown();
    sub_camera_info_.shutdown();
  }

  void FlowToMaskImage::cameraInfoCallback(const sensor_msgs::CameraInfo& camera_info_msg)
  {
    target_image_height_ = camera_info_msg.height;
    target_image_width_ = camera_info_msg.width;
  }

  // void FlowToMaskImage::toMask(const opencv_apps::FlowArrayStamped& flow_msg)
  // {
  //   if (!(target_image_height_ && target_image_width_))
  //     return;

  //   cv::Mat mask(target_image_height_, target_image_width_,  CV_8UC1);
  //   for(int i = 0; i < flow_msg.flow.size(); i++){
  //     int point_y = std::max(std::min (((int)flow_msg.flow[i].point.y + (int)flow_msg.flow[i].velocity.y), target_image_height_ - 1), 0);
  //     int point_x = std::max(std::min (((int)flow_msg.flow[i].point.x + (int)flow_msg.flow[i].velocity.x), target_image_width_ - 1), 0);

  //     if(diff_threshold_ < sqrt(flow_msg.flow[i].velocity.y * flow_msg.flow[i].velocity.y + flow_msg.flow[i].velocity.x * flow_msg.flow[i].velocity.x)){
  // 	cv::rectangle(mask,
  // 		      cv::Point(point_x, point_y) - cv::Point(draw_rect_size_, draw_rect_size_),
  // 		      cv::Point(point_x, point_y) + cv::Point(draw_rect_size_, draw_rect_size_),
  // 		      255,
  // 		      CV_FILLED
  // 		      );
  //     }
  //   }
  //   pub_mask_image_.publish(cv_bridge::CvImage(flow_msg.header,
  // 					       sensor_msgs::image_encodings::MONO8,
  // 					       mask).toImageMsg());
  // }
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::FlowToMaskImage, nodelet::Nodelet);
