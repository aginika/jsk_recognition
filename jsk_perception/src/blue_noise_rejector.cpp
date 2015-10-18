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


#include "jsk_perception/blue_noise_rejector.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace jsk_perception
{
  void BlueNoiseRejector::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    pub_b_ = advertise<sensor_msgs::Image>(*pnh_, "output/after_blue", 1);
  }

  void BlueNoiseRejector::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &BlueNoiseRejector::decompose, this);
  }

  void BlueNoiseRejector::unsubscribe()
  {
    sub_.shutdown();
  }

  void BlueNoiseRejector::decompose(
    const sensor_msgs::Image::ConstPtr& image_msg)
  {
    if ((image_msg->width == 0) && (image_msg->height == 0)) {
        JSK_ROS_WARN("invalid image input");
        return;
    }
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
      image_msg, image_msg->encoding);
    cv::Mat image = cv_ptr->image;
    if (image_msg->encoding == sensor_msgs::image_encodings::RGB8) {
      cv::cvtColor(image, image, CV_RGB2BGR);
    }

    cv::Mat color;

    if(false){
      std::vector<cv::Mat> bgr_planes;
      cv::split(image, bgr_planes);
      cv::Mat red = bgr_planes[2];
      cv::Mat blue = bgr_planes[0];
      cv::Mat green = bgr_planes[1];

      /*
	Noise Reduction in Blue
      */
      cv::Mat denoised_blue;
      double h = 3.0;
      int template_window_size = 7;
      int search_window_size = 21;
      cv::fastNlMeansDenoising(blue, denoised_blue, h, template_window_size, search_window_size);
      blue = denoised_blue;
      
      /*
	Merge RGB Channels
      */
      
      std::vector<cv::Mat> array_to_merge;
      array_to_merge.push_back(blue);
      array_to_merge.push_back(green);
      array_to_merge.push_back(red);
      pub_b_.publish(cv_bridge::CvImage(image_msg->header,
					sensor_msgs::image_encodings::MONO8,
					blue).toImageMsg());
      cv::merge(array_to_merge, color);
    }
    else{
      double h = 3.0;
      double h_color = 8.0;
      int template_window_size = 7;
      int search_window_size = 21;
      cv::fastNlMeansDenoisingColored(image, color, h, h_color, template_window_size, search_window_size);
    }
    pub_.publish(cv_bridge::CvImage(image_msg->header,
				    sensor_msgs::image_encodings::BGR8,
				    color).toImageMsg());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::BlueNoiseRejector, nodelet::Nodelet);
