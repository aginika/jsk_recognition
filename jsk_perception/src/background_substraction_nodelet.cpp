// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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

#include "jsk_perception/background_substraction.h"

namespace jsk_perception
{
  void BackgroundSubstraction::onInit()
  {
    DiagnosticNodelet::onInit();

    pnh_->param("bilateral_d", bilateral_d_, 11);
    pnh_->param("bilateral_sigma_color", bilateral_sigma_color_, 40.0);
    pnh_->param("bilateral_sigma_space", bilateral_sigma_space_, 200.0);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &BackgroundSubstraction::configCallback, this, _1, _2);
    srv_->setCallback (f);
    
    image_pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    
  }

  void BackgroundSubstraction::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);

    bilateral_d_ = config.bilateral_d;
    bilateral_sigma_color_ = config.bilateral_sigma_color;
    bilateral_sigma_space_ = config.bilateral_sigma_space;
#if CV_MAJOR_VERSION >= 3
    bg_ = cv::createBackgroundSubtractorMOG2();
#else
    bg_ = cv::BackgroundSubtractorMOG2();
#endif
    nmixtures_ = config.nmixtures;
    detect_shadows_ = config.detect_shadows;
#if CV_MAJOR_VERSION >= 3
    bg_->setNMixtures(nmixtures_);
#else
    bg_.set("nmixtures", nmixtures_);
#endif
    if (detect_shadows_) {
#if CV_MAJOR_VERSION >= 3
      bg_->setDetectShadows(1);
#else
      bg_.set("detectShadows", 1);
#endif
    }
    else {
#if CV_MAJOR_VERSION >= 3
      bg_->setDetectShadows(1);
#else
      bg_.set("detectShadows", 0);
#endif
    }
  }
  
  void BackgroundSubstraction::subscribe()
  {
    it_.reset(new image_transport::ImageTransport(*pnh_));
    sub_ = it_->subscribe("image", 1, &BackgroundSubstraction::substract, this);
  }

  void BackgroundSubstraction::unsubscribe()
  {
    sub_.shutdown();
  }

  void BackgroundSubstraction::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (vital_checker_->isAlive()) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   "BackgroundSubstraction running");
    }
    else {
      jsk_topic_tools::addDiagnosticErrorSummary(
        "BackgroundSubstraction", vital_checker_, stat);
    }

  }
  
  void BackgroundSubstraction::substract(
    const sensor_msgs::Image::ConstPtr& image_msg)
  {
    vital_checker_->poke();
    boost::mutex::scoped_lock lock(mutex_);
    cv_bridge::CvImagePtr cv_ptr
      = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;
    cv::Mat fg;
    std::vector <std::vector<cv::Point > > contours;

    cv::Mat image2;
    cv::bilateralFilter(image, image2, bilateral_d_, bilateral_sigma_color_, bilateral_sigma_space_);
    // if (bilateral_d_ %2 == 1)
    //   cv::medianBlur(image, image2, bilateral_d_);
#if CV_MAJOR_VERSION >= 3
    bg_->apply(image2, fg);
#else
    bg_(image2, fg);
#endif
    sensor_msgs::Image::Ptr diff_image
      = cv_bridge::CvImage(image_msg->header,
                           sensor_msgs::image_encodings::MONO8,
                           fg).toImageMsg();
    image_pub_.publish(diff_image);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::BackgroundSubstraction, nodelet::Nodelet);
