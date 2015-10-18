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

    scale_ = 1.0;
    gamma_ = 1.0;
    use_bilateral_ = true;
    use_lut_ = true;
    times_ = 1;

    pnh_->param("bilateral_d", bilateral_d_, 11);
    pnh_->param("bilateral_sigma_color", bilateral_sigma_color_, 40.0);
    pnh_->param("bilateral_sigma_space", bilateral_sigma_space_, 200.0);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &BackgroundSubstraction::configCallback, this, _1, _2);
    srv_->setCallback (f);
    
    image_pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    image_pub2_ = advertise<sensor_msgs::Image>(*pnh_, "after_lut", 1);
    image_pub3_ = advertise<sensor_msgs::Image>(*pnh_, "after_bila", 1);
    image_pub4_ = advertise<sensor_msgs::Image>(*pnh_, "after_diff", 1);

    for (int i = 0; i < 256; i++){
      LUT[i] = (int)(pow((double)i / 255.0, 1.0 / gamma_) * 255.0);
    }
  }

  void BackgroundSubstraction::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);

    times_ = config.times;
    use_bilateral_ = config.use_bilateral;
    use_lut_ = config.use_lut;
    scale_ = config.scale;

    bilateral_d_ = config.bilateral_d;
    bilateral_sigma_color_ = config.bilateral_sigma_color;
    bilateral_sigma_space_ = config.bilateral_sigma_space;
    thresh_ = config.thresh;

    gamma_ = config.gamma;
    for (int i = 0; i < 256; i++){
      LUT[i] = (int)(pow((double)i / 255.0, 1.0 / gamma_) * 255.0);
    }

    bg_ = cv::BackgroundSubtractorMOG2();
    history_ = config.history;
    background_ratio_ = config.background_ratio;
    nmixtures_ = config.nmixtures;
    detect_shadows_ = config.detect_shadows;
    shadow_threshold_ = config.shadow_threshold;
    shadow_value_ = config.shadow_value;
    // var_threshold_ = config.var_threshold;
    // bg_.set("var_threshold", var_threshold_);
    bg_.set("fCT", complexity_reduction_threshold_);
    bg_.set("fTau", shadow_threshold_);
    // bg_.set("nShadowDetection", (uchar)shadow_value_);
    bg_.set("backgroundRatio", background_ratio_);
    bg_.set("history", history_);
    bg_.set("nmixtures", nmixtures_);

    if (detect_shadows_) {
      bg_.set("detectShadows", 1);
    }
    else {
      bg_.set("detectShadows", 0);
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
    cv_bridge::CvImagePtr cv_ptr;
    if(image_msg->encoding != "16UC1")
      cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    else{
      cv_ptr = cv_bridge::toCvCopy(image_msg, "16UC1");
    }
      
    cv::Mat before_image = cv_ptr->image;
    if (image_msg->encoding == "16UC1")
      before_image.convertTo(before_image, CV_8UC1);
    cv::Mat fg;
    std::vector <std::vector<cv::Point > > contours;


    cv::Mat dst_imgUp, dst_imgDown;
    if (scale_ != 1.0 && scale_ != 0.0){
      cv::pyrDown(before_image, dst_imgDown, cv::Size(before_image.cols * scale_, before_image.rows * scale_));
      before_image = dst_imgDown;
    }

    cv::Mat image;
    if(use_lut_){
      cv::LUT(before_image, cv::Mat(cv::Size(256, 1), CV_8U, LUT), image);
    }
    else
      image = before_image;
    sensor_msgs::Image::Ptr lut_ros
      = cv_bridge::CvImage(image_msg->header,
			   sensor_msgs::image_encodings::BGR8,
			   image).toImageMsg();
    image_pub2_.publish(lut_ros);
    

    if(use_bilateral_){
      cv::Mat image2;
      cv::bilateralFilter(image, image2, bilateral_d_, bilateral_sigma_color_, bilateral_sigma_space_);
      image = image2;
    }

    if (scale_ != 1.0 && scale_ != 0.0){
      cv::pyrUp(image, dst_imgUp, cv::Size(image.cols / scale_, image.rows / scale_));
      image = dst_imgUp;
    }

    sensor_msgs::Image::Ptr image2_ros
      = cv_bridge::CvImage(image_msg->header,
                           sensor_msgs::image_encodings::BGR8,
                           image).toImageMsg();
    image_pub3_.publish(image2_ros);
    // if (bilateral_d_ %2 == 1)
    //   cv::medianBlur(image, image2, bilateral_d_);
#if CV_MAJOR_VERSION >= 3
    bg_->apply(image, fg);
#else
    bg_(image, fg);
#endif
    sensor_msgs::Image::Ptr fg_ros
      = cv_bridge::CvImage(image_msg->header,
                           sensor_msgs::image_encodings::MONO8,
                           fg).toImageMsg();
    image_pub4_.publish(fg_ros);

    cv::Mat last;
    cv::threshold(fg, last, thresh_, 0, cv::THRESH_TOZERO);

    cv::Mat erode, dilate;
    for(int i=0; i<times_; i++)
      cv::erode(last, erode, cv::Mat(), cv::Point(-1,-1), 1);
    for(int i=0; i<times_; i++)
      cv::dilate(erode, dilate, cv::Mat(), cv::Point(-1,-1), 1);
    last = dilate;

    sensor_msgs::Image::Ptr diff_image
      = cv_bridge::CvImage(image_msg->header,
                           sensor_msgs::image_encodings::MONO8,
                           last).toImageMsg();
    image_pub_.publish(diff_image);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::BackgroundSubstraction, nodelet::Nodelet);
