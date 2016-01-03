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

#include "jsk_perception/substraction.h"

namespace jsk_perception
{
  void Substraction::onInit()
  {
    DiagnosticNodelet::onInit();

    //Mode
    //0 : Not considering Time
    //1 : Considerint with Time
    //2 : BackgroundSubtractorMOG2
    pnh_->param("mode", mode_, 0);
    pnh_->param("median", median_, 1);
    pnh_->param("passthrough", passthrough_, true);
    pnh_->param("min_thres", min_thres_, 7);
    pnh_->param("mask_image", mask_image_, false);
    pnh_->param("mask_pub", mask_pub_, false);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &Substraction::configCallback, this, _1, _2);
    srv_->setCallback (f);
    
    sr_pub_ = advertise<sensor_msgs::Image>(*pnh_, "sr_output", 1);
    rs_pub_ = advertise<sensor_msgs::Image>(*pnh_, "rs_output", 1);
    both_pub_ = advertise<sensor_msgs::Image>(*pnh_, "both_output", 1);

    if(mask_pub_){
      sr_mask_pub_ = advertise<sensor_msgs::Image>(*pnh_, "sr_mask_output", 1);
      rs_mask_pub_ = advertise<sensor_msgs::Image>(*pnh_, "rs_maskoutput", 1);
      both_mask_pub_ = advertise<sensor_msgs::Image>(*pnh_, "both_mask_output", 1);
    }

  }

  void Substraction::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
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
  
  void Substraction::subscribe()
  {
    it_.reset(new image_transport::ImageTransport(*pnh_));
    sub_ = it_->subscribe("real_image", 1, &Substraction::substract, this);
    bg_sub_ = it_->subscribe("sim_image", 1, &Substraction::backgroundSubstract, this);
  }

  void Substraction::unsubscribe()
  {
    sub_.shutdown();
    bg_sub_.shutdown();
  }

  void Substraction::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (vital_checker_->isAlive()) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   "Substraction running");
    }
    else {
      jsk_topic_tools::addDiagnosticErrorSummary(
        "Substraction", vital_checker_, stat);
    }

  }
  
  void Substraction::backgroundSubstract(const sensor_msgs::Image::ConstPtr& image_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    bg_ptr
      = cv_bridge::toCvCopy(image_msg, "16UC1");

    if(mode_ != 0){
#if CV_MAJOR_VERSION >= 3
      bg_ = cv::createBackgroundSubtractorMOG2();
#else
      bg_ = cv::BackgroundSubtractorMOG2();
#endif
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
    cv::Mat fg;
#if CV_MAJOR_VERSION >= 3
    bg_->apply(bg_ptr->image, fg);
    bg_->apply(bg_ptr->image, fg);
    bg_->apply(bg_ptr->image, fg);
#else
    bg_(bg_ptr->image, fg);
    bg_(bg_ptr->image, fg);
    bg_(bg_ptr->image, fg);
#endif
    }
      }

  void Substraction::substract(
      const sensor_msgs::Image::ConstPtr& image_msg)
  {
    if(!bg_ptr)
      return;
    vital_checker_->poke();
    boost::mutex::scoped_lock lock(mutex_);
    cv_bridge::CvImagePtr cv_ptr
      = cv_bridge::toCvCopy(image_msg, "16UC1");
    cv::Mat image = cv_ptr->image;
    cv::Mat sr_fg, rs_fg, both_fg;

    if(passthrough_)
      {
      for(int y = 0; y < image.rows; y++){
          for(int x = 0; x < image.cols; x++){
          if (image.at<unsigned short>(y, x) > min_thres_){
            image.at<unsigned short>(y,x) = 0;
            }
          if (bg_ptr->image.at<unsigned short>(y, x) > min_thres_){
            bg_ptr->image.at<unsigned short>(y,x) = 0;
            }
          }
        }
      }

    if(median_)
      {
      cv::medianBlur(bg_ptr->image, bg_ptr->image, 5);
      cv::medianBlur(image, image, 5);
      }

    // std::vector<cv::Mat> dst_img;
    // cv::buildPyramid(src_img, dst_img, 2);

    if (mode_ == 0){
      cv::subtract(image, bg_ptr->image, sr_fg);
      //cv::fastNlMeansDenoising(sr_fg, sr_fg);
      cv::Mat const structure_elem = cv::getStructuringElement(
                                                               cv::MORPH_RECT, cv::Size(5, 5));
      cv::morphologyEx(sr_fg, sr_fg,
                       cv::MORPH_OPEN, structure_elem, cv::Point(-1, -1), 2);
      cv::subtract(bg_ptr->image, image, rs_fg);
      cv::morphologyEx(rs_fg, rs_fg,
                       cv::MORPH_OPEN, structure_elem, cv::Point(-1, -1), 2);
      //cv::fastNlMeansDenoising(rs_fg, rs_fg);
      both_fg = sr_fg + rs_fg;
    }else{
#if CV_MAJOR_VERSION >= 3
      bg_->apply(image, sr_fg);
#else
      bg_(image, sr_fg);
#endif
    }

    if(mask_image_)
      {
        cv::Mat sr_mask, rs_mask, both_mask;
        cv::Mat tmp_sr, tmp_rs, tmp_both;
      sr_fg.convertTo(tmp_sr, CV_8UC1);
      rs_fg.convertTo(tmp_rs, CV_8UC1);
      both_fg.convertTo(tmp_both, CV_8UC1);
      cv::threshold(tmp_sr, sr_mask, 0, 255, CV_THRESH_BINARY);
      cv::threshold(tmp_rs, rs_mask, 0, 255, CV_THRESH_BINARY);
      cv::threshold(tmp_both, both_mask, 0, 255, CV_THRESH_BINARY);

      cv_bridge::toCvCopy(image_msg, "16UC1")->image.copyTo(sr_fg, sr_mask);
      bg_ptr->image.copyTo(rs_fg, rs_mask);
      both_fg = sr_fg + rs_fg;

      if(mask_pub_){
        sensor_msgs::Image::Ptr sr_mask_image
          = cv_bridge::CvImage(image_msg->header,
                         sensor_msgs::image_encodings::MONO8,
                         sr_mask).toImageMsg();
        sr_mask_pub_.publish(sr_mask_image);

        sensor_msgs::Image::Ptr rs_mask_image
          = cv_bridge::CvImage(image_msg->header,
                         sensor_msgs::image_encodings::MONO8,
                         rs_mask).toImageMsg();
        rs_mask_pub_.publish(rs_mask_image);

        sensor_msgs::Image::Ptr both_mask_image
          = cv_bridge::CvImage(image_msg->header,
                         sensor_msgs::image_encodings::MONO8,
                         both_mask).toImageMsg();
        both_mask_pub_.publish(both_mask_image);
      }
      }

    sensor_msgs::Image::Ptr sr_diff_image
      = cv_bridge::CvImage(image_msg->header,
                     sensor_msgs::image_encodings::TYPE_16UC1,
                     sr_fg).toImageMsg();
    sr_pub_.publish(sr_diff_image);

    sensor_msgs::Image::Ptr rs_diff_image
      = cv_bridge::CvImage(image_msg->header,
                     sensor_msgs::image_encodings::TYPE_16UC1,
                     rs_fg).toImageMsg();
    rs_pub_.publish(rs_diff_image);

    sensor_msgs::Image::Ptr both_diff_image
      = cv_bridge::CvImage(image_msg->header,
                     sensor_msgs::image_encodings::TYPE_16UC1,
                     both_fg).toImageMsg();
    both_pub_.publish(both_diff_image);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::Substraction, nodelet::Nodelet);
