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

#include "jsk_perception/depth_image_completion.h"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

namespace jsk_perception
{
  void DepthImageCompletion::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
  }

  void DepthImageCompletion::subscribe()
  {
    sub_mask_ = pnh_->subscribe("input", 1, &DepthImageCompletion::convert, this);
  }

  void DepthImageCompletion::unsubscribe()
  {
    sub_mask_.shutdown();
  }

  void DepthImageCompletion::convert(
    const sensor_msgs::Image::ConstPtr& image_msg)
  {
    vital_checker_->poke();

    std::vector<cv::Point> indices;
    // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
    //   image_msg, sensor_msgs::image_encodings::MONO8);
    cv_bridge::CvImagePtr cv_ptr
      = cv_bridge::toCvCopy(image_msg, "16UC1");
    cv::Mat mask = cv_ptr->image;
    cv::Mat x_img, y_img, ave_img;
    mask.copyTo(x_img);
    mask.copyTo(y_img);
    mask.copyTo(ave_img);


    for(int y = 0; y < mask.rows; y++){
      bool zero_flag = false;
      int zero_start = -1;
      for(int x = 0; x < mask.cols; x++){
	if (mask.at<unsigned short>(y, x) < 100 && (x + 1) != mask.cols){
	  if( !zero_flag){
	    zero_flag = true;
	    zero_start = x;
	    //ROS_ERROR("zero check start");
	  }
	  continue;
	}else if(zero_flag){
	  //bool reverse = false;
	  //interpolate depth pixels...
	  unsigned short start_pixel = 0;
	  if (zero_start < 10 && x > mask.rows - 10){
	    zero_flag = false;
	    zero_start = -1;
	    continue;
	  }
	  if(zero_start - 1 >= 0)
	    start_pixel = x_img.at<unsigned short>(y, zero_start - 1);
	  else if(y != 0)
	    start_pixel = x_img.at<unsigned short>(y - 1, zero_start);
	  unsigned short end_pixel = 0;
	  if(x + 1 <  mask.cols && x > 0){
	    end_pixel = x_img.at<unsigned short>(y, x);
	    if(std::abs(start_pixel - end_pixel) > 500)
	      if(end_pixel < start_pixel)
		end_pixel = start_pixel;
	       else
	     	 start_pixel = end_pixel;
	  }else if(y != 0)
	    end_pixel = x_img.at<unsigned short>(y - 1, x);


	  for(int interpolate_x = 0; interpolate_x <= x - zero_start; interpolate_x++){
	    // if( y > 0 &&  300 > std::abs(x_img.at<unsigned short>(y - 1, zero_start) - x_img.at<unsigned short>(y - 1, x))){
	    //   if(interpolate_x + zero_start > 0){
	    // 	x_img.at<unsigned short>(y,interpolate_x + zero_start)
	    // 	  = x_img.at<unsigned short>(y, zero_start + interpolate_x - 1) + (x_img.at<unsigned short>(y - 1, zero_start + interpolate_x)
	    // 									 - x_img.at<unsigned short>(y - 1, zero_start + interpolate_x -1));
	    //   }
	    // }else{
	      x_img.at<unsigned short>(y,interpolate_x + zero_start) =
		start_pixel* 1.0 * ( x - zero_start - interpolate_x)/ (x - zero_start)
		+
		end_pixel* 1.0 * interpolate_x/ (x - zero_start);
	    // }
	  }
	  zero_flag = false;
	  zero_start = -1;
	}else{
	  
	}
      }
    }

    //======================================================================================
    //======================================================================================
    //======================================================================================
    //======================================================================================
    //======================================================================================
    //======================================================================================

    for(int y = 0; y < mask.cols; y++){
      bool zero_flag = false;
      int zero_start = -1;
      for(int x = 0; x < mask.rows; x++){
    	if (mask.at<unsigned short>(x, y) < 100 && (x + 1) != mask.rows){
    	  if( !zero_flag){
    	    zero_flag = true;
    	    zero_start = x;
    	  }
    	  continue;
    	}else if(zero_flag){
    	  if (zero_start < 10 && x > mask.rows - 10){
    	    zero_flag = false;
    	    zero_start = -1;
    	    continue;
    	  }

    	  //interpolate depth pixels...
    	  unsigned short start_pixel = 0;
    	  if(zero_start - 1 >= 0)
    	    start_pixel = y_img.at<unsigned short>(zero_start - 1, y);
    	  else if(y != 0)
    	    start_pixel = y_img.at<unsigned short>(zero_start, y - 1);
	    

    	  unsigned short end_pixel = 0;
    	  if(x + 1 <  mask.cols){
    	    end_pixel = y_img.at<unsigned short>(x, y);
    	    if(std::abs(start_pixel - end_pixel) > 500){
    	       if(end_pixel < start_pixel)
    		 end_pixel = start_pixel;
    	       else
    		 start_pixel = end_pixel;
    	     }
    	  }else if(y != 0)
    	    end_pixel = y_img.at<unsigned short>(x, y - 1);


    	  for(int interpolate_x = 0; interpolate_x <= x - zero_start; interpolate_x++){
    	    y_img.at<unsigned short>(interpolate_x + zero_start, y) =
    		start_pixel* 1.0 * ( x - zero_start - interpolate_x)/ (x - zero_start)
    		+
    		end_pixel* 1.0 * interpolate_x/ (x - zero_start);
    	  }
    	  zero_flag = false;
    	  zero_start = -1;
    	}else{
	    
    	}
      }
    }
    

    for(int y = 0; y < mask.rows; y++){
      for(int x = 0; x < mask.cols; x++){
    	unsigned short x_val = x_img.at<unsigned short>(y,x);
    	unsigned short y_val = y_img.at<unsigned short>(y,x);
    	if (x_val != 0 && y_val != 0)
    	  ave_img.at<unsigned short>(y,x) = (std::abs(x_val - y_val) > 300) ? std::max(x_val, y_val) : (x_val + y_val)/2;
    	else
    	  ave_img.at<unsigned short>(y,x) = 0;
      }
    }    

    sensor_msgs::Image::Ptr converted_msg
      = cv_bridge::CvImage(image_msg->header,
			   sensor_msgs::image_encodings::TYPE_16UC1,
			   ave_img).toImageMsg();
    pub_.publish(converted_msg);
  }  
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::DepthImageCompletion, nodelet::Nodelet);
