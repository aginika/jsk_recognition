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
 *   * Neither the name of the Willow Garage nor the names of its
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


#include "jsk_pcl_ros/subtract_pointcloud.h"
#include <pluginlib/class_list_macros.h>

namespace jsk_pcl_ros
{
  void SubtractPointCloud::subtract(const sensor_msgs::PointCloud2ConstPtr &input)
  {
    sensor_msgs::PointCloud2 output;
    if (input_frame_ != input->header.frame_id){
      mask_pointcloud_->points.clear();      
      input_frame_ = input->header.frame_id;
      return;
    }

    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*input, *cloud);

    if(mask_pointcloud_->points.size() == 0)
      return;

    // == Subtract ==
    //collect the nearest pointclouds and extract
    //After that remove outliner pointcloud
    pcl::KdTreeFLANN<PointType> tree;
    tree.setInputCloud (cloud);

    std::vector<int> nn_idx (1);
    std::vector<float> nn_dists (1);
    std::vector<int> indices;
    for (size_t i = 0; i < mask_pointcloud_->points.size (); ++i)
      {
	tree.radiusSearch (*mask_pointcloud_, i, radius_, nn_idx, nn_dists);
	indices.insert(indices.end(), nn_idx.begin(), nn_idx.end());	
      }

    //Unique the vector
    sort(indices.begin(), indices.end());
    indices.erase(unique(indices.begin(), indices.end()), indices.end());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    inliers->indices = indices;

    //extract the pointcloud without mask
    pcl::PointCloud<PointType>::Ptr result(new pcl::PointCloud<PointType>());
    pcl::ExtractIndices<PointType> ex;
    ex.setInputCloud( cloud );
    ex.setIndices( inliers );
    ex.setNegative( true );
    ex.filter( *result );

    if (remove_outlier_){
      pcl::StatisticalOutlierRemoval<PointType> sor;
      sor.setInputCloud (result);
      sor.setMeanK (50);
      sor.setStddevMulThresh (1.0);
      sor.filter (*result);
    }

    sensor_msgs::PointCloud2 result_pc;
    pcl::toROSMsg(*result, result_pc);
    result_pc.header.frame_id = input_frame_;
    result_pc.header.stamp = ros::Time::now();

    if( result_frame_ != std::string("") ){
      pcl_ros::transformPointCloud(result_frame_, result_pc, result_pc, *tf_listener_);
    }
    result_publisher_.publish(result_pc);
  }

  void SubtractPointCloud::setMaskPointCloud(const sensor_msgs::PointCloud2ConstPtr &input)
  {
    if(input_frame_ == "")
      return;

    mask_pointcloud_.reset(new pcl::PointCloud<PointType>());
    mask_frame_ = input->header.frame_id;

    //if the input and mask frame is differenct
    // we need to transform the mask pointcloud
    sensor_msgs::PointCloud2 output;
    if(mask_frame_ != input_frame_){
      pcl_ros::transformPointCloud(mask_frame_, *input, output, *tf_listener_);
      pcl::fromROSMsg(output, *mask_pointcloud_);
    }else{
      pcl::fromROSMsg(*input, *mask_pointcloud_);
    }
  }
  
  void SubtractPointCloud::onInit(void)
  {
    PCLNodelet::onInit();
    sub_input_ = pnh_->subscribe("input", 1, &SubtractPointCloud::subtract, this);
    sub_mask_ = pnh_->subscribe("mask", 1, &SubtractPointCloud::setMaskPointCloud,this);
    result_publisher_ = pnh_->advertise<sensor_msgs::PointCloud2>("output", 1);

    tf_listener_ = TfListenerSingleton::getInstance();

    mask_pointcloud_.reset(new pcl::PointCloud<PointType>());
    if (!pnh_->getParam("result_frame", result_frame_))
    {
      result_frame_ = std::string("");
    }

    if (!pnh_->getParam("search_radius", radius_))
    {
      ROS_WARN("~search_radius is not specified, using %lf", 0.01);
      radius_ = 0.01;
    }

    if (!pnh_->getParam("remove_outlier", remove_outlier_))
    {
      remove_outlier_ = true;
    }
  }
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::SubtractPointCloud, nodelet::Nodelet);
