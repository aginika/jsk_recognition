// -*- mode: C++ -*-
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

#include "jsk_pcl_ros/grasp_object_region_growing.h"

#include "jsk_pcl_ros/ClusterPointIndices.h"
#include <pcl/segmentation/impl/region_growing.hpp>
#include <pluginlib/class_list_macros.h>

#include "jsk_pcl_ros/pcl_conversion_util.h"

namespace jsk_pcl_ros
{

  void GraspObjectRegionGrowing::onInit()
  {
    PCLNodelet::onInit();
    if (!pnh_->getParam("rarm_hand", rarm_hand_))
      {
        rarm_hand_ = "";
        ROS_ERROR("You need to set ~rarm_hand param.");
        exit(-1);
      }
    if (!pnh_->getParam("larm_hand", larm_hand_))
      {
        larm_hand_ = "";
        ROS_ERROR("You need to set ~larm_hand param.");
        exit(-1);
      }
    if (!pnh_->getParam("search_radius", radius_))
      {
        radius_ = 0.05;
        ROS_ERROR("You need to set ~search_radius param.");
        exit(-1);
      }

    tf_listener_.reset(new tf::TransformListener);
    pub_ = advertise<jsk_pcl_ros::ClusterPointIndices>(*pnh_, "output", 1);
  }

  void GraspObjectRegionGrowing::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &GraspObjectRegionGrowing::segment, this);
  }

  void GraspObjectRegionGrowing::unsubscribe()
  {
    sub_.shutdown();
  }

  pcl::PointXYZ GraspObjectRegionGrowing::getDummyHandPositionPoint(std::string input_frame, ros::Time stamp, bool right)
  {
    tf::StampedTransform transform;
    if(right)
      tf_listener_->lookupTransform(input_frame, rarm_hand_, stamp, transform);
    else
      tf_listener_->lookupTransform(input_frame, larm_hand_, stamp, transform);

    pcl::PointXYZ point;
    tf::Vector3 trans = transform.getOrigin();
    point.x = (float)trans.getX();
    point.y = (float)trans.getY();
    point.z = (float)trans.getZ();
    return point;
  }

  void GraspObjectRegionGrowing::segment(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    //Get Nearest Point From input cloud with radius.
    pcl::KdTreeFLANN<pcl::PointXYZ> ktree;
    ktree.setInputCloud (cloud);

    //Right
    std::vector<int> right_idx (1);
    std::vector<float> right_dists (1);

    pcl::PointXYZ point = getDummyHandPositionPoint(msg->header.frame_id, msg->header.stamp, true);
    ktree.radiusSearch (point, radius_, right_idx, right_dists);

    //Left
    std::vector<int> left_idx (1);
    std::vector<float> left_dists (1);
    point = getDummyHandPositionPoint(msg->header.frame_id, msg->header.stamp, false);
    ktree.radiusSearch (point, radius_, left_idx, left_dists);


    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (min_size_);
    reg.setMaxClusterSize (max_size_);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (number_of_neighbors_);
    reg.setInputCloud (cloud);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (smoothness_threshold_);
    reg.setCurvatureThreshold (curvature_threshold_);

    //For right
    pcl::PointIndices::Ptr right_inliers (new pcl::PointIndices ());
    for(int i = 0 ; i < right_idx.size(); i++){
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
      reg.getSegmentFromPoint(i, *inliers);
      right_inliers->indices.insert(right_inliers->indices.end(), inliers->indices.begin(), inliers->indices.end());
    }
    sort(right_inliers->indices.begin(), right_inliers->indices.end());
    right_inliers->indices.erase(unique(right_inliers->indices.begin(), right_inliers->indices.end()), right_inliers->indices.end());

    //For left
    pcl::PointIndices::Ptr left_inliers (new pcl::PointIndices ());
    for(int i = 0 ; i < left_idx.size(); i++){
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
      reg.getSegmentFromPoint(i, *inliers);
      left_inliers->indices.insert(left_inliers->indices.end(), inliers->indices.begin(), inliers->indices.end());
    }
    sort(left_inliers->indices.begin(), left_inliers->indices.end());
    left_inliers->indices.erase(unique(left_inliers->indices.begin(), left_inliers->indices.end()), left_inliers->indices.end());

    jsk_pcl_ros::ClusterPointIndices output;
    output.header = msg->header;

    PCLIndicesMsg right_indices;
    right_indices.header = msg->header;
    right_indices.indices = right_inliers->indices;
    output.cluster_indices.push_back(right_indices);
    PCLIndicesMsg left_indices;
    left_indices.header = msg->header;
    left_indices.indices = right_inliers->indices;
    output.cluster_indices.push_back(left_indices);

    pub_.publish(output);
  }
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::GraspObjectRegionGrowing,
                        nodelet::Nodelet);
