/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
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

#define BOOST_PARAMETER_MAX_ARITY 7
#include "jsk_pcl_ros/nearest_neighbor_points.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/common/centroid.h>
#include <pcl/search/organized.h>
#include "jsk_pcl_ros/pcl_conversion_util.h"
#include <jsk_topic_tools/rosparam_utils.h>

namespace jsk_pcl_ros
{
  void NearestNeighborPoints::extract(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input, *cloud);

    // std::vector<int> indices;
    // cloud->is_dense = false;
    // pcl::removeNaNFromPointCloud(
    //   *cloud, *cloud, indices);
    // pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    pcl::search::OrganizedNeighbor<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    jsk_recognition_msgs::PointsArray pa;
    pa.header.frame_id = input->header.frame_id;
    pa.header.stamp = input->header.stamp;
    for(int i = 0; i < tfs_.size(); i++){
      std::string frame_id = tfs_[i];
      tf_listener_->waitForTransform(frame_id, input->header.frame_id, input->header.stamp,
				     ros::Duration(1.0));
      
      try{
	tf::StampedTransform transform;
	tf_listener_->lookupTransform(input->header.frame_id, frame_id, 
				      input->header.stamp, transform);
	

	tf::Vector3 pos = transform.getOrigin();
	pcl::PointXYZRGB p;
	p.x = (float)pos.x();
	p.y = (float)pos.y();
	p.z = (float)pos.z();

	geometry_msgs::PoseStamped pose;
	pose.pose.orientation.w = 1.0;
	pose.pose.position.x = p.x;
	pose.pose.position.y = p.y;
	pose.pose.position.z = p.z;
	pose.header = input->header;
	pub_pose_.publish(pose);

	std::vector<int> near_indices;
	std::vector<float> near_distances;
	kdtree.radiusSearch(p, search_radius_, near_indices, near_distances);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr nearest_points(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	inliers->indices = near_indices;
	if(near_indices.size() > 0){
	  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	  extract.setInputCloud (cloud);
	  extract.setIndices (inliers);
	  extract.setNegative (false);
	  extract.filter (*nearest_points);

	  *all_cloud += *nearest_points;
	  sensor_msgs::PointCloud2 pointcloud2;
	  pcl::toROSMsg(*nearest_points, pointcloud2);
	  pointcloud2.header.frame_id = input->header.frame_id;
	  pointcloud2.header.stamp = input->header.stamp;
	  pa.cloud_list.push_back(pointcloud2);
	  ROS_ERROR("Add Points %d", (int)nearest_points->points.size());
	}
      }
      catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
      }
    }
    if(pa.cloud_list.size()>0){
      pub_.publish(pa);
      sensor_msgs::PointCloud2 pointcloud2;
      pcl::toROSMsg(*all_cloud, pointcloud2);
      pointcloud2.header.frame_id = input->header.frame_id;
      pointcloud2.header.stamp = input->header.stamp;
      pub_cloud_.publish(pointcloud2);
    }else{
      ROS_ERROR("PA Cloud_LIST %d", (int)pa.cloud_list.size());
    }
  }

  void NearestNeighborPoints::subscribe()
  {
    sub_input_ = pnh_->subscribe("input", 1, &NearestNeighborPoints::extract, this);
  }

  void NearestNeighborPoints::unsubscribe()
  {
    sub_input_.shutdown();
  }
  
  void NearestNeighborPoints::onInit(void)
  {
    DiagnosticNodelet::onInit();
    jsk_topic_tools::readVectorParameter(*pnh_, "tfs", tfs_);
    pnh_->param("search_radius", search_radius_, 0.1);
    pub_ = pnh_->advertise<jsk_recognition_msgs::PointsArray>("output", 1);
    pub_cloud_ = pnh_->advertise<sensor_msgs::PointCloud2>("output_cloud", 1);
    pub_pose_ = pnh_->advertise<geometry_msgs::PoseStamped>("output_pose", 1);
    tf_listener_ = TfListenerSingleton::getInstance();
    subscribe();
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::NearestNeighborPoints, nodelet::Nodelet);
