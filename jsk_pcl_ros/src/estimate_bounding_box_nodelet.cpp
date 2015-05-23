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
#include "jsk_pcl_ros/estimate_bounding_box.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl/common/centroid.h>
#include "jsk_pcl_ros/pcl_conversion_util.h"

namespace jsk_pcl_ros
{
  void EstimateBoundingBox::extract(const sensor_msgs::PointCloud2ConstPtr &input_cloud
                                    ,const PCLIndicesMsg::ConstPtr &input_indices)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_cloud, *cloud);
    // pcl::fromROSMsg(*input, *cloud_xyz);
    // cluster_counter_.add(indices_input->cluster_indices.size());

    jsk_recognition_msgs::PolygonArrayPtr planes(new jsk_recognition_msgs::PolygonArray());
    jsk_recognition_msgs::ModelCoefficientsArrayPtr coefficients(new jsk_recognition_msgs::ModelCoefficientsArray());

    pcl::IndicesPtr vindices;
    vindices.reset (new std::vector<int> (input_indices->indices));
    std::vector<int> indices_nana;
    cloud->is_dense = false;
    pcl::removeNaNFromPointCloud(
      *cloud, *cloud, indices_nana);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    extract.setInputCloud (cloud);
    extract.setIndices (vindices);
    extract.setNegative (false);
    extract.filter (*segmented_cloud);
    segmented_cloud->is_dense = false;
    pcl::removeNaNFromPointCloud(
      *segmented_cloud, *segmented_cloud, indices_nana);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (segmented_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.02);
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (segmented_cloud);
    ec.extract (cluster_indices);

    if(calc_bbox_){
      if(cluster_indices.size() == 1){        
      jsk_recognition_msgs::BoundingBox bounding_box;
      Eigen::Vector4f center;
        pcl::compute3DCentroid(*segmented_cloud, center);

        bool successp = computeBoundingBox(segmented_cloud, input_cloud->header, center, planes, coefficients, bounding_box);
        jsk_recognition_msgs::BoundingBoxArray bbox_array;
        bbox_array.header.frame_id = input_cloud->header.frame_id;
        bbox_array.header.stamp = input_cloud->header.stamp;
        bbox_array.boxes.push_back(bounding_box);
        
	box_array_pub_.publish(bbox_array);

        if (publish_tf_) {
          tf::Transform transform;
          transform.setOrigin(tf::Vector3(center[0], center[1], center[2]));
          transform.setRotation(tf::createIdentityQuaternion());
          br_.sendTransform(tf::StampedTransform(transform, input_cloud->header.stamp,
                                                 input_cloud->header.frame_id, frame_));
        }
        geometry_msgs::PoseStamped pose;
        pose.pose.orientation.w = 1.0;
        pose.pose.position.x = center[0];
        pose.pose.position.y = center[1];
        pose.pose.position.z = center[2];
        pose.header = input_cloud->header;
	//        pub_pose_.publish(pose);
        geometry_msgs::PointStamped point;
        point.point.x = center[0];
        point.point.y = center[1];
        point.point.z = center[2];
        point.header = input_cloud->header;
	//        pub_point_.publish(point);
      }else if(cluster_indices.size() > 1){

        jsk_recognition_msgs::BoundingBoxArray bbox_array;
        for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
          jsk_recognition_msgs::BoundingBox bounding_box;
          Eigen::Vector4f center;
	  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	  inliers->indices = it->indices;

	  extract.setInputCloud (segmented_cloud);
	  extract.setIndices (inliers);
	  extract.setNegative (false);
	  extract.filter (*segmented_cloud2);
          
          bool successp = computeBoundingBox(segmented_cloud2, input_cloud->header, center, planes, coefficients, bounding_box);
          bbox_array.boxes.push_back(bounding_box);
        }
	if(bbox_array.boxes.size()){
	  bbox_array.header.frame_id = input_cloud->header.frame_id;
	  bbox_array.header.stamp = input_cloud->header.stamp;
	  box_array_pub_.publish(bbox_array);
	}

      }
    }
    else{
      if(calc_centroid_){
      Eigen::Vector4f center;
        pcl::compute3DCentroid(*segmented_cloud, center);
        if (publish_tf_) {
          tf::Transform transform;
          transform.setOrigin(tf::Vector3(center[0], center[1], center[2]));
          transform.setRotation(tf::createIdentityQuaternion());
          br_.sendTransform(tf::StampedTransform(transform, input_cloud->header.stamp,
                                                 input_cloud->header.frame_id, frame_));
        }
        geometry_msgs::PoseStamped pose;
        pose.pose.orientation.w = 1.0;
        pose.pose.position.x = center[0];
        pose.pose.position.y = center[1];
        pose.pose.position.z = center[2];
        pose.header = input_cloud->header;
	//        pub_pose_.publish(pose);
        geometry_msgs::PointStamped point;
        point.point.x = center[0];
        point.point.y = center[1];
        point.point.z = center[2];
        point.header = input_cloud->header;
	//        pub_point_.publish(point);
      }
    }
    if (publish_extract_){
      sensor_msgs::PointCloud2 debug_ros_output;
      pcl::toROSMsg(*segmented_cloud, debug_ros_output);
      debug_ros_output.header = input_cloud->header;
      debug_ros_output.is_dense = false;
      pub_extract_.publish(debug_ros_output);
    }

  }

  void EstimateBoundingBox::subscribe()
  {
    sub_input_points_.subscribe(*pnh_, "points", 1);
    sub_input_indices_.subscribe(*pnh_, "indices", 1);

    async_ = boost::make_shared<message_filters::Synchronizer<ASyncPolicy> >(100);
    async_->connectInput(sub_input_points_, sub_input_indices_);
    async_->registerCallback(boost::bind(&EstimateBoundingBox::extract,
                                         this, _1, _2));

  }

  void EstimateBoundingBox::unsubscribe()
  {
    sub_input_.shutdown();
  }
  
  void EstimateBoundingBox::onInit(void)
  {
    DiagnosticNodelet::onInit();
    pnh_->param("publish_tf", publish_tf_, false);
    pnh_->param("publish_extract", publish_extract_, true);
    pnh_->param("calc_bbox", calc_bbox_, true);
    pnh_->param("calc_centroid", calc_centroid_, false);
    pnh_->param("force_to_flip_z_axis", force_to_flip_z_axis_, false);
    box_array_pub_ = pnh_->advertise<jsk_recognition_msgs::BoundingBoxArray>("box_array", 1);

    pub_pose_ = pnh_->advertise<geometry_msgs::PoseStamped>("output/pose", 1);
    pub_point_ = pnh_->advertise<geometry_msgs::PointStamped>("output/point", 1);
    pub_extract_ = pnh_->advertise<sensor_msgs::PointCloud2>("output/extract_points", 1);
    subscribe();
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::EstimateBoundingBox, nodelet::Nodelet);
