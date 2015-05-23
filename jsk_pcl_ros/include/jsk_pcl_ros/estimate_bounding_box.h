// -*- mode: C++ -*-
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

#ifndef JSK_PCL_ROS_ESTIMATE_BOUNDING_BOX_H_
#define JSK_PCL_ROS_ESTIMATE_BOUNDING_BOX_H_

// ros
#include <ros/ros.h>
#include <ros/names.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

// pcl
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include "jsk_pcl_ros/pcl_conversion_util.h"
#include "jsk_pcl_ros/pcl_util.h"
#include <jsk_pcl_ros/cluster_point_indices_decomposer.h>

#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_msgs/ModelCoefficientsArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace jsk_pcl_ros
{
  class EstimateBoundingBox: public jsk_pcl_ros::ClusterPointIndicesDecomposer//jsk_topic_tools::DiagnosticNodelet//
  {
  public:
    EstimateBoundingBox(): ClusterPointIndicesDecomposer(){} //DiagnosticNodelet("EstimateBoundingBox") {}
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2,
      PCLIndicesMsg > ASyncPolicy;

  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void extract(const sensor_msgs::PointCloud2ConstPtr &input_cloud
                         ,const PCLIndicesMsg::ConstPtr &input_indices);
    
    ros::Subscriber sub_input_;
    tf::TransformBroadcaster br_;
    std::string frame_;
    bool publish_tf_;
    bool calc_centroid_;
    bool calc_bbox_;
    bool publish_extract_;
    message_filters::Subscriber < sensor_msgs::PointCloud2 > sub_input_points_;
    message_filters::Subscriber < PCLIndicesMsg > sub_input_indices_;
    boost::shared_ptr<message_filters::Synchronizer<ASyncPolicy> > async_;

    ros::Publisher pub_pose_;
    ros::Publisher pub_point_;
    ros::Publisher pub_extract_;
    ros::Publisher box_array_pub_;
  private:
  };
}

#endif
