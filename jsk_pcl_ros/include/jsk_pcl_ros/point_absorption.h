// -*- mode: C++ -*-
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

#ifndef JSK_PCL_ROS_POINT_ABSORPTION_H_
#define JSK_PCL_ROS_POINT_ABSORPTION_H_

// ros
#include <ros/ros.h>
#include <ros/names.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

// pcl
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <jsk_recognition_msgs/ModelCoefficientsArray.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/io/obj_io.h>
#include <pcl/TextureMesh.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/surface/organized_fast_mesh.h>


namespace jsk_pcl_ros
{
  class PointAbsorption: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    PointAbsorption(): DiagnosticNodelet("PointAbsorption") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void createMesh(const sensor_msgs::PointCloud2ConstPtr& input,
			    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
			    const pcl::PointIndices::Ptr& indices);
    virtual void absorpt(const sensor_msgs::PointCloud2ConstPtr &input);
    virtual void getPolygon(const jsk_recognition_msgs::ModelCoefficientsArray &input);
    
    ros::Subscriber sub_input_;
    ros::Subscriber sub_polygon_;
    tf::TransformBroadcaster br_;
    std::string frame_;
    boost::mutex mutex_;
    ros::Publisher pub_;
    ros::Publisher pub_excluded_;
    ros::Publisher pub_included_;
    ros::Publisher pub_marker_;
    double thres_;
    int mesh_skip_;

    double distance_thres_;
    bool is_organized_;
    jsk_recognition_msgs::ModelCoefficientsArray coeffarr;
    std::vector<pcl::ModelCoefficients> mc_array;
    bool store_shadow_faces_;

    double triangle_pixel_size_;
    double max_edge_length_;
    double mu_;
    double search_radius_;
    int max_nn_;
    double max_sur_angle_;
    double min_an_;
    double max_an_;
    bool normal_consistensy_;
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;

    // pcl::OrganizedFastMesh<pcl::PointXYZRGBNormal> ofm;
    pcl::OrganizedFastMesh<pcl::PointXYZRGB> ofm;
  private:
  };
}

#endif
