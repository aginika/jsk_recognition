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
#include "jsk_pcl_ros/pointcloud_to_marker.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl/common/centroid.h>
#include "jsk_pcl_ros/pcl_conversion_util.h"

namespace jsk_pcl_ros
{
  void PointCloudToMarker::extract(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*input, cloud);

    visualization_msgs::Marker marker;
    marker.header.frame_id = input->header.frame_id;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;

    ROS_ERROR("Create Now.");
    std::vector<geometry_msgs::Point> points;
    std::vector<std_msgs::ColorRGBA> colors;
    int counter=0;
    for(int i = 0; i < cloud.width - (1+mesh_skip_); i+=(1+mesh_skip_)){
      for(int j = 0; j < cloud.height - (1+mesh_skip_); j+=(1+mesh_skip_)){
	counter++;
	pcl::PointXYZRGB point;
	geometry_msgs::Point pos;
	std_msgs::ColorRGBA color;
	
	std::vector<int> indices;
	indices.push_back(i);
	indices.push_back(j);
	indices.push_back(i);
	indices.push_back(j+(1+mesh_skip_));
	indices.push_back(i+(1+mesh_skip_));
	indices.push_back(j+(1+mesh_skip_));
	indices.push_back(i);
	indices.push_back(j);
	indices.push_back(i+(1+mesh_skip_));
	indices.push_back(j+(1+mesh_skip_));
	indices.push_back(i+(1+mesh_skip_));
	indices.push_back(j);

	for (int k = 0; k < 6 ; k++){
	  point = cloud.at(indices[2*k], indices[2*k+1]);

	  //Include NAN
	  if(!pcl_isfinite (point.x) ||
	     !pcl_isfinite (point.y) ||
	     !pcl_isfinite (point.z)){
	    for(int x = 0; x < k ; x++){
	      points.pop_back();
	      colors.pop_back();
	    }
	    counter--;
	    break;
	  }

	  pos.x = point.x;
	  pos.y = point.y;
	  pos.z = point.z;

	  //Too Far
	  if(points.size() > 0 && k != 0){
	    geometry_msgs::Point latest = points.back();
	    geometry_msgs::Point diff;
	    diff.x = pos.x - latest.x;
	    diff.y = pos.y - latest.y;
	    diff.z = pos.z - latest.z;
	    if(sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z) > thres_){
	      for(int x = 0; x < k ; x++){
	  	points.pop_back();
	  	colors.pop_back();
	      }
	      counter--;
	      break;
	    }
	  }
	    
	  color.r = point.r/255.0;
	  color.g = point.g/255.0;
	  color.b = point.b/255.0;
	  color.a = 1;
	  points.push_back(pos);
	  colors.push_back(color);
	}
      }
    }
    marker.points = points;
    marker.colors = colors;
    pub_.publish(marker);
  }

  void PointCloudToMarker::subscribe()
  {
    sub_input_ = pnh_->subscribe("input", 1, &PointCloudToMarker::extract, this);
  }

  void PointCloudToMarker::unsubscribe()
  {
    sub_input_.shutdown();
  }
  
  void PointCloudToMarker::onInit(void)
  {
    DiagnosticNodelet::onInit();
    pnh_->param("thres", thres_, 1.0);
    pnh_->param("mesh_skip", mesh_skip_, 0);
    pub_ = pnh_->advertise<visualization_msgs::Marker>("output", 1);
    subscribe();
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::PointCloudToMarker, nodelet::Nodelet);
