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

#ifndef JSK_PCL_ROS_GRASP_OBJECT_REGION_GROWING_H_
#define JSK_PCL_ROS_GRASP_OBJECT_REGION_GROWING_H_

#include <pcl_ros/pcl_nodelet.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <dynamic_reconfigure/server.h>
#include "jsk_pcl_ros/RegionGrowingSegmentationConfig.h"

#include <jsk_topic_tools/connection_based_nodelet.h>
#include <tf/transform_broadcaster.h>

namespace jsk_pcl_ros
{
  class GraspObjectRegionGrowing : public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
  protected:
    ros::Publisher pub_;
    ros::Publisher right_pub_;
    ros::Publisher left_pub_;
    ros::Subscriber sub_;
    int number_of_neighbors_;
    int min_size_;
    int max_size_;
    double smoothness_threshold_;
    double curvature_threshold_;
    typedef jsk_pcl_ros::RegionGrowingSegmentationConfig Config;
    double radius_;
    boost::shared_ptr<tf::TransformListener> tf_listener_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;

    std::string rarm_hand_;
    std::string larm_hand_;
    boost::mutex mutex_;
    virtual void segment(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void subscribe();
    virtual void unsubscribe();
    virtual pcl::PointNormal getDummyHandPositionPoint(std::string input_frame, ros::Time stamp, bool right = true);
    virtual void configCallback (Config &config, uint32_t level);
  private:
    virtual void onInit();
  };
}

#endif