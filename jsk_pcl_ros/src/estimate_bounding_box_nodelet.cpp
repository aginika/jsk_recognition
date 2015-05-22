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
  bool EstimateBoundingBox::computeBoundingBox
  (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud,
   const std_msgs::Header header,
   const Eigen::Vector4f center,
   const jsk_recognition_msgs::PolygonArrayConstPtr& planes,
   const jsk_recognition_msgs::ModelCoefficientsArrayConstPtr& coefficients,
   jsk_recognition_msgs::BoundingBox& bounding_box)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      segmented_cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
    // align boxes if possible
    Eigen::Matrix4f m4 = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    if (align_boxes_) {
      int nearest_plane_index = findNearestPlane(center, planes, coefficients);
      if (nearest_plane_index == -1) {
        segmented_cloud_transformed = segmented_cloud;
        JSK_NODELET_ERROR("no planes to align boxes are given");
      }
      else {
        Eigen::Vector3f normal, z_axis;
        if (force_to_flip_z_axis_) {
          normal[0] = - coefficients->coefficients[nearest_plane_index].values[0];
          normal[1] = - coefficients->coefficients[nearest_plane_index].values[1];
          normal[2] = - coefficients->coefficients[nearest_plane_index].values[2];
        }
        else {
          normal[0] = coefficients->coefficients[nearest_plane_index].values[0];
          normal[1] = coefficients->coefficients[nearest_plane_index].values[1];
          normal[2] = coefficients->coefficients[nearest_plane_index].values[2];
        }
        normal = normal.normalized();
        Eigen::Quaternionf rot;
        rot.setFromTwoVectors(Eigen::Vector3f::UnitZ(), normal);
        Eigen::AngleAxisf rotation_angle_axis(rot);
        Eigen::Vector3f rotation_axis = rotation_angle_axis.axis();
        double theta = rotation_angle_axis.angle();
        if (isnan(theta) ||
            isnan(rotation_axis[0]) ||
            isnan(rotation_axis[1]) ||
            isnan(rotation_axis[2])) {
          segmented_cloud_transformed = segmented_cloud;
          JSK_NODELET_ERROR("cannot compute angle to align the point cloud: [%f, %f, %f], [%f, %f, %f]",
                            z_axis[0], z_axis[1], z_axis[2],
                            normal[0], normal[1], normal[2]);
        }
        else {
          Eigen::Matrix3f m = Eigen::Matrix3f::Identity() * rot;
          if (use_pca_) {
            // first project points to the plane
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud
              (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::ProjectInliers<pcl::PointXYZRGB> proj;
            proj.setModelType (pcl::SACMODEL_PLANE);
            pcl::ModelCoefficients::Ptr
              plane_coefficients (new pcl::ModelCoefficients);
            plane_coefficients->values
              = coefficients->coefficients[nearest_plane_index].values;
            proj.setModelCoefficients(plane_coefficients);
            proj.setInputCloud(segmented_cloud);
            proj.filter(*projected_cloud);
            if (projected_cloud->points.size() >= 3) {
              pcl::PCA<pcl::PointXYZRGB> pca;
              pca.setInputCloud(projected_cloud);
              Eigen::Matrix3f eigen = pca.getEigenVectors();
              m.col(0) = eigen.col(0);
              m.col(1) = eigen.col(1);
              // flip axis to satisfy right-handed system
              if (m.col(0).cross(m.col(1)).dot(m.col(2)) < 0) {
                m.col(0) = - m.col(0);
              }
              if (m.col(0).dot(Eigen::Vector3f::UnitX()) < 0) {
                // rotate around z
                m = m * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());
              }
            }
            else {
              JSK_NODELET_ERROR("Too small indices for PCA computation");
              return false;
            }
          }
          // m4 <- m
          for (size_t row = 0; row < 3; row++) {
            for (size_t column = 0; column < 3; column++) {
              m4(row, column) = m(row, column);
            }
          }
          q = m;
          Eigen::Matrix4f inv_m = m4.inverse();
          pcl::transformPointCloud(*segmented_cloud, *segmented_cloud_transformed, inv_m);
        }
      }
    }
    else {
      segmented_cloud_transformed = segmented_cloud;
    }

    // create a bounding box
    Eigen::Vector4f minpt, maxpt;
    pcl::getMinMax3D<pcl::PointXYZRGB>(*segmented_cloud_transformed, minpt, maxpt);

    double xwidth = maxpt[0] - minpt[0];
    double ywidth = maxpt[1] - minpt[1];
    double zwidth = maxpt[2] - minpt[2];
    Eigen::Vector4f center2((maxpt[0] + minpt[0]) / 2.0, (maxpt[1] + minpt[1]) / 2.0, (maxpt[2] + minpt[2]) / 2.0, 1.0);
    Eigen::Vector4f center_transformed = m4 * center2;
    bounding_box.header = header;
    bounding_box.pose.position.x = center_transformed[0];
    bounding_box.pose.position.y = center_transformed[1];
    bounding_box.pose.position.z = center_transformed[2];
    bounding_box.pose.orientation.x = q.x();
    bounding_box.pose.orientation.y = q.y();
    bounding_box.pose.orientation.z = q.z();
    bounding_box.pose.orientation.w = q.w();
    bounding_box.dimensions.x = xwidth;
    bounding_box.dimensions.y = ywidth;
    bounding_box.dimensions.z = zwidth;
    return true;
  }

  void EstimateBoundingBox::extract(const pcl_msgs::PointIndicesPtr& input)
  {
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);

    if(calc_bbox){
      jsk_recognition_msgs::BoundingBox bounding_box;
      bool successp = computeBoundingBox(segmented_cloud, input->header, center, planes, coefficients, bounding_box);
      box_pub_.publish(bounding_box);
    }


    if(calc_centroid){
      pcl::compute3DCentroid(cloud_xyz, center);
      if (publish_tf_) {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(center[0], center[1], center[2]));
        transform.setRotation(tf::createIdentityQuaternion());
        br_.sendTransform(tf::StampedTransform(transform, input->header.stamp,
                                               input->header.frame_id, frame_));
      }
      geometry_msgs::PoseStamped pose;
      pose.pose.orientation.w = 1.0;
      pose.pose.position.x = center[0];
      pose.pose.position.y = center[1];
      pose.pose.position.z = center[2];
      pose.header = input->header;
      pub_pose_.publish(pose);
      geometry_msgs::PointStamped point;
      point.point.x = center[0];
      point.point.y = center[1];
      point.point.z = center[2];
      point.header = input->header;
      pub_point_.publish(point);
    }

  }

  void EstimateBoundingBox::subscribe()
  {
    sub_input_ = pnh_->subscribe("input", 1, &EstimateBoundingBox::extract, this);
  }

  void EstimateBoundingBox::unsubscribe()
  {
    sub_input_.shutdown();
  }
  
  void EstimateBoundingBox::onInit(void)
  {
    DiagnosticNodelet::onInit();
    pnh_->param("publish_tf", publish_tf_, false);
    if (calc_bbox)
      box_pub_ = advertise<jsk_recognition_msgs::BoundingBox>(*pnh_, "box", 1);

    if (publish_tf_ && calc_centroid) {
      if (!pnh_->getParam("frame", frame_))
        {
          JSK_ROS_WARN("~frame is not specified, using %s", getName().c_str());
          frame_ = getName();
        }
      pub_pose_ = pnh_->advertise<geometry_msgs::PoseStamped>("output/pose", 1);
      pub_point_ = pnh_->advertise<geometry_msgs::PointStamped>("output/point", 1);
      subscribe();
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::EstimateBoundingBox, nodelet::Nodelet);
