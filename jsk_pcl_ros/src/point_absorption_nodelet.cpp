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

#define BOOST_PARAMETER_MAX_ARITY 7
#include "jsk_pcl_ros/point_absorption.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl/common/centroid.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/TextureMesh.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/png_io.h>
#include <pcl/filters/filter.h>
#include "jsk_pcl_ros/pcl_conversion_util.h"
#include <pcl/point_types.h>

namespace jsk_pcl_ros
{
  void PointAbsorption::absorpt(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if(mc_array.size() > 0){
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(*input, *cloud);

      if(!is_organized_){
	std::vector<int> aindices;
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, aindices);
      }
    
      std::vector<std::vector<int> > mc_index_array;
      for(int i = 0; i < mc_array.size(); i++)
	mc_index_array.push_back(std::vector<int>());
      for(int i = 0; i < cloud->points.size(); i++){
	pcl::PointXYZRGB point = cloud->points[i];
	  double min_distance = distance_thres_;
	  int index = -1;
	  // ROS_ERROR("i : %d", i);
	  for(int j = 0; j < mc_array.size(); j++){
	    pcl::ModelCoefficients mc = mc_array[j];
	    double a = mc.values[0];
	    double b = mc.values[1];
	    double c = mc.values[2];
	    double d = mc.values[3];
	    double distance = std::abs(a * point.x + b * point.y + c * point.z + d)/sqrt(a*a + b*b + c*c);
	    // ROS_ERROR("     j: %d %f (%f %f %f %f in x %f y %f z %f) abs_r %f / sqrt %f", j, distance, a, b, c, d, point.x, point.y, point.z, abs_r, sqrt_r);
	    if (distance < distance_thres_ && min_distance > distance){
	      index = j;
	      min_distance = distance;
	    }
	  }
	  if (index != -1)
	    mc_index_array[index].push_back(i);
	// ROS_ERROR("ANSWER(%d) : %d %lf", i, index, min_distance);
      }
      ROS_ERROR("Done CHECKING");

      if(! is_organized_){
	//Projection
	pcl::PointCloud<pcl::PointXYZRGB> projected_cloud;
	pcl::PointIndices::Ptr indices(new pcl::PointIndices);
	for(int i = 0; i < mc_array.size(); i++){
	  // ROS_ERROR(" size  %d  (%d / %d)", (int)mc_index_array[i].size(), i , (int)mc_array.size());
	  if(mc_index_array[i].size() > 0){
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
	    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
	    proj.setModelType (pcl::SACMODEL_PLANE);
	    proj.setInputCloud (cloud);
	    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	    inliers->indices = mc_index_array[i];
	    proj.setIndices(inliers);
	    indices->indices.insert(indices->indices.end(), inliers->indices.begin(),inliers->indices.end());
	    pcl::ModelCoefficients::Ptr mcp(new pcl::ModelCoefficients);
	    mcp->values = mc_array[i].values;
	    proj.setModelCoefficients (mcp);
	    proj.filter (*cloud_projected);

	    ROS_ERROR("Creating Mesh %d", i);
	    createMesh(cloud_projected, inliers);
	    ROS_ERROR("Creating Mesh done %d", i);
	    // ROS_ERROR("Projected Size of %d  [ %d ] (MC_INDEX %d)", i ,(int)cloud_projected->points.size(), (int)mc_index_array[i].size());
	    projected_cloud += *cloud_projected;
	  }
	}

	//Extract not included
	pcl::PointCloud<pcl::PointXYZRGB> extracted_cloud;
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (indices);
	extract.setNegative (true);
	extract.filter (extracted_cloud);

	//Update Cloud
	ROS_ERROR("Done PROJECTIONS");

	//publish cloud
	sensor_msgs::PointCloud2 in_pointcloud2;
	pcl::toROSMsg(projected_cloud, in_pointcloud2);
	in_pointcloud2.header.frame_id = input->header.frame_id;
	in_pointcloud2.header.stamp = input->header.stamp;
	pub_included_.publish(in_pointcloud2);
 
	sensor_msgs::PointCloud2 ex_pointcloud2;
	pcl::toROSMsg(extracted_cloud, ex_pointcloud2);
	ex_pointcloud2.header.frame_id = input->header.frame_id;
	ex_pointcloud2.header.stamp = input->header.stamp;
	pub_excluded_.publish(ex_pointcloud2);

	projected_cloud += extracted_cloud;
	sensor_msgs::PointCloud2 pointcloud2;
	pcl::toROSMsg(projected_cloud, pointcloud2);
	pointcloud2.header.frame_id = input->header.frame_id;
	pointcloud2.header.stamp = input->header.stamp;
	pub_.publish(pointcloud2);
      }
      //WHEN ORGANIZED!!!
      else{
	pcl::PointCloud<pcl::PointXYZRGB> projected_cloud;
	pcl::PointIndices::Ptr indices(new pcl::PointIndices);
	for(int i = 0; i < mc_array.size(); i++){
	  // ROS_ERROR(" size  %d  (%d / %d)", (int)mc_index_array[i].size(), i , (int)mc_array.size());
	  if(mc_index_array[i].size() > 0){
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
	    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
	    proj.setModelType (pcl::SACMODEL_PLANE);
	    proj.setInputCloud (cloud);
	    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	    inliers->indices = mc_index_array[i];
	    proj.setIndices(inliers);
	    indices->indices.insert(indices->indices.end(), inliers->indices.begin(),inliers->indices.end());
	    pcl::ModelCoefficients::Ptr mcp(new pcl::ModelCoefficients);
	    mcp->values = mc_array[i].values;
	    proj.setModelCoefficients (mcp);
	    proj.filter (*cloud_projected);
	    // ROS_ERROR("Projected Size of %d  [ %d ] (MC_INDEX %d)", i ,(int)cloud_projected->points.size(), (int)mc_index_array[i].size());

	    //Update PointCLoud
	    projected_cloud += *cloud_projected;
	    for(int j =0; j <mc_index_array[i].size(); j++){
	      cloud->points[mc_index_array[i][j]] = cloud_projected->points[j];
	    }
	  }
	}

	sensor_msgs::PointCloud2 in_pointcloud2;
	pcl::toROSMsg(projected_cloud, in_pointcloud2);
	in_pointcloud2.header.frame_id = input->header.frame_id;
	in_pointcloud2.header.stamp = input->header.stamp;
	pub_included_.publish(in_pointcloud2);

	sensor_msgs::PointCloud2 pointcloud2;
	pcl::toROSMsg(*cloud, pointcloud2);
	pointcloud2.header.frame_id = input->header.frame_id;
	pointcloud2.header.stamp = input->header.stamp;
	pub_.publish(pointcloud2);
      }
    }
  }

  void PointAbsorption::createMesh(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
				   const pcl::PointIndices::Ptr& indices)
  {
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (cloud);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.03);
    ne.compute (*cloud_normals);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr
      all_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields(*cloud, *cloud_normals, *all_cloud);

    ROS_ERROR("Done NormalEstimate");

    // pcl::PolygonMesh triangles;
    // ofm.setInputCloud (cloud);
    // ofm.setIndices(indices);
    // ofm.reconstruct (triangles);
    // pcl::io::savePolygonFileSTL("test.stl",triangles);

    pcl::PolygonMesh triangles;
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2;
    tree2.reset (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree2->setInputCloud (all_cloud);

    // Set parameters
    gp3.setInputCloud (all_cloud);
    gp3.setSearchMethod (tree2);
    // gp3.setSearchRadius (0.05);
    // //gp3.setIndices(indices);
    // gp3.setMu (3);
    // gp3.setMaximumNearestNeighbors (100);
    // gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    // gp3.setMinimumAngle(M_PI/18); // 10 degrees
    // gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    // gp3.setNormalConsistency(false);

    gp3.reconstruct (triangles);

    // ROS_ERROR("Done Recostruct");
    // pcl::PointCloud<pcl::PointXYZRGBNormal> extracted_cloud;
    // pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
    // extract.setInputCloud (all_cloud);
    // extract.setIndices (indices);
    // extract.setNegative (true);
    // extract.filter (extracted_cloud);
    // pcl::PCLPointCloud2 output_source;
    // toPCLPointCloud2 (extracted_cloud, output_source);

    pcl::TextureMesh tex_mesh;
    std::vector<std::string> tex_files;
    tex_mesh.cloud = triangles.cloud;
    tex_files.push_back("tex4.png");
    tex_mesh.tex_polygons.push_back(triangles.polygons);

    pcl::TextureMapping<pcl::PointXYZ> tm;
    tm.setF(0.01);
    tm.setVectorField(1, 0, 0);

    pcl::TexMaterial tex_material;
    tex_material.tex_Ka.r = 0.2f;
    tex_material.tex_Ka.g = 0.2f;
    tex_material.tex_Ka.b = 0.2f;

    tex_material.tex_Kd.r = 0.8f;
    tex_material.tex_Kd.g = 0.8f;
    tex_material.tex_Kd.b = 0.8f;

    tex_material.tex_Ks.r = 1.0f;
    tex_material.tex_Ks.g = 1.0f;
    tex_material.tex_Ks.b = 1.0f;
    tex_material.tex_d = 1.0f;
    tex_material.tex_Ns = 0.0f;
    tex_material.tex_illum = 2;

    tm.setTextureMaterials(tex_material);

    tm.setTextureFiles(tex_files);
    tm.mapTexture2MeshUV(tex_mesh);
    ROS_ERROR("Done maptexture2mesh");

    pcl::io::savePNGFile<pcl::PointXYZRGB>("tex4.png", *cloud);
    pcl::io::saveOBJFile ("point_absorption.obj", tex_mesh);
  }

  void PointAbsorption::getPolygon(const jsk_recognition_msgs::ModelCoefficientsArray &input)
  {
    boost::mutex::scoped_lock lock(mutex_);
    mc_array.clear();
    for (int i = 0; i < input.coefficients.size(); i++){
      pcl::ModelCoefficients mc;
      mc.values = input.coefficients[i].values;
      mc_array.push_back(mc);
    }
  }

  void PointAbsorption::subscribe()
  {
    sub_input_ = pnh_->subscribe("input", 1, &PointAbsorption::absorpt, this);
    sub_polygon_ = pnh_->subscribe("input_polygon", 1, &PointAbsorption::getPolygon, this);
  }

  void PointAbsorption::unsubscribe()
  {
    sub_input_.shutdown();
  }
  
  void PointAbsorption::onInit(void)
  {
    DiagnosticNodelet::onInit();
    pnh_->param("distance_thres", distance_thres_, 0.1);
    pnh_->param("is_organized", is_organized_, true);
    pub_ = pnh_->advertise<sensor_msgs::PointCloud2>("output", 1);
    pub_included_ = pnh_->advertise<sensor_msgs::PointCloud2>("output_included", 1);
    if(!is_organized_){
      pub_excluded_ = pnh_->advertise<sensor_msgs::PointCloud2>("output_excluded", 1);
    }
    pnh_->param("triangle_pixel_size", triangle_pixel_size_, 3.0);
    pnh_->param("max_edge_length", max_edge_length_, 4.5);
    pnh_->param("store_shadow_faces", store_shadow_faces_, true);

    ofm.setTrianglePixelSize (triangle_pixel_size_);
    // ofm.setTriangulationType (pcl::OrganizedFastMesh<pcl::PointXYZRGBNormal>::TRIANGLE_ADAPTIVE_CUT);
    ofm.setTriangulationType (pcl::OrganizedFastMesh<pcl::PointXYZRGB>::QUAD_MESH);
    // ofm.setMaxEdgeLength(max_edge_length_);
    // ofm.storeShadowedFaces(store_shadow_faces_);


    pnh_->param("search_radius", search_radius_, 0.1);
    pnh_->param("mu", mu_, 2.5);
    pnh_->param("max_nn", max_nn_, 100);
    pnh_->param("max_sur_angle", max_sur_angle_, M_PI/4);
    pnh_->param("min_an", min_an_, M_PI/18);
    pnh_->param("max_an", max_an_, 2*M_PI/3);
    pnh_->param("normal_consistensy", normal_consistensy_, true);

    gp3.setSearchRadius (search_radius_);
    //gp3.setIndices(indices);
    gp3.setMu (mu_);
    gp3.setMaximumNearestNeighbors (max_nn_);
    gp3.setMaximumSurfaceAngle(max_sur_angle_);//M_PI/4); // 45 degrees
    gp3.setMinimumAngle(min_an_);//M_PI/18); // 10 degrees
    gp3.setMaximumAngle(max_an_);//;2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(normal_consistensy_);//false);


    subscribe();
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::PointAbsorption, nodelet::Nodelet);
