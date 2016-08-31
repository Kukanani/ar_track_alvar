/*
  Software License Agreement (BSD License)

  Copyright (c) 2016, Adam Allevato
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

  * Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the following
  disclaimer in the documentation and/or other materials provided
  with the distribution.
  * Neither the name of the Willow Garage nor the names of its
  contributors may be used to endorse or promote products derived
  from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

  author: Adam Allevato
*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>

//save the fingers
typedef pcl::PointXYZRGB ARPoint;
typedef pcl::PointCloud<ARPoint> ARCloud;

/// listens for incoming point clouds
ros::Subscriber cloud_sub_; 
/// publishes a pose corresponding to the position and orientation of the detected cylinder
tf::TransformBroadcaster* tf_broadcaster; 

float normalDistanceWeight;
int maxIterations;
float distanceThreshold;
float minRadius;
float maxRadius;
int kSearch;

tf::Pose createPoseFromCylinderCoefficients(pcl::ModelCoefficients::Ptr coeffs) {
  
  tf::Vector3 cylinder_position = tf::Vector3(
    coeffs->values[0],
    coeffs->values[1],
    coeffs->values[2]
  );
  tf::Vector3 up = tf::Vector3(0.0, 0.0, 1.0f);
  tf::Vector3 cylinder_axis = tf::Vector3(
    coeffs->values[3],
    coeffs->values[4],
    coeffs->values[5]
  );
  tf::Vector3 rotation_axis = up.cross(cylinder_axis);
  float rotation_angle = up.angle(cylinder_axis); 

  tf::Quaternion rotation_quat = tf::Quaternion(rotation_axis, rotation_angle);

  return tf::Pose(
    rotation_quat,
    cylinder_position
  );
}

void findCylinder(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  ARCloud::Ptr cloud = ARCloud::Ptr(new ARCloud());
  pcl::fromROSMsg(*msg, *cloud);
  if(cloud->width < 3) {
    ROS_WARN("Cloud too small!");
    return;
  }

  Eigen::Vector4f clusterCentroid;
  pcl::compute3DCentroid(*cloud, clusterCentroid);
  
  pcl::PointCloud<pcl::Normal>::Ptr cloudNormals (new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<ARPoint, pcl::Normal> ne;
  
  pcl::search::KdTree<ARPoint>::Ptr tree (new pcl::search::KdTree<ARPoint> ());
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud);
  ne.setKSearch (kSearch);
  ne.compute (*cloudNormals);
  
  pcl::SACSegmentationFromNormals<ARPoint, pcl::Normal> seg; 
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (normalDistanceWeight);
  seg.setMaxIterations (maxIterations);
  seg.setDistanceThreshold (distanceThreshold);
  ROS_INFO_STREAM("max radius: " << maxRadius << ", min radius: " << minRadius);
  seg.setRadiusLimits (minRadius, maxRadius);
  seg.setInputCloud (cloud);
  seg.setInputNormals (cloudNormals);

  pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
  seg.segment (*inliers_cylinder, *coeffs);
  if(coeffs->values.empty()) {
    return;
  }
  std::cout << "Cylinder coefficients: " << *coeffs << std::endl;

  //build the output pose
  //SACMODEL_CYLINDER - used to determine cylinder models. The seven coefficients
  // of the cylinder are given by a point on its axis, the axis direction, and a
  // radius, as:
  //    [point_on_axis.x 
  //     point_on_axis.y 
  //     point_on_axis.z 
  //     axis_direction.x 
  //     axis_direction.y 
  //     axis_direction.z 
  //     radius] 
  //via http://docs.pointclouds.org/1.7.0/group__sample__consensus.html
  tf::Pose cylinderPose = createPoseFromCylinderCoefficients(coeffs);
  tf::StampedTransform camToCylinder(cylinderPose, ros::Time::now(), msg->header.frame_id, "cylinder_center");

  tf_broadcaster->sendTransform(camToCylinder);
}

int main(int argc, char *argv[])
{
  //ROS start
  ros::init (argc, argv, "cylinder_from_tag_cloud");
  ros::NodeHandle n, pn("~");

  //set up "magic" variables. Could be easily converted to command-line params, see "radius" below
  kSearch = 8;
  normalDistanceWeight = 0.1f;
  maxIterations = 10000;
  distanceThreshold = 0.1;
  float tolerance = 0.01; // the fit cylinder can be within this amount of the nominal value
  std::string cam_image_topic = "/ar_tag_point_cloud";


  // look for the radius variable to be specified as a private node parameter
  float radius = 0.1; //10cm default
  if(pn.hasParam("radius")) {
    pn.getParam("radius", radius);
    ROS_INFO_STREAM("specified radius: " << radius << "m");
  } else {
    ROS_WARN_STREAM("cylinder radius not specified, using default of " << radius << "m");
  }
  minRadius = radius * (1.f - tolerance);
  maxRadius = radius * (1.f + tolerance);
  ROS_INFO_STREAM("max radius: " << maxRadius << ", min radius: " << minRadius);

  // set up ROS topics
  tf_broadcaster = new tf::TransformBroadcaster();
  cloud_sub_ = n.subscribe(cam_image_topic, 1, &findCylinder);

  //start actively listening
  ros::Rate rate(30);

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}