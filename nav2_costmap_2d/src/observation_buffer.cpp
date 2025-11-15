/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_costmap_2d/utils/util3d.hpp"

#include <algorithm>
#include <list>
#include <string>
#include <vector>
#include <chrono>

#include "tf2/convert.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
using namespace std::chrono_literals;

namespace nav2_costmap_2d
{
ObservationBuffer::ObservationBuffer(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  std::string topic_name,
  double observation_keep_time,
  double expected_update_rate,
  double min_obstacle_height, double max_obstacle_height,
  double obstacle_max_range, double obstacle_min_range,
  tf2_ros::Buffer & tf2_buffer,
  std::string global_frame,
  tf2::Duration tf_tolerance,
  bool ray_tracing,
  double resolution_,
  std::string sensor_frame,
  double raytrace_max_range, double raytrace_min_range)
: tf2_buffer_(tf2_buffer),
  observation_keep_time_(rclcpp::Duration::from_seconds(observation_keep_time)),
  expected_update_rate_(rclcpp::Duration::from_seconds(expected_update_rate)),
  global_frame_(global_frame),
  sensor_frame_(sensor_frame),
  topic_name_(topic_name),
  min_obstacle_height_(min_obstacle_height), max_obstacle_height_(max_obstacle_height),
  obstacle_max_range_(obstacle_max_range), obstacle_min_range_(obstacle_min_range),
  raytrace_max_range_(raytrace_max_range), raytrace_min_range_(raytrace_min_range), 
  tf_tolerance_(tf_tolerance),
  ray_tracing_(ray_tracing),
  octomap_(resolution_, obstacle_min_range_)
{
  auto node = parent.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();
  last_updated_ = node->now();
}

ObservationBuffer::~ObservationBuffer()
{
}

void ObservationBuffer::bufferCloud(const sensor_msgs::msg::PointCloud2 & cloud)
{
  geometry_msgs::msg::PointStamped global_origin;

  // create a new observation on the list to be populated
  observation_list_.push_front(Observation());

  // check whether the origin frame has been set explicitly
  // or whether we should get it from the cloud
  std::string origin_frame = sensor_frame_ == "" ? cloud.header.frame_id : sensor_frame_;

  if (ray_tracing_)
  {
    sensor_msgs::msg::PointCloud2 ros_ground_cloud;
    sensor_msgs::msg::PointCloud2 ros_empty_cloud;

    // stay at original cloud frame
    geometry_msgs::msg::TransformStamped vp;
    vp.header.frame_id = origin_frame;
    vp.child_frame_id = origin_frame;
    vp.transform.translation.x = 0.0f;
    vp.transform.translation.y = 0.0f;
    vp.transform.translation.z = 0.0f;
    vp.transform.rotation.x = 0.0f;
    vp.transform.rotation.y = 0.0f;
    vp.transform.rotation.z = 0.0f;
    vp.transform.rotation.w = 1.0f;

    octomap_.update(vp, ros_ground_cloud, cloud, ros_empty_cloud);

    pcl::IndicesPtr groundIndices(new std::vector<int>);
    pcl::IndicesPtr obstaclesIndices(new std::vector<int>);
    pcl::IndicesPtr emptyIndices(new std::vector<int>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudWithRayTracing = octomap_.createCloud(0, obstaclesIndices.get(), emptyIndices.get(), groundIndices.get());

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloudWithRayTracing);
    extract.setIndices(emptyIndices);
    extract.setNegative(false); // false = оставить только выбранные
    pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*empty_cloud);

    pcl::toROSMsg(*empty_cloud, *(observation_list_.front().cloud_));
  }
  else
  {
    try {
      // given these observations come from sensors...
      // we'll need to store the origin pt of the sensor
      geometry_msgs::msg::PointStamped local_origin;
      local_origin.header.stamp = cloud.header.stamp;
      local_origin.header.frame_id = origin_frame;
      local_origin.point.x = 0;
      local_origin.point.y = 0;
      local_origin.point.z = 0;
      tf2_buffer_.transform(local_origin, global_origin, global_frame_, tf_tolerance_);
      tf2::convert(global_origin.point, observation_list_.front().origin_);

      // make sure to pass on the raytrace/obstacle range
      // of the observation buffer to the observations
      observation_list_.front().raytrace_max_range_ = raytrace_max_range_;
      observation_list_.front().raytrace_min_range_ = raytrace_min_range_;
      observation_list_.front().obstacle_max_range_ = obstacle_max_range_;
      observation_list_.front().obstacle_min_range_ = obstacle_min_range_;

      sensor_msgs::msg::PointCloud2 global_frame_cloud;

      // transform the point cloud
      tf2_buffer_.transform(cloud, global_frame_cloud, global_frame_, tf_tolerance_);
      global_frame_cloud.header.stamp = cloud.header.stamp;

      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_global_frame_cloud;
      pcl::fromROSMsg(global_frame_cloud, *pcl_global_frame_cloud);

      pcl_global_frame_cloud = passThrough<pcl::PointXYZ>(pcl_global_frame_cloud, "z", min_obstacle_height_, max_obstacle_height_);
      pcl::toROSMsg(*pcl_global_frame_cloud, *(observation_list_.front().cloud_));

      // now we need to remove observations from the cloud that are below
      // or above our height thresholds
      sensor_msgs::msg::PointCloud2 & observation_cloud = *(observation_list_.front().cloud_);
      observation_cloud.header.frame_id = global_frame_cloud.header.frame_id;
    } catch (tf2::TransformException & ex) {
      // if an exception occurs, we need to remove the empty observation from the list
      observation_list_.pop_front();
      RCLCPP_ERROR(
        logger_,
        "TF Exception that should never happen for sensor frame: %s, cloud frame: %s, %s",
        sensor_frame_.c_str(),
        cloud.header.frame_id.c_str(), ex.what());
      return;
    }
  } 

  // if the update was successful, we want to update the last updated time
  last_updated_ = clock_->now();

  // we'll also remove any stale observations from the list
  purgeStaleObservations();
}

// returns a copy of the observations
void ObservationBuffer::getObservations(std::vector<Observation> & observations)
{
  // first... let's make sure that we don't have any stale observations
  purgeStaleObservations();

  // now we'll just copy the observations for the caller
  std::list<Observation>::iterator obs_it;
  for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it) {
    observations.push_back(*obs_it);
  }
}

void ObservationBuffer::purgeStaleObservations()
{
  if (!observation_list_.empty()) {
    std::list<Observation>::iterator obs_it = observation_list_.begin();
    // if we're keeping observations for no time... then we'll only keep one observation
    if (observation_keep_time_ == rclcpp::Duration(0.0s)) {
      observation_list_.erase(++obs_it, observation_list_.end());
      return;
    }

    // otherwise... we'll have to loop through the observations to see which ones are stale
    for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it) {
      Observation & obs = *obs_it;
      // check if the observation is out of date... and if it is,
      // remove it and those that follow from the list
      if ((clock_->now() - obs.cloud_->header.stamp) >
        observation_keep_time_)
      {
        observation_list_.erase(obs_it, observation_list_.end());
        return;
      }
    }
  }
}

bool ObservationBuffer::isCurrent() const
{
  if (expected_update_rate_ == rclcpp::Duration(0.0s)) {
    return true;
  }

  bool current = (clock_->now() - last_updated_) <=
    expected_update_rate_;
  // if (!current) {
  //   RCLCPP_WARN(
  //     logger_,
  //     "The %s observation buffer has not been updated for %.2f seconds, "
  //     "and it should be updated every %.2f seconds.",
  //     topic_name_.c_str(),
  //     (clock_->now() - last_updated_).seconds(),
  //     expected_update_rate_.seconds());
  // }
  return current;
}

void ObservationBuffer::resetLastUpdated()
{
  last_updated_ = clock_->now();
}
}  // namespace nav2_costmap_2d
