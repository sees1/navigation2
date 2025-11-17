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
#ifndef NAV2_COSTMAP_2D__OBSERVATION_BUFFER_HPP_
#define NAV2_COSTMAP_2D__OBSERVATION_BUFFER_HPP_

#include <vector>
#include <list>
#include <string>
#include <thread>
#include <chrono>
#include <mutex>
#include <future>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav2_costmap_2d/observation.hpp"
#include "nav2_costmap_2d/octomap/octomap.hpp"
#include <pcl/filters/extract_indices.h>
#include "nav2_util/lifecycle_node.hpp"


namespace nav2_costmap_2d
{

  class ObservationBufferBase
  {
  public:
    ObservationBufferBase(
      const nav2_util::LifecycleNode::WeakPtr & parent,
      std::string topic_name,
      double expected_update_rate,
      double obstacle_max_range, double obstacle_min_range,
      tf2_ros::Buffer & tf2_buffer,
      tf2::Duration tf_tolerance,
      std::string global_frame = "map");

    virtual ~ObservationBufferBase() { };

    virtual void bufferCloud(const sensor_msgs::msg::PointCloud2 & cloud) = 0;

    virtual void getObservations(std::vector<Observation> & observations) = 0;

    void resetLastUpdated();

    bool isCurrent() const;

    inline void lock()
    {
      lock_.lock();
    }

    inline void unlock()
    {
      lock_.unlock();
    }

  protected:
    virtual void purgeStaleObservations() = 0;

  protected:
    rclcpp::Clock::SharedPtr clock_;
    const rclcpp::Duration expected_update_rate_;
    rclcpp::Time last_updated_;

    rclcpp::Logger logger_{rclcpp::get_logger("nav2_costmap_2d")};

    tf2_ros::Buffer & tf2_buffer_;
    tf2::Duration tf_tolerance_;

    std::string global_frame_;

    std::string topic_name_;

    double obstacle_max_range_;
    double obstacle_min_range_;

    std::recursive_mutex lock_;  ///< @brief A lock for accessing data in callbacks safely
  };

  class ObservationBuffer : public ObservationBufferBase
  {
  public:
    ObservationBuffer(
      const nav2_util::LifecycleNode::WeakPtr & parent,
      std::string topic_name,
      double observation_keep_time,
      double expected_update_rate,
      double min_obstacle_height, double max_obstacle_height,
      double obstacle_max_range, double obstacle_min_range,
      double raytrace_max_range, double raytrace_min_range,
      tf2_ros::Buffer & tf2_buffer,
      std::string global_frame,
      tf2::Duration tf_tolerance,
      std::string sensor_frame = "");

    ~ObservationBuffer() { }

    void bufferCloud(const sensor_msgs::msg::PointCloud2 & cloud) override;

    void getObservations(std::vector<Observation> & observations) override;

  protected:
    void purgeStaleObservations() override;

  protected:
    const rclcpp::Duration observation_keep_time_;
    std::list<Observation> observation_list_;
    std::string sensor_frame_;
    double min_obstacle_height_;
    double max_obstacle_height_;
    double raytrace_max_range_;
    double raytrace_min_range_;
  };

  class NegativeObservationBuffer : public ObservationBufferBase
  {
  public:
    NegativeObservationBuffer(
      const nav2_util::LifecycleNode::WeakPtr & parent,
      std::string topic_name,
      double expected_update_rate,
      double obstacle_max_range,
      double obstacle_min_range,
      tf2_ros::Buffer & tf2_buffer,
      std::string global_frame,
      tf2::Duration tf_tolerance,
      double resolution = 0.1);

    ~NegativeObservationBuffer() { }

    void bufferCloud(const sensor_msgs::msg::PointCloud2 & cloud) override;

    void getObservations(std::vector<Observation> & observations) override;

  protected:
    void purgeStaleObservations() override;

  protected:
    double resolution_;
    OctoMap octomap_;
  };

}  // namespace nav2_costmap_2d
#endif  // NAV2_COSTMAP_2D__OBSERVATION_BUFFER_HPP_
