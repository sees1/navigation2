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
 *         David V. Lu!!
 *         Steve Macenski
 *********************************************************************/
#include "nav2_costmap_2d/negative_obstacle_layer.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::NegativeObstacleLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

using nav2_costmap_2d::ObservationBuffer;
using nav2_costmap_2d::Observation;
using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{

NegativeObstacleLayer::~NegativeObstacleLayer()
{
  dyn_params_handler_.reset();
  for (auto & notifier : observation_notifiers_) {
    notifier.reset();
  }
}

void NegativeObstacleLayer::onInitialize()
{
  bool track_unknown_space;
  double transform_tolerance;

  // The topics that we'll subscribe to from the parameter server
  std::string topics_string;

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));
  // declareParameter("min_obstacle_height", rclcpp::ParameterValue(0.0));
  // declareParameter("max_obstacle_height", rclcpp::ParameterValue(2.0));
  declareParameter("combination_method", rclcpp::ParameterValue(1));
  declareParameter("observation_sources", rclcpp::ParameterValue(std::string("")));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "footprint_clearing_enabled", footprint_clearing_enabled_);
  node->get_parameter(name_ + "." + "min_obstacle_height", min_obstacle_height_);
  node->get_parameter(name_ + "." + "max_obstacle_height", max_obstacle_height_);
  node->get_parameter(name_ + "." + "combination_method", combination_method_);
  node->get_parameter("track_unknown_space", track_unknown_space);
  node->get_parameter("transform_tolerance", transform_tolerance);
  node->get_parameter(name_ + "." + "observation_sources", topics_string);

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&NegativeObstacleLayer::dynamicParametersCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Subscribed to Topics: %s", topics_string.c_str());

  rolling_window_ = layered_costmap_->isRolling();

  if (track_unknown_space) {
    default_value_ = NO_INFORMATION;
  } else {
    default_value_ = FREE_SPACE;
  }

  NegativeObstacleLayer::matchSize();

  current_ = true;
  was_reset_ = false;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;

  // now we need to split the topics based on whitespace which we can use a stringstream for
  std::stringstream ss(topics_string);

  std::string source;
  while (ss >> source) {

    // get the parameters for the specific topic
    double observation_keep_time;
    double expected_update_rate;
    double min_obstacle_height;
    double max_obstacle_height;
    std::string topic;
    // std::string data_type;
    bool inf_is_valid;

    declareParameter(source + "." + "topic", rclcpp::ParameterValue(source));
    // declareParameter(source + "." + "sensor_frame", rclcpp::ParameterValue(std::string("")));
    declareParameter(source + "." + "observation_persistence", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "expected_update_rate", rclcpp::ParameterValue(0.0));
    // declareParameter(source + "." + "data_type", rclcpp::ParameterValue(std::string("LaserScan")));
    declareParameter(source + "." + "min_obstacle_height", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "max_obstacle_height", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "inf_is_valid", rclcpp::ParameterValue(false));
    // declareParameter(source + "." + "marking", rclcpp::ParameterValue(true));
    // declareParameter(source + "." + "clearing", rclcpp::ParameterValue(false));
    declareParameter(source + "." + "obstacle_max_range", rclcpp::ParameterValue(2.5));
    declareParameter(source + "." + "obstacle_min_range", rclcpp::ParameterValue(0.0));
    // declareParameter(source + "." + "raytrace_max_range", rclcpp::ParameterValue(3.0));
    // declareParameter(source + "." + "raytrace_min_range", rclcpp::ParameterValue(0.0));

    node->get_parameter(name_ + "." + source + "." + "topic", topic);
    // node->get_parameter(name_ + "." + source + "." + "sensor_frame", sensor_frame);
    node->get_parameter(name_ + "." + source + "." + "observation_persistence", observation_keep_time);
    node->get_parameter(name_ + "." + source + "." + "expected_update_rate",expected_update_rate);
    // node->get_parameter(name_ + "." + source + "." + "data_type", data_type);
    node->get_parameter(name_ + "." + source + "." + "min_obstacle_height", min_obstacle_height);
    node->get_parameter(name_ + "." + source + "." + "max_obstacle_height", max_obstacle_height);
    node->get_parameter(name_ + "." + source + "." + "inf_is_valid", inf_is_valid);
    // node->get_parameter(name_ + "." + source + "." + "marking", marking);
    // node->get_parameter(name_ + "." + source + "." + "clearing", clearing);


    // get the obstacle range for the sensor
    double obstacle_max_range, obstacle_min_range;
    node->get_parameter(name_ + "." + source + "." + "obstacle_max_range", obstacle_max_range);
    node->get_parameter(name_ + "." + source + "." + "obstacle_min_range", obstacle_min_range);

    // create an observation buffer
    observation_buffers_.push_back(std::shared_ptr<ObservationBuffer>(new ObservationBuffer(node,
                                                                                            topic,
                                                                                            observation_keep_time,
                                                                                            expected_update_rate,
                                                                                            min_obstacle_height,
                                                                                            max_obstacle_height,
                                                                                            obstacle_max_range,
                                                                                            obstacle_min_range,
                                                                                            *tf_,
                                                                                            global_frame_,
                                                                                            tf2::durationFromSec(transform_tolerance),
                                                                                            true,
                                                                                            resolution_)));

    marking_buffers_.push_back(observation_buffers_.back());

    RCLCPP_DEBUG(logger_,
                 "Created an observation buffer for source %s, topic %s, global frame: %s, "
                 "expected update rate: %.2f, observation persistence: %.2f",
                 source.c_str(), topic.c_str(), global_frame_.c_str(), expected_update_rate, observation_keep_time);

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = 50;

    // create a callback for the topic
    // if (data_type == "LaserScan") {
    //   auto sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
    //       rclcpp_lifecycle::LifecycleNode>>(node, topic, custom_qos_profile, sub_opt);
    //   sub->unsubscribe();

    //   auto filter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
    //     *sub, *tf_, global_frame_, 50,
    //     node->get_node_logging_interface(),
    //     node->get_node_clock_interface(),
    //     tf2::durationFromSec(transform_tolerance));

    //   if (inf_is_valid) {
    //     filter->registerCallback(
    //       std::bind(
    //         &ObstacleLayer::laserScanValidInfCallback, this, std::placeholders::_1,
    //         observation_buffers_.back()));

    //   } else {
    //     filter->registerCallback(
    //       std::bind(
    //         &ObstacleLayer::laserScanCallback, this, std::placeholders::_1,
    //         observation_buffers_.back()));
    //   }

    //   observation_subscribers_.push_back(sub);

    //   observation_notifiers_.push_back(filter);
    //   observation_notifiers_.back()->setTolerance(rclcpp::Duration::from_seconds(0.05));

    // } else {

    auto sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2,
                                                            rclcpp_lifecycle::LifecycleNode>>(node, topic, custom_qos_profile, sub_opt);
    sub->unsubscribe();

    if (inf_is_valid)
      RCLCPP_WARN(logger_, "obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");

    auto filter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(*sub,
                                                                                          *tf_,
                                                                                          global_frame_,
                                                                                          50,
                                                                                          node->get_node_logging_interface(),
                                                                                          node->get_node_clock_interface(),
                                                                                          tf2::durationFromSec(transform_tolerance));

    filter->registerCallback(std::bind(&NegativeObstacleLayer::pointCloud2Callback, this, std::placeholders::_1, observation_buffers_.back()));

    observation_subscribers_.push_back(sub);
    observation_notifiers_.push_back(filter);
  }
}

rcl_interfaces::msg::SetParametersResult
NegativeObstacleLayer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    // if (param_type == ParameterType::PARAMETER_DOUBLE) {
    //   if (param_name == name_ + "." + "min_obstacle_height") {
    //     min_obstacle_height_ = parameter.as_double();
    //   } else if (param_name == name_ + "." + "max_obstacle_height") {
    //     max_obstacle_height_ = parameter.as_double();
    //   }
    // } else 
    if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + "." + "enabled") {
        enabled_ = parameter.as_bool();
      } else if (param_name == name_ + "." + "footprint_clearing_enabled") {
        footprint_clearing_enabled_ = parameter.as_bool();
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == name_ + "." + "combination_method") {
        combination_method_ = parameter.as_int();
      }
    }
  }

  result.successful = true;
  return result;
}

void NegativeObstacleLayer::pointCloud2Callback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
  const std::shared_ptr<ObservationBuffer> & buffer)
{
  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(*message);
  buffer->unlock();
}

void NegativeObstacleLayer::updateBounds(double robot_x,
                                         double robot_y,
                                         double robot_yaw,
                                         double * min_x,
                                         double * min_y,
                                         double * max_x,
                                         double * max_y)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (rolling_window_)
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  
  if (!enabled_)
    return;

  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<Observation> observations;

  // update the global current status
  current_ = current && getMarkingObservations(observations);

  // place the new obstacles into a priority queue... each with a priority of zero to begin with
  for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
  {
    const Observation & obs = *it;

    const sensor_msgs::msg::PointCloud2 & cloud = *(obs.cloud_);

    double sq_obstacle_max_range = obs.obstacle_max_range_ * obs.obstacle_max_range_;
    double sq_obstacle_min_range = obs.obstacle_min_range_ * obs.obstacle_min_range_;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      double px = *iter_x, py = *iter_y, pz = *iter_z;

      // if the obstacle is too low, we won't add it
      if (pz < min_obstacle_height_) {
        RCLCPP_DEBUG(logger_, "The point is too low");
        continue;
      }

      // if the obstacle is too high or too far away from the robot we won't add it
      if (pz > max_obstacle_height_) {
        RCLCPP_DEBUG(logger_, "The point is too high");
        continue;
      }

      // compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist =
        (px -
        obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y) +
        (pz - obs.origin_.z) * (pz - obs.origin_.z);

      // if the point is far enough away... we won't consider it
      if (sq_dist >= sq_obstacle_max_range) {
        RCLCPP_DEBUG(logger_, "The point is too far away");
        continue;
      }

      // if the point is too close, do not conisder it
      if (sq_dist < sq_obstacle_min_range) {
        RCLCPP_DEBUG(logger_, "The point is too close");
        continue;
      }

      // now we need to compute the map coordinates for the observation
      unsigned int mx, my;
      if (!worldToMap(px, py, mx, my)) {
        RCLCPP_DEBUG(logger_, "Computing map coords failed");
        continue;
      }

      unsigned int index = getIndex(mx, my);
      costmap_[index] = LETHAL_OBSTACLE;
      touch(px, py, min_x, min_y, max_x, max_y);
    }
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void
NegativeObstacleLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x,
  double * max_y)
{
  if (!footprint_clearing_enabled_) {return;}
  transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

  for (unsigned int i = 0; i < transformed_footprint_.size(); i++) {
    touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}

void
NegativeObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }

  // if not current due to reset, set current now after clearing
  if (!current_ && was_reset_) {
    was_reset_ = false;
    current_ = true;
  }

  if (footprint_clearing_enabled_) {
    setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
  }

  switch (combination_method_) {
    case 0:  // Overwrite
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }
}

void
NegativeObstacleLayer::addStaticObservation(
  nav2_costmap_2d::Observation & obs,
  bool marking, bool clearing)
{
  if (marking) {
    static_marking_observations_.push_back(obs);
  }
  if (clearing) {
    static_clearing_observations_.push_back(obs);
  }
}

void
NegativeObstacleLayer::clearStaticObservations(bool marking, bool clearing)
{
  if (marking) {
    static_marking_observations_.clear();
  }
  if (clearing) {
    static_clearing_observations_.clear();
  }
}

bool
NegativeObstacleLayer::getMarkingObservations(std::vector<Observation> & marking_observations) const
{
  bool current = true;
  // get the marking observations
  for (unsigned int i = 0; i < marking_buffers_.size(); ++i) {
    marking_buffers_[i]->lock();
    marking_buffers_[i]->getObservations(marking_observations);
    current = marking_buffers_[i]->isCurrent() && current;
    marking_buffers_[i]->unlock();
  }
  marking_observations.insert(
    marking_observations.end(),
    static_marking_observations_.begin(), static_marking_observations_.end());
  return current;
}

void NegativeObstacleLayer::activate()
{
  for (auto & notifier : observation_notifiers_)
    notifier->clear();

  // if we're stopped we need to re-subscribe to topics
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->subscribe();
  
  resetBuffersLastUpdated();
}

void NegativeObstacleLayer::deactivate()
{
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->unsubscribe();
}

void NegativeObstacleLayer::reset()
{
  resetMaps();
  resetBuffersLastUpdated();
  current_ = false;
  was_reset_ = true;
}

void NegativeObstacleLayer::resetBuffersLastUpdated()
{
  for (unsigned int i = 0; i < observation_buffers_.size(); ++i)
    if (observation_buffers_[i])
      observation_buffers_[i]->resetLastUpdated();
}

}  // namespace nav2_costmap_2d
