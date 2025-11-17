
#ifndef NAV2_COSTMAP_2D__NEG_OBSTACLE_LAYER_HPP_
#define NAV2_COSTMAP_2D__NEG_OBSTACLE_LAYER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "laser_geometry/laser_geometry.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreorder"
#include "tf2_ros/message_filter.h"
#pragma GCC diagnostic pop
#include "message_filters/subscriber.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_costmap_2d/footprint.hpp"

namespace nav2_costmap_2d
{

  class NegativeObstacleLayer : public CostmapLayer
  {
  public:
    NegativeObstacleLayer() { costmap_ = NULL; }

    virtual ~NegativeObstacleLayer();
    
    virtual void onInitialize();

    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                              double * min_x, double * min_y,
                              double * max_x, double * max_y);
    
    virtual void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                            int min_i,
                            int min_j,
                            int max_i,
                            int max_j);

    virtual void deactivate();

    virtual void activate();

    virtual void reset();

    virtual bool isClearable() {return true;}

    rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

    void resetBuffersLastUpdated();

    void pointCloud2Callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
                            const std::shared_ptr<nav2_costmap_2d::ObservationBufferBase> & buffer);

    // for testing purposes
    void addStaticObservation(nav2_costmap_2d::Observation & obs, bool marking, bool clearing);
    void clearStaticObservations(bool marking, bool clearing);

  protected:
    bool getMarkingObservations(std::vector<nav2_costmap_2d::Observation> & marking_observations) const;

    void updateFootprint(double robot_x, double robot_y, double robot_yaw, double * min_x,
                        double * min_y, double * max_x, double * max_y);

  protected:
    std::vector<geometry_msgs::msg::Point> transformed_footprint_;
    bool footprint_clearing_enabled_;

    std::string global_frame_;

    std::vector<std::shared_ptr<message_filters::SubscriberBase<rclcpp_lifecycle::LifecycleNode>>> observation_subscribers_;
    std::vector<std::shared_ptr<tf2_ros::MessageFilterBase>> observation_notifiers_;
    std::vector<std::shared_ptr<nav2_costmap_2d::ObservationBufferBase>> marking_buffers_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr empty_cloud_pub_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

    // Used only for testing purposes
    std::vector<nav2_costmap_2d::Observation> static_clearing_observations_;
    std::vector<nav2_costmap_2d::Observation> static_marking_observations_;

    bool rolling_window_;
    bool was_reset_;
    int combination_method_;
  };

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__NEG_OBSTACLE_LAYER_HPP_
