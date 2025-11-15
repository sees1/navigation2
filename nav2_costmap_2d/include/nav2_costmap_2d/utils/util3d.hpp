#ifndef NAV2_COSTMAP_2D__UTIL3D_HPP_
#define NAV2_COSTMAP_2D__UTIL3D_HPP_

#include <vector>
#include <string>
#include <cassert>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>

#include <Eigen/Core>

namespace nav2_costmap_2d
{
  template <typename PointT>
  pcl::IndicesPtr passThrough(const typename pcl::PointCloud<PointT>::Ptr & cloud,
                              const pcl::IndicesPtr & indices,
                              const std::string & axis,
                              float min,
                              float max,
                              bool negative = false);
  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr passThrough(const typename pcl::PointCloud<PointT>::Ptr & cloud,
                                                    const std::string & axis,
                                                    float min,
                                                    float max,
                                                    bool negative = false);
  template<typename PointT>
  pcl::IndicesPtr cropBox(const typename pcl::PointCloud<PointT>::Ptr & cloud,
                          const pcl::IndicesPtr & indices,
                          const Eigen::Vector4f & min,
                          const Eigen::Vector4f & max,
                          const Eigen::Affine3f & transform,
                          bool negative = false);
  template<typename PointT>
  typename pcl::PointCloud<PointT>::Ptr cropBox(const typename pcl::PointCloud<PointT>::Ptr & cloud,
                                                const Eigen::Vector4f & min,
                                                const Eigen::Vector4f & max,
                                                const Eigen::Affine3f & transform,
                                                bool negative = false);
} // namespace nav2_costmap_2d

#include "nav2_costmap_2d/utils/impl/util3d_impl.hpp"

#endif  // NAV2_COSTMAP_2D__UTIL3D_HPP_