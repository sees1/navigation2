#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

#include <pcl/features/normal_3d_omp.h>

#include <pcl/search/kdtree.h>

#include <pcl/common/common.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace nav2_costmap_2d
{
  template <typename PointT>
  pcl::IndicesPtr passThrough(const typename pcl::PointCloud<PointT>::Ptr & cloud,
                              const pcl::IndicesPtr & indices,
                              const std::string & axis,
                              float min,
                              float max,
                              bool negative)
  {
    std::cout << "cloud = " << cloud->size() << ", max = " << max << " min = " << min << " axis = " << axis << std::endl;
    assert(axis.compare("x") == 0 || axis.compare("y") == 0 || axis.compare("z") == 0);

    pcl::IndicesPtr output(new std::vector<int>);
    pcl::PassThrough<PointT> filter;
    filter.setNegative(negative);
    filter.setFilterFieldName(axis);
    filter.setFilterLimits(min, max);
    filter.setInputCloud(cloud);
    filter.setIndices(indices);
    filter.filter(*output);
    return output;
  }

  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr passThrough(const typename pcl::PointCloud<PointT>::Ptr & cloud,
                                                    const std::string & axis,
                                                    float min,
                                                    float max,
                                                    bool negative)
  {
    std::cout << "cloud = " << cloud->size() << ", max = " << max << " min = " << min << " axis = " << axis << std::endl;
    assert(axis.compare("x") == 0 || axis.compare("y") == 0 || axis.compare("z") == 0);

    typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
    pcl::PassThrough<PointT> filter;
    filter.setNegative(negative);
    filter.setFilterFieldName(axis);
    filter.setFilterLimits(min, max);
    filter.setInputCloud(cloud);
    filter.filter(*output);
    return output;
  }

  template<typename PointT>
  pcl::IndicesPtr cropBox(const typename pcl::PointCloud<PointT>::Ptr & cloud,
                          const pcl::IndicesPtr & indices,
                          const Eigen::Vector4f & min,
                          const Eigen::Vector4f & max,
                          const Eigen::Affine3f & transform,
                          bool negative)
  {
    assert(min[0] < max[0] && min[1] < max[1] && min[2] < max[2]);

    pcl::IndicesPtr output(new std::vector<int>);
    pcl::CropBox<PointT> filter;
    filter.setNegative(negative);
    filter.setMin(min);
    filter.setMax(max);
    filter.setTransform(transform);
    filter.setInputCloud(cloud);
    filter.setIndices(indices);
    filter.filter(*output);
    return output;
  }
  
  template<typename PointT>
  typename pcl::PointCloud<PointT>::Ptr cropBox(const typename pcl::PointCloud<PointT>::Ptr & cloud,
                                                const Eigen::Vector4f & min,
                                                const Eigen::Vector4f & max,
                                                const Eigen::Affine3f & transform,
                                                bool negative)
  {
    assert(min[0] < max[0] && min[1] < max[1] && min[2] < max[2]);

    typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> filter;
    filter.setNegative(negative);
    filter.setMin(min);
    filter.setMax(max);
    filter.setTransform(transform);
    filter.setInputCloud(cloud);
    filter.filter(*output);
    return output;
  }

} // namespace nav2_costmap_2d