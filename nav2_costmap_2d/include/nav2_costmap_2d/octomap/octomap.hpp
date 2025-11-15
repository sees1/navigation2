#ifndef NAV2_COSTMAP_2D__OCTOMAP_HPP_
#define NAV2_COSTMAP_2D__OCTOMAP_HPP_

#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeKey.h>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <map>
#include <unordered_set>
#include <string>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "Eigen/Core"

namespace nav2_costmap_2d
{

// forward declaraton for "friend"
class Typed3DOcTree;

class Typed3DOcTreeNode : public octomap::OcTreeNode
{
public:
	enum OccupancyType {kTypeUnknown=-1, kTypeEmpty=0, kTypeGround=1, kTypeObstacle=100};

public:
	friend class Typed3DOcTree; // needs access to node children (inherited)

	Typed3DOcTreeNode()
  : OcTreeNode(),
    node_ref_id_(0),
    type_(kTypeUnknown)
  { }

	Typed3DOcTreeNode(const Typed3DOcTreeNode& rhs)
  : OcTreeNode(rhs),
    node_ref_id_(rhs.node_ref_id_),
    type_(rhs.type_)
  { }

	void setNodeRefId(int node_ref_id)               {node_ref_id_ = node_ref_id;}
	void setOccupancyType(char type)                 {type_ = type;}
	void setPointRef(const octomap::point3d & point) {point_ref_ = point;}
	int  getNodeRefId() const                        {return node_ref_id_;}
	int  getOccupancyType() const                    {return type_;}
	const octomap::point3d & getPointRef() const     {return point_ref_;}

	// following methods defined for octomap < 1.8 compatibility
	Typed3DOcTreeNode* getChild(unsigned int i);
	const Typed3DOcTreeNode* getChild(unsigned int i) const;
	bool pruneNode();
	void expandNode();
	bool createChild(unsigned int i);

	void updateOccupancyTypeChildren();

private:
	int node_ref_id_;
	int type_; // -1=undefined, 0=empty, 100=obstacle, 1=ground
	octomap::point3d point_ref_;
};

// Same as official ColorOctree but using Typed3DOcTreeNode, which is inheriting OcTreeNode
class Typed3DOcTree : public octomap::OccupancyOcTreeBase <Typed3DOcTreeNode>
{
public:
  // Default constructor, sets resolution of leafs
  Typed3DOcTree(double resolution);
  virtual ~Typed3DOcTree() { }

  // virtual constructor: creates a new object of same type
  // (Covariant return type requires an up-to-date compiler)
  Typed3DOcTree* create() const { return new Typed3DOcTree(resolution); }

  std::string getTreeType() const {return "Typed3DOcTree";} // same type as ColorOcTree to be compatible with ROS OctoMap msg

  /**
  * Prunes a node when it is collapsible. This overloaded
  * version only considers the node occupancy for pruning,
  * different colors of child nodes are ignored.
  * @return true if pruning was successful
  */
  virtual bool pruneNode(Typed3DOcTreeNode* node);

  virtual bool isNodeCollapsible(const Typed3DOcTreeNode* node) const;

  // update inner nodes, sets color to average child color
  void updateInnerOccupancy();

protected:
  void updateInnerOccupancyRecurs(Typed3DOcTreeNode* node, unsigned int depth);

  /**
   * Static member object which ensures that this OcTree's prototype
   * ends up in the classIDMapping only once. You need this as a
   * static member in any derived octree class in order to read .ot
   * files through the AbstractOcTree factory. You should also call
   * ensureLinking() once from the constructor.
   */
  class StaticMemberInitializer{
      public:
        StaticMemberInitializer();

        /**
       * Dummy function to ensure that MSVC does not drop the
       * StaticMemberInitializer, causing this tree failing to register.
       * Needs to be called from the constructor of this octree.
       */
        void ensureLinking() {};
  };

  /// static member to ensure static initialization (only once)
  static StaticMemberInitializer typed_3d_oc_tree_member_init_;
};

class OctoMap
{
public:
  OctoMap(float cell_size = 0.1, 
          float range_max = 5.0,
          unsigned int empty_flood_fill_depth = 0,
          float occupancy_thr = 0.5,
          float log_odds_hit = 0.7,
          float log_odds_miss = 0.4,
          float log_odds_clamping_min = 0.1192,
          float log_odds_clamping_max = 0.971);
          
  virtual ~OctoMap();

	const Typed3DOcTree * octree() const { return octree_; }

	pcl::PointCloud<pcl::PointXYZ>::Ptr createCloud(unsigned int tree_depth = 0,
                                                  std::vector<int> * obstacle_indices = 0,
                                                  std::vector<int> * empty_indices    = 0,
                                                  std::vector<int> * ground_indices   = 0,
                                                  bool original_ref_points = true,
                                                  std::vector<int> * frontier_indices = 0,
                                                  std::vector<double> * cloud_prob    = 0) const;

  void update(const geometry_msgs::msg::TransformStamped& view_point,
              sensor_msgs::msg::PointCloud2::SharedPtr ground,
              sensor_msgs::msg::PointCloud2::SharedPtr obstacles,
              sensor_msgs::msg::PointCloud2::SharedPtr empty);

  void update(const geometry_msgs::msg::TransformStamped& view_point,
              const sensor_msgs::msg::PointCloud2& ground,
              const sensor_msgs::msg::PointCloud2& obstacles,
              const sensor_msgs::msg::PointCloud2& empty);

	bool writeBinary(const std::string & path);

	virtual void clear();

public:
  // static member's
  static std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> findEmptyNode(Typed3DOcTree* octree_,
                                                                                           unsigned int tree_depth,
                                                                                           octomap::point3d start_position);

  static void floodFill(Typed3DOcTree* octree_,
                        unsigned int tree_depth,
                        octomap::point3d start_position,
                        std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash>& empty_nodes,
                        std::queue<octomap::point3d>& position_to_explore);

  static bool isNodeVisited(std::unordered_set<octomap::OcTreeKey,octomap::OcTreeKey::KeyHash> const& empty_nodes,
                            octomap::OcTreeKey const key);

  static octomap::point3d findCloseEmpty(Typed3DOcTree* octree_,
                                         unsigned int tree_depth,
                                         octomap::point3d start_position);

  static bool isValidEmpty(Typed3DOcTree* octree_,
                           unsigned int tree_depth,
                           octomap::point3d start_position);

protected:
	virtual void assemble(const geometry_msgs::msg::TransformStamped& view_point);

private:
	void updateMinMax(const octomap::point3d & point);

private:
	Typed3DOcTree * octree_;

  pcl::PointCloud<pcl::PointXYZ> ground_;
  pcl::PointCloud<pcl::PointXYZ> obstacles_;
  pcl::PointCloud<pcl::PointXYZ> empty_;

	float range_max_;
  unsigned int empty_flood_fill_depth_;

	float cell_size_;

	float occupancy_thr_;
	float log_odds_hit_;
	float log_odds_miss_;
	float log_odds_clamping_min_;
	float log_odds_clamping_max_;

	double min_values_[3];
	double max_values_[3];
};

}  // end namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__OCTOMAP_HPP_
