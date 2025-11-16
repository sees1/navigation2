#include "nav2_costmap_2d/octomap/octomap.hpp"

#include <string>
#include <vector>
#include <algorithm>
#include <memory>
#include <cassert>
#include <cstdio>
#include <chrono>
#include <cstdarg>

#define WARNING(fmt, ...) std::printf("\033[33m" fmt "\033[0m\n", ##__VA_ARGS__);   

namespace nav2_costmap_2d
{
  /*
    (Typed3DOcTreeNode) method's definition
  */

	// method defined for octomap < 1.8 compatibility
	Typed3DOcTreeNode* Typed3DOcTreeNode::getChild(unsigned int i)
  {
#ifdef OCTOMAP_PRE_18
    return static_cast<Typed3DOcTreeNode*> (OcTreeNode::getChild(i));
#else
    i++; // for happy compilator
    throw std::runtime_error("Function getChild(uint) should not be used with octomap >= 1.8");
#endif
  }

  // method defined for octomap < 1.8 compatibility
	const Typed3DOcTreeNode* Typed3DOcTreeNode::getChild(unsigned int i) const
  {
#ifdef OCTOMAP_PRE_18
    return static_cast<const Typed3DOcTreeNode*> (OcTreeNode::getChild(i));
#else
    i++; // for happy compilator
    throw std::runtime_error("Function getChild(uint) const should not be used with octomap >= 1.8");
#endif
  }

	// method defined for octomap < 1.8 compatibility
  bool Typed3DOcTreeNode::pruneNode()
  {
#ifdef OCTOMAP_PRE_18
    // checks for equal occupancy only, color ignored
    if (!this->collapsible()) return false;
    // set occupancy value
    setLogOdds(getChild(0)->getLogOdds());
    // delete children
    for (unsigned int i = 0; i < 8; i++)
      delete children[i];
    
    delete[] children;
    children = NULL;
    return true;
#else
    throw std::runtime_error("Function pruneNode() const should not be used with octomap >= 1.8");
#endif
  }

	// method defined for octomap < 1.8 compatibility
  void Typed3DOcTreeNode::expandNode()
  {
#ifdef OCTOMAP_PRE_18
    assert(!hasChildren());
    for (unsigned int k = 0; k < 8; k++) {
      createChild(k);
      children[k]->setValue(value);
    }
#else
    throw std::runtime_error("Function expandNode() const should not be used with octomap >= 1.8");
#endif
  }

	// method defined for octomap < 1.8 compatibility
  bool Typed3DOcTreeNode::createChild(unsigned int i)
  {
#ifdef OCTOMAP_PRE_18
    if (children == NULL) allocChildren();
    children[i] = new Typed3DOcTreeNode();
    return true;
#else
    i++; // for happy compilator
    throw std::runtime_error("Function createChild() const should not be used with octomap >= 1.8");
#endif
  }

	void Typed3DOcTreeNode::updateOccupancyTypeChildren()
  {
    if (children != NULL)
    {
      int type = kTypeUnknown;

      for (unsigned int i = 0; i < 8 && type != kTypeObstacle; i++)
      {
        Typed3DOcTreeNode* child = static_cast<Typed3DOcTreeNode*>(children[i]);

        if (child != NULL && child->getOccupancyType() >= kTypeEmpty)
          if(type == kTypeUnknown)
            type = child->getOccupancyType();
      }

      type_ = type;
    }
  }

  /*
    (Typed3DOcTree) method's definition
  */

  Typed3DOcTree::Typed3DOcTree(double resolution)
  : octomap::OccupancyOcTreeBase<Typed3DOcTreeNode>(resolution)
  {
    typed_3d_oc_tree_member_init_.ensureLinking();
  }

  // method defined for octomap < 1.8 compatibility
  bool Typed3DOcTree::pruneNode(Typed3DOcTreeNode* node)
  {
#ifndef OCTOMAP_PRE_18
    if (!isNodeCollapsible(node))
      return false;
  
    // set value to children's values (all assumed equal)
    node->copyData(*(getNodeChild(node, 0)));
  
    // delete children
    for (unsigned int i = 0; i < 8; i++)
      deleteNodeChild(node, i);
    
    delete[] node->children;
    node->children = NULL;
  
    return true;
#else
    throw std::runtime_error("Function pruneNode() const should not be used with octomap >= 1.8");
#endif
  }
  
  // method defined for octomap < 1.8 compatibility
  bool Typed3DOcTree::isNodeCollapsible(const Typed3DOcTreeNode* node) const
  {
#ifndef OCTOMAP_PRE_18
    // all children must exist, must not have children of
    // their own and have the same occupancy probability
    if (!nodeChildExists(node, 0))
      return false;

    const Typed3DOcTreeNode* firstChild = getNodeChild(node, 0);
    
    if (nodeHasChildren(firstChild))
      return false;

    for (unsigned int i = 1; i<8; i++)
      // compare nodes only using their occupancy, ignoring color for pruning
      if (!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i)) || !(getNodeChild(node, i)->getValue() == firstChild->getValue()))
        return false;

    return true;
#else
    throw std::runtime_error("Function isNodeCollapsible() const should not be used with octomap >= 1.8");
#endif
  }

  void Typed3DOcTree::updateInnerOccupancy()
  {
    this->updateInnerOccupancyRecurs(this->root, 0);
  }

  void Typed3DOcTree::updateInnerOccupancyRecurs(Typed3DOcTreeNode* node, unsigned int depth)
  {
#ifndef OCTOMAP_PRE_18
    // only recurse and update for inner nodes:
    if (nodeHasChildren(node))
    {
      // return early for last level:
      if (depth < this->tree_depth)
        for (unsigned int i = 0; i < 8; i++)
          if (nodeChildExists(node, i))
            updateInnerOccupancyRecurs(getNodeChild(node, i), depth + 1);
      
      node->updateOccupancyChildren();
      node->updateOccupancyTypeChildren();
    }
#else
    // only recurse and update for inner nodes:
    if (node->hasChildren())
    {
      // return early for last level:
      if (depth < this->tree_depth)
      for (unsigned int i = 0; i < 8; i++)
        if (node->childExists(i))
          updateInnerOccupancyRecurs(node->getChild(i), depth+1);
      
      node->updateOccupancyChildren();
      node->updateOccupancyTypeChildren();
    }
#endif
  }

  Typed3DOcTree::StaticMemberInitializer::StaticMemberInitializer()
  {
    Typed3DOcTree* tree = new Typed3DOcTree(0.1);
#ifndef OCTOMAP_PRE_18
    tree->clearKeyRays();
#endif
    AbstractOcTree::registerTreeType(tree);
  }

  Typed3DOcTree::StaticMemberInitializer Typed3DOcTree::typed_3d_oc_tree_member_init_;

  /*
    (OctoMap) method's definition
  */

  OctoMap::OctoMap(float cell_size, 
                   float range_max,
                   unsigned int empty_flood_fill_depth,
	                 float occupancy_thr,
	                 float log_odds_hit,
	                 float log_odds_miss,
	                 float log_odds_clamping_min,
	                 float log_odds_clamping_max)
	: range_max_(range_max),
		empty_flood_fill_depth_(empty_flood_fill_depth),
    cell_size_(cell_size),
    occupancy_thr_(occupancy_thr),
    log_odds_hit_(log_odds_hit),
    log_odds_miss_(log_odds_miss),
    log_odds_clamping_min_(log_odds_clamping_min),
    log_odds_clamping_max_(log_odds_clamping_max)
  {
    octree_ = new Typed3DOcTree(cell_size_);
  
    min_values_[0] = min_values_[1] = min_values_[2] = 0.0;
    max_values_[0] = max_values_[1] = max_values_[2] = 0.0;

    assert(cell_size_ > 0.0f);
    assert(log_odds_clamping_max_ > log_odds_clamping_min_);

    if(occupancy_thr_ <= 0.0f)
    {
      WARNING("Cannot set occupancy threshold to null for OctoMap, using default value %f instead.", 0.5);
      
      occupancy_thr_ = 0.5; // default value
    }
    
    octree_->setOccupancyThres(occupancy_thr_);
    log_odds_hit_ = octomap::logodds(log_odds_hit_);
    octree_->setProbHit(octomap::probability(log_odds_hit_));
    log_odds_miss_ = octomap::logodds(log_odds_miss_);
    octree_->setProbMiss(octomap::probability(log_odds_miss_));
    log_odds_clamping_min_ = octomap::logodds(log_odds_clamping_min_);
    octree_->setClampingThresMin(octomap::probability(log_odds_clamping_min_));
    log_odds_clamping_max_ = octomap::logodds(log_odds_clamping_max_);
    octree_->setClampingThresMax(octomap::probability(log_odds_clamping_max_));

    assert(empty_flood_fill_depth_ <= 16);
  }

  OctoMap::~OctoMap()
  {
    this->clear();
    delete octree_;
  }

	pcl::PointCloud<pcl::PointXYZ>::Ptr OctoMap::createCloud(unsigned int tree_depth,
                                                           std::vector<int> * obstacle_indices,
                                                           std::vector<int> * empty_indices,
                                                           std::vector<int> * ground_indices,
                                                           bool original_ref_points,
                                                           std::vector<int> * frontier_indices,
                                                           std::vector<double> * cloud_prob) const
  {
    assert(tree_depth <= octree_->getTreeDepth());

    std::cout << "Depth = " << tree_depth << " (maxDepth = " << octree_->getTreeDepth() << ") octree = " << octree_->size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    cloud->resize(octree_->size());
    
    if (cloud_prob != nullptr)
      cloud_prob->resize(octree_->size());
    if (obstacle_indices != nullptr)
      obstacle_indices->resize(octree_->size());
    if (empty_indices != nullptr)
      empty_indices->resize(octree_->size());
    if (frontier_indices != nullptr)
      frontier_indices->resize(octree_->size());
    if (ground_indices != nullptr)
      ground_indices->resize(octree_->size());

    if(tree_depth == 0)
      tree_depth = octree_->getTreeDepth();

    bool add_all_points = obstacle_indices == 0 && 
                          ground_indices == 0   &&
                          empty_indices == 0;
    int oi = 0;
    int si = 0;
    int ei = 0;
    int fi = 0;
    int gi = 0;
    
    float half_cell_size = octree_->getNodeSize(tree_depth) / 2.0f;

    for (Typed3DOcTree::iterator it = octree_->begin(tree_depth); it != octree_->end(); ++it)
    {
      if(octree_->isNodeOccupied(*it) && (obstacle_indices != nullptr || ground_indices != nullptr || add_all_points))
      {
        octomap::point3d pt = octree_->keyToCoord(it.getKey());

        if(cloud_prob)
          (*cloud_prob)[oi] = it->getOccupancy();

        if(original_ref_points && it->getOccupancyType() > 0)
        {
          const octomap::point3d & p = it->getPointRef();
          (*cloud)[oi].x = p.x();
          (*cloud)[oi].y = p.y();
          (*cloud)[oi].z = p.z();
        }
        else
        {
          (*cloud)[oi].x = pt.x() - half_cell_size;
          (*cloud)[oi].y = pt.y() - half_cell_size;
          (*cloud)[oi].z = pt.z();
        }

        if(it->getOccupancyType() == Typed3DOcTreeNode::kTypeGround)
        {
          if(ground_indices)
          {
            ground_indices->at(gi++) = oi;
          }
        }
        else if(obstacle_indices)
        {
          obstacle_indices->at(si++) = oi;
        }

        ++oi;
      }
      else if(!octree_->isNodeOccupied(*it) && (empty_indices != nullptr || add_all_points || frontier_indices != nullptr))
      {
        octomap::point3d pt = octree_->keyToCoord(it.getKey());
        if(cloud_prob)
          (*cloud_prob)[oi] = it->getOccupancy();
        
        if(frontier_indices != nullptr &&
           (!octree_->search(pt.x() + octree_->getNodeSize(tree_depth), pt.y(), pt.z(), tree_depth) || !octree_->search(pt.x() - octree_->getNodeSize(tree_depth), pt.y(), pt.z(), tree_depth) ||
            !octree_->search(pt.x(), pt.y() + octree_->getNodeSize(tree_depth), pt.z(), tree_depth) || !octree_->search(pt.x(), pt.y() - octree_->getNodeSize(tree_depth), pt.z(), tree_depth) ||
            !octree_->search(pt.x(), pt.y(), pt.z() + octree_->getNodeSize(tree_depth), tree_depth) || !octree_->search(pt.x(), pt.y(), pt.z() - octree_->getNodeSize(tree_depth), tree_depth) )) //ajouter 1 au key ?
        {
          //unknown neighbor FACE cell
          frontier_indices->at(fi++) = oi;
        }
  
        (*cloud)[oi].x = pt.x() - half_cell_size;
        (*cloud)[oi].y = pt.y() - half_cell_size;
        (*cloud)[oi].z = pt.z();
      
        if(empty_indices)
          empty_indices->at(ei++) = oi;
        
        ++oi;
      }
    }

    cloud->resize(oi);
    
    if(cloud_prob)
      cloud_prob->resize(oi);
    if(obstacle_indices)
    {
      obstacle_indices->resize(si);
      std::cout << "obstacle = " << si << std::endl;
    }
    if(empty_indices)
    {
      empty_indices->resize(ei);
      std::cout << "empty = " << ei << std::endl;
    }
    if(frontier_indices)
    {
      frontier_indices->resize(fi);
      std::cout << "frontier = "<< fi << std::endl;
    }
    if(ground_indices)
    {
      ground_indices->resize(gi);
      std::cout << "ground = " << gi << std::endl;
    }

    return cloud;
  }

  void OctoMap::update(const geometry_msgs::msg::TransformStamped& view_point,
                       sensor_msgs::msg::PointCloud2::SharedPtr ground,
                       sensor_msgs::msg::PointCloud2::SharedPtr obstacles,
                       sensor_msgs::msg::PointCloud2::SharedPtr empty)
  {
    pcl::fromROSMsg(*ground, ground_);
    pcl::fromROSMsg(*obstacles, obstacles_);
    pcl::fromROSMsg(*empty, empty_);
    assemble(view_point);
  }

  void OctoMap::update(const geometry_msgs::msg::TransformStamped& view_point,
                       const sensor_msgs::msg::PointCloud2& ground,
                       const sensor_msgs::msg::PointCloud2& obstacles,
                       const sensor_msgs::msg::PointCloud2& empty)
  {
    pcl::fromROSMsg(ground, ground_);
    pcl::fromROSMsg(obstacles, obstacles_);
    pcl::fromROSMsg(empty, empty_);
    assemble(view_point);
  }

  void OctoMap::update(const geometry_msgs::msg::TransformStamped& view_point,
                       const pcl::PointCloud<pcl::PointXYZ>& ground,
                       const pcl::PointCloud<pcl::PointXYZ>& obstacles,
                       const pcl::PointCloud<pcl::PointXYZ>& empty)
  {
    ground_ = ground;
    obstacles_ = obstacles;
    empty_ = empty;
    assemble(view_point);
  }

  void OctoMap::assemble(const geometry_msgs::msg::TransformStamped& view_point)
  {
    float range_max_sqrd = range_max_ * range_max_;

    std::cout << "Adding pose to octomap (resolution = " << octree_->getResolution() << ")";

    octomap::point3d sensor_origin(view_point.transform.translation.x, view_point.transform.translation.y, view_point.transform.translation.z);

    updateMinMax(sensor_origin);

    octomap::OcTreeKey tmpKey;

    if (!octree_->coordToKeyChecked(sensor_origin, tmpKey))
      throw std::runtime_error("Could not generate Key for origin");

    // instead of direct scan insertion, compute update to filter ground:
    octomap::KeySet free_cells;

    // insert ground points only as free:
    unsigned int max_ground_pts = ground_.size();

    std::cout << "Compute free cells (from " << max_ground_pts <<  " ground points)" << std::endl;

    Eigen::Matrix4f m;
    m << 1.0f, 0.0f, 0.0f, static_cast<float>(view_point.transform.translation.x),
         0.0f, 1.0f, 0.0f, static_cast<float>(view_point.transform.translation.y),
         0.0f, 0.0f, 1.0f, static_cast<float>(view_point.transform.translation.z),
         0.0f, 0.0f, 0.0f, 1.0f;
    Eigen::Affine3f t(m);

    pcl::PointCloud<pcl::PointXYZ> tmp_ground = ground_;

    for (unsigned int i = 0; i < max_ground_pts; ++i)
    {
      pcl::PointXYZ pt = tmp_ground.at(i);
      Eigen::Vector4f p(pt.x, pt.y, pt.z, 1.0f);
      p = t.matrix() * p;
      pt.x = p.x();
      pt.y = p.y();
      pt.z = p.z();

      octomap::point3d point(pt.x, pt.y, pt.z);

      bool ignore_occupied_cell = false;
      if (range_max_sqrd > 0.0f)
      {
        octomap::point3d v(pt.x - cell_size_ - sensor_origin.x(), pt.y - cell_size_ - sensor_origin.y(), pt.z - cell_size_ - sensor_origin.z());
        
        if(v.norm_sq() > range_max_sqrd)
        {
          // compute new point to max range
          v.normalize();
          v *= range_max_;
          point = sensor_origin + v;
          ignore_occupied_cell = true;
        }
      }

      if(!ignore_occupied_cell)
      {
        // occupied endpoint
        octomap::OcTreeKey key;
        if (octree_->coordToKeyChecked(point, key))
        {
          updateMinMax(point);
          Typed3DOcTreeNode * n = octree_->updateNode(key, true);

          if(n)
          {
            n->setNodeRefId(1);
            n->setPointRef(point);
            n->setOccupancyType(Typed3DOcTreeNode::kTypeGround);
          }
        }
      }

      // only clear space (ground points)
      octomap::KeyRay key_ray;
      if (octree_->computeRayKeys(sensor_origin, point, key_ray))
        free_cells.insert(key_ray.begin(), key_ray.end());
    }

    std::cout << "Ground cells = " << max_ground_pts << ", Free cells = " << free_cells.size() << std::endl;

    // all other points: free on ray, occupied on endpoint:
    unsigned int max_obstacle_pts = obstacles_.size();
    std::cout << "Compute occupied cells (from " << max_obstacle_pts << " obstacle points)" << std::endl;
    
    pcl::PointCloud<pcl::PointXYZ> tmp_obstacle = obstacles_;
    for (unsigned int i = 0; i < max_obstacle_pts; ++i)
    {
      pcl::PointXYZ pt = tmp_obstacle.at(i);
      Eigen::Vector4f p(pt.x, pt.y, pt.z, 1.0f);
      p = t.matrix() * p;
      pt.x = p.x();
      pt.y = p.y();
      pt.z = p.z();

      octomap::point3d point(pt.x, pt.y, pt.z);

      bool ignore_occupied_cell = false;
      if(range_max_sqrd > 0.0f)
      {
        octomap::point3d v(pt.x - cell_size_ - sensor_origin.x(), pt.y - cell_size_ - sensor_origin.y(), pt.z - cell_size_ - sensor_origin.z());
        if(v.norm_sq() > range_max_sqrd)
        {
          // compute new point to max range
          v.normalize();
          v *= range_max_;
          point = sensor_origin + v;
          ignore_occupied_cell = true;
        }
      }

      if(!ignore_occupied_cell)
      {
        // occupied endpoint
        octomap::OcTreeKey key;
        if (octree_->coordToKeyChecked(point, key))
        {
          updateMinMax(point);
          Typed3DOcTreeNode * n = octree_->updateNode(key, true);

          if(n)
          {
            n->setNodeRefId(1);
            n->setPointRef(point);
            n->setOccupancyType(Typed3DOcTreeNode::kTypeObstacle);
          }
        }
      }

      // free cells
      octomap::KeyRay keyRay;
      if (octree_->computeRayKeys(sensor_origin, point, keyRay))
        free_cells.insert(keyRay.begin(), keyRay.end());
    }

    std::cout << "Occupied cells = " << max_obstacle_pts << ", Free cells = " << free_cells.size() << std::endl;


    // mark free cells only if not seen occupied in this cloud
    for(auto it = free_cells.begin(), end = free_cells.end(); it != end; ++it)
    {
      Typed3DOcTreeNode * n = octree_->updateNode(*it, false,  true);
      if(n && n->getOccupancyType() == Typed3DOcTreeNode::kTypeUnknown)
      {
        n->setOccupancyType(Typed3DOcTreeNode::kTypeEmpty);
        n->setNodeRefId(1);
      }
    }

    // all empty cells
    unsigned int max_empty_pts = empty_.size();
    if(max_empty_pts)
    {
      pcl::PointCloud<pcl::PointXYZ> tmp_empty = empty_;
      std::cout << "Compute free cells (from " << max_empty_pts << " empty points)";

      for (unsigned int i = 0; i < max_empty_pts; ++i)
      {
        pcl::PointXYZ pt = tmp_empty.at(i);
        Eigen::Vector4f p(pt.x, pt.y, pt.z, 1.0f);
        p = t.matrix() * p;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();

        octomap::point3d point(pt.x, pt.y, pt.z);

        bool ignore_cell = false;
        if(range_max_sqrd > 0.0f)
        {
          octomap::point3d v(pt.x - sensor_origin.x(), pt.y - sensor_origin.y(), pt.z - sensor_origin.z());
         
          if(v.norm_sq() > range_max_sqrd)
            ignore_cell = true;
        }

        if(!ignore_cell)
        {
          octomap::OcTreeKey key;
          if (octree_->coordToKeyChecked(point, key))
          {
            updateMinMax(point);
            Typed3DOcTreeNode * n = octree_->updateNode(key, false, true);

            if(n && n->getOccupancyType() == Typed3DOcTreeNode::kTypeUnknown)
            {
              n->setOccupancyType(Typed3DOcTreeNode::kTypeEmpty);
              n->setNodeRefId(1);
            }
          }
        }
      }
    }

    if(empty_.size() || !free_cells.empty())
      octree_->updateInnerOccupancy();

    std::cout << "Finish" << std::endl;
  
    if(empty_flood_fill_depth_ > 0)
    { 
      auto start = std::chrono::steady_clock::now(); 

      auto key = octree_->coordToKey(0, 0, 0, empty_flood_fill_depth_);
      auto pos = octree_->keyToCoord(key);
      
      std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> empty_nodes = findEmptyNode(octree_, empty_flood_fill_depth_, pos);
      std::vector<octomap::OcTreeKey> node_to_delete;
        
      for (Typed3DOcTree::iterator it = octree_->begin_leafs(empty_flood_fill_depth_); it != octree_->end_leafs(); ++it)
      {
        if(!octree_->isNodeOccupied(*it))
        {
          if(!isNodeVisited(empty_nodes, it.getKey()))
          {   
            node_to_delete.push_back(it.getKey());            
          }
        }
      }

      for(unsigned int y = 0; y < node_to_delete.size(); y++)
        octree_->deleteNode(node_to_delete[y], empty_flood_fill_depth_);

      auto end = std::chrono::steady_clock::now();
      int elapsed = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();

      std::cout << "Flood Fill: deleted " << node_to_delete.size() << " empty cells (" << elapsed << "s)" << std::endl;
    }
  }

  bool OctoMap::writeBinary(const std::string & path)
  {
    return octree_->writeBinary(path);
  }

  void OctoMap::clear()
  {
    octree_->clear();
  }

  void OctoMap::updateMinMax(const octomap::point3d & point)
  {
    if(point.x() < min_values_[0])
      min_values_[0] = point.x();
    
    if(point.y() < min_values_[1])
      min_values_[1] = point.y();
    
    if(point.z() < min_values_[2])
      min_values_[2] = point.z();
    
    if(point.x() > max_values_[0])
      max_values_[0] = point.x();
    
    if(point.y() > max_values_[1])
      max_values_[1] = point.y();
    
    if(point.z() > max_values_[2])
      max_values_[2] = point.z();
  }

  std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> OctoMap::findEmptyNode(Typed3DOcTree* octree_,
                                                                                             unsigned int tree_depth,
                                                                                             octomap::point3d start_position)
  {
    std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> explore_node;
    std::queue<octomap::point3d> position_to_explore;
    
    start_position = findCloseEmpty(octree_, tree_depth, start_position);
    
    floodFill(octree_, tree_depth, start_position, explore_node, position_to_explore);
    while(!position_to_explore.empty())
    {    
      floodFill(octree_, tree_depth, position_to_explore.front(), explore_node, position_to_explore);
      position_to_explore.pop();
    }

    return explore_node;
  }

  void OctoMap::floodFill(Typed3DOcTree* octree_,
                          unsigned int tree_depth,
                          octomap::point3d start_position,
                          std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash>& empty_nodes,
                          std::queue<octomap::point3d>& position_to_explore)
  {
    auto key = octree_->coordToKey(start_position, tree_depth);
    if(!isNodeVisited(empty_nodes, key))
    {
      if(isValidEmpty(octree_, tree_depth, start_position))
      {
        empty_nodes.insert(key);
        position_to_explore.push(octomap::point3d(start_position.x() + octree_->getNodeSize(tree_depth), start_position.y(), start_position.z()));
        position_to_explore.push(octomap::point3d(start_position.x() - octree_->getNodeSize(tree_depth), start_position.y(), start_position.z()));
        position_to_explore.push(octomap::point3d(start_position.x(), start_position.y() + octree_->getNodeSize(tree_depth), start_position.z()));
        position_to_explore.push(octomap::point3d(start_position.x(), start_position.y() - octree_->getNodeSize(tree_depth), start_position.z()));
        position_to_explore.push(octomap::point3d(start_position.x(), start_position.y(), start_position.z() + octree_->getNodeSize(tree_depth)));
        position_to_explore.push(octomap::point3d(start_position.x(), start_position.y(), start_position.z() - octree_->getNodeSize(tree_depth)));
      }         
    }
  }
      
  bool OctoMap::isNodeVisited(std::unordered_set<octomap::OcTreeKey,octomap::OcTreeKey::KeyHash> const& empty_nodes,
                              octomap::OcTreeKey const key)
  {
    for(auto it = empty_nodes.find(key); it != empty_nodes.end(); it++)
      if(*it == key)
        return true;

    return false;
  }

  octomap::point3d OctoMap::findCloseEmpty(Typed3DOcTree* octree_,
                                           unsigned int tree_depth,
                                           octomap::point3d start_position)
  {
    //try current position
    if(isValidEmpty(octree_, tree_depth, start_position))
      return start_position;

    //x pos
    if(isValidEmpty(octree_, tree_depth, octomap::point3d(start_position.x() + octree_->getNodeSize(tree_depth), start_position.y(), start_position.z())))
      return octomap::point3d(start_position.x() + octree_->getNodeSize(tree_depth), start_position.y(), start_position.z());
    
    //x neg 
    if(isValidEmpty(octree_, tree_depth, octomap::point3d(start_position.x() - octree_->getNodeSize(tree_depth), start_position.y(), start_position.z())))
      return octomap::point3d(start_position.x() - octree_->getNodeSize(tree_depth), start_position.y(), start_position.z());

    //y pos
    if(isValidEmpty(octree_, tree_depth, octomap::point3d(start_position.x(), start_position.y() + octree_->getNodeSize(tree_depth), start_position.z())))
      return octomap::point3d(start_position.x(), start_position.y() + octree_->getNodeSize(tree_depth), start_position.z());

    //y neg
    if(isValidEmpty(octree_, tree_depth, octomap::point3d(start_position.x() - octree_->getNodeSize(tree_depth), start_position.y(), start_position.z())))
      return octomap::point3d(start_position.x(), start_position.y() - octree_->getNodeSize(tree_depth), start_position.z());

    //z pos
    if(isValidEmpty(octree_, tree_depth, octomap::point3d(start_position.x(), start_position.y(), start_position.z() + octree_->getNodeSize(tree_depth))))
      return octomap::point3d(start_position.x(), start_position.y(), start_position.z() + octree_->getNodeSize(tree_depth));

    //z neg
    if(isValidEmpty(octree_, tree_depth, octomap::point3d(start_position.x(), start_position.y(), start_position.z() - octree_->getNodeSize(tree_depth))))
      return octomap::point3d(start_position.x(), start_position.y(), start_position.z() - octree_->getNodeSize(tree_depth));

    //no valid position
    return start_position;
  }

  bool OctoMap::isValidEmpty(Typed3DOcTree* octree_,
                             unsigned int tree_depth,
                             octomap::point3d start_position)
  {
    auto node_ptr = octree_->search(start_position.x(), start_position.y(), start_position.z(), tree_depth);
    
    if(node_ptr != NULL)
      if(!octree_->isNodeOccupied(*node_ptr))
        return true;

    return false;
  }

} // namespace nav2_costmap_2d