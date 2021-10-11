//
// Created by caochao on 12/31/19.
//

#include "../../include/keypose_graph/keypose_graph.h"
#include <viewpoint_manager/viewpoint_manager.h>

namespace keypose_graph_ns
{
/**
 * Main constructor for keypose nodes that initializes keypose.
 * 
 * @param x x position of keypose node.
 * @param y y position of keypose node.
 * @param z z position of keypose node.
 * @param node_ind unique id for each node.
 * @param keypose_id unique id for each keypose.
 * @param is_keypose keypose or nonkeypose node.
 */
KeyposeNode::KeyposeNode(double x, double y, double z, int node_ind, int keypose_id, bool is_keypose)
  : cell_ind_(0), node_ind_(node_ind), keypose_id_(keypose_id), is_keypose_(is_keypose), is_connected_(true)
{
  position_.x = x;
  position_.y = y;
  position_.z = z;

  offset_to_keypose_.x = 0.0;
  offset_to_keypose_.y = 0.0;
  offset_to_keypose_.z = 0.0;
}

/**
 * Overloaded constructor with a complex representation of keypose node point 
 * that calls main constructor for keypose nodes.
 * 
 * @param point x,y,z of node.
 * @param node_ind unique id for each node.
 * @param keypose_id unique id for each keypose.
 * @param is_keypose keypose or nonkeypose node.
 */
KeyposeNode::KeyposeNode(const geometry_msgs::Point& point, int node_ind, int keypose_id, bool is_keypose)
  : KeyposeNode(point.x, point.y, point.z, node_ind, keypose_id, is_keypose)
{
}

/**
 * Main constructor for KeyposeGraph that reads parameters into class and initializes kd trees 
 * and point clouds.
 * 
 * @param nh main ROS node's handle.
 */
KeyposeGraph::KeyposeGraph(ros::NodeHandle& nh)
  : allow_vertical_edge_(false)
  , current_keypose_id_(0)
  , kAddNodeMinDist(1.0)
  , kAddEdgeCollisionCheckResolution(0.4)
  , kAddEdgeCollisionCheckRadius(0.3)
  , kAddEdgeConnectDistThr(3.0)
  , kAddEdgeToLastKeyposeDistThr(3.0)
  , kAddEdgeVerticalThreshold(1.0)
  , kAddEdgeCollisionCheckPointNumThr(1)
{
  ReadParameters(nh);
  kdtree_connected_nodes_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  connected_nodes_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  kdtree_nodes_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  nodes_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
}

/**
 * Reads parameters from ROS parameter server.
 * 
 * @param nh main ROS node's handle.
 */
void KeyposeGraph::ReadParameters(ros::NodeHandle& nh)
{
  kAddNodeMinDist = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddNodeMinDist", 0.5);
  kAddNonKeyposeNodeMinDist = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddNonKeyposeNodeMinDist", 0.5);
  kAddEdgeConnectDistThr = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddEdgeConnectDistThr", 0.5);
  kAddEdgeToLastKeyposeDistThr = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddEdgeToLastKeyposeDistThr", 0.5);
  kAddEdgeVerticalThreshold = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddEdgeVerticalThreshold", 0.5);
  kAddEdgeCollisionCheckResolution =
      misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddEdgeCollisionCheckResolution", 0.5);
  kAddEdgeCollisionCheckRadius = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddEdgeCollisionCheckRadius", 0.5);
  kAddEdgeCollisionCheckPointNumThr =
      misc_utils_ns::getParam<int>(nh, "keypose_graph/kAddEdgeCollisionCheckPointNumThr", 0.5);
}

/**
 * Adds node to keypose graph.
 * 
 * @param position position of keypose node.
 * @param node_ind node index of keypose.
 * @param keypose_id unique id of keypose.
 * @param is_keypose keypose status of input node.
 */
void KeyposeGraph::AddNode(const geometry_msgs::Point& position, int node_ind, int keypose_id, bool is_keypose)
{
  KeyposeNode new_node(position, node_ind, keypose_id, is_keypose);
  nodes_.push_back(new_node);
  std::vector<int> neighbors;
  graph_.push_back(neighbors);
  std::vector<double> neighbor_dist;
  dist_.push_back(neighbor_dist);
}
/**
 * Adds node to keypose graph, and connects an edge from node to another node.
 * 
 * @param position position of keypose node.
 * @param node_ind node index of keypose.
 * @param keypose_id unique id of keypose.
 * @param is_keypose keypose status of input node.
 * @param connected_node_ind node index of node to connect edge to.
 * @param connected_node_dist distance from input node to connected node.
 */
void KeyposeGraph::AddNodeAndEdge(const geometry_msgs::Point& position, int node_ind, int keypose_id, bool is_keypose,
                                  int connected_node_ind, double connected_node_dist)
{
  AddNode(position, node_ind, keypose_id, is_keypose);
  AddEdge(connected_node_ind, node_ind, connected_node_dist);
}

/**
 * Adds edge from origin to destination node.
 * 
 * @param from_node_ind origin node
 * @param to_node_ind destination node
 * @param dist distance between nodes
 */
void KeyposeGraph::AddEdge(int from_node_ind, int to_node_ind, double dist)
{
  MY_ASSERT(from_node_ind >= 0 && from_node_ind < graph_.size() && from_node_ind < dist_.size());
  MY_ASSERT(to_node_ind >= 0 && to_node_ind < graph_.size() && to_node_ind < dist_.size());

  graph_[from_node_ind].push_back(to_node_ind);
  graph_[to_node_ind].push_back(from_node_ind);

  dist_[from_node_ind].push_back(dist);
  dist_[to_node_ind].push_back(dist);
}

/**
 * Checks if there's a keypose node within reasonable distance from query position by getting 
 * the closest node index and distance to the query, then checking if it lies within range.
 * 
 * @param position query position
 * @return boolean whether node within distance threshold exists.
 */
bool KeyposeGraph::HasNode(const Eigen::Vector3d& position)
{
  int closest_node_ind = -1;
  double min_dist = DBL_MAX;
  geometry_msgs::Point geo_position;
  geo_position.x = position.x();
  geo_position.y = position.y();
  geo_position.z = position.z();
  GetClosestNodeIndAndDistance(geo_position, closest_node_ind, min_dist);
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size())
  {
    double xy_dist = misc_utils_ns::PointXYDist<geometry_msgs::Point>(geo_position, nodes_[closest_node_ind].position_);
    double z_dist = std::abs(geo_position.z - nodes_[closest_node_ind].position_.z);
    if (xy_dist < kAddNonKeyposeNodeMinDist && z_dist < 1.0)
    {
      return true;
    }
  }
  return false;
}

/**
 * Checks if an edge exists between both query nodes.
 * 
 * @param node_ind1 query node 1.
 * @param node_ind2 query node 2.
 */
bool KeyposeGraph::HasEdgeBetween(int node_ind1, int node_ind2)
{
  if (node_ind1 >= 0 && node_ind1 < nodes_.size() && node_ind2 >= 0 && node_ind2 < nodes_.size())
  {
    if (std::find(graph_[node_ind1].begin(), graph_[node_ind1].end(), node_ind2) != graph_[node_ind1].end() ||
        std::find(graph_[node_ind2].begin(), graph_[node_ind2].end(), node_ind1) != graph_[node_ind2].end())
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}

/**
 * Checks for connectivity between two input positions for an edge between both positions, 
 * or if both point to the same node.
 * 
 * @param from_position query position 1.
 * @param to_position query position 2.
 * @return returns true if both positions are same node, or have an edge between them. 
 */
bool KeyposeGraph::IsConnected(const Eigen::Vector3d& from_position, const Eigen::Vector3d& to_position)
{
  geometry_msgs::Point from_node_position;
  from_node_position.x = from_position.x();
  from_node_position.y = from_position.y();
  from_node_position.z = from_position.z();
  int closest_from_node_ind = -1;
  double closest_from_node_dist = DBL_MAX;
  GetClosestNodeIndAndDistance(from_node_position, closest_from_node_ind, closest_from_node_dist);

  geometry_msgs::Point to_node_position;
  to_node_position.x = to_position.x();
  to_node_position.y = to_position.y();
  to_node_position.z = to_position.z();
  int closest_to_node_ind = -1;
  double closest_to_node_dist = DBL_MAX;
  GetClosestNodeIndAndDistance(to_node_position, closest_to_node_ind, closest_to_node_dist);

  if (closest_from_node_ind != -1 && closest_from_node_ind == closest_to_node_ind)
  {
    return true;
  }
  else if (HasEdgeBetween(closest_from_node_ind, closest_to_node_ind))
  {
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * Adds a non keypose node if a node doesn't already exist near the input position. 
 * If node already exists, returns index to preexisting node. 
 * 
 * @param new_node_position 
 * @return index of added non-keypose node, or closest non-keypose node.
 */
int KeyposeGraph::AddNonKeyposeNode(const geometry_msgs::Point& new_node_position)
{
  int new_node_index = -1;
  int closest_node_ind = -1;
  double closest_node_dist = DBL_MAX;
  GetClosestNodeIndAndDistance(new_node_position, closest_node_ind, closest_node_dist);
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size())
  {
    double xy_dist =
        misc_utils_ns::PointXYDist<geometry_msgs::Point>(new_node_position, nodes_[closest_node_ind].position_);
    double z_dist = std::abs(new_node_position.z - nodes_[closest_node_ind].position_.z);
    if (xy_dist < kAddNonKeyposeNodeMinDist && z_dist < 1.0)
    {
      return closest_node_ind;
    }
  }
  new_node_index = nodes_.size();
  KeyposeNode new_node(new_node_position, new_node_index, current_keypose_id_, false);
  new_node.SetCurrentKeyposePosition(current_keypose_position_);
  nodes_.push_back(new_node);
  std::vector<int> neighbors;
  graph_.push_back(neighbors);
  std::vector<double> neighbor_dist;
  dist_.push_back(neighbor_dist);

  return new_node_index;
}

/**
 * Updates keypose graph to include nodes within the input path as long as they do not already 
 * have an edge between them.
 * 
 * @param path input path.
 */
void KeyposeGraph::AddPath(const nav_msgs::Path& path)
{
  if (path.poses.size() < 2)
  {
    return;
  }
  int prev_node_index = -1;
  for (int i = 0; i < path.poses.size(); i++)
  {
    int cur_node_index = AddNonKeyposeNode(path.poses[i].pose.position);
    // TODO: why not just remove i != 0 and start from int i = 1?
    if (i != 0)
    {
      // Add edge to previous node
      if (prev_node_index >= 0 && prev_node_index < nodes_.size())
      {
        // Check duplication
        if (!HasEdgeBetween(prev_node_index, cur_node_index))
        {
          geometry_msgs::Point prev_node_position = nodes_[prev_node_index].position_;
          double dist_to_prev = misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(
              prev_node_position, path.poses[i].pose.position);
          graph_[prev_node_index].push_back(cur_node_index);
          graph_[cur_node_index].push_back(prev_node_index);

          dist_[prev_node_index].push_back(dist_to_prev);
          dist_[cur_node_index].push_back(dist_to_prev);
        }
      }
      else
      {
        ROS_ERROR_STREAM("KeyposeGraph::AddPath: prev_node_index " << prev_node_index << " out of bound [0, "
                                                                   << nodes_.size() - 1 << "]");
        return;
      }
    }
    prev_node_index = cur_node_index;
  }
  UpdateNodes();
}

/**
 * Gets visualization markers for nodes and edges and pushes them into their respective 
 * container.
 * 
 * @param[out] node_marker
 * @param[out] edge_marker
 */
void KeyposeGraph::GetMarker(visualization_msgs::Marker& node_marker, visualization_msgs::Marker& edge_marker)
{
  node_marker.points.clear();
  edge_marker.points.clear();

  for (const auto& node : nodes_)
  {
    node_marker.points.push_back(node.position_);
  }

  // Maintains a vector of edge coordinates to prevent duplicated adding of edge markers.
  std::vector<std::pair<int, int>> added_edge;
  for (int i = 0; i < graph_.size(); i++)
  {
    int start_ind = i;
    for (int j = 0; j < graph_[i].size(); j++)
    {
      int end_ind = graph_[i][j];
      if (std::find(added_edge.begin(), added_edge.end(), std::make_pair(start_ind, end_ind)) == added_edge.end())
      {
        geometry_msgs::Point start_node_position = nodes_[start_ind].position_;
        geometry_msgs::Point end_node_position = nodes_[end_ind].position_;
        edge_marker.points.push_back(start_node_position);
        edge_marker.points.push_back(end_node_position);
        added_edge.emplace_back(start_ind, end_ind);
      }
    }
  }
}

/**
 * Returns total number of connected nodes.
 * 
 * @return number of connected nodes.
 */
int KeyposeGraph::GetConnectedNodeNum()
{
  int connected_node_num = 0;
  for (int i = 0; i < nodes_.size(); i++)
  {
    if (nodes_[i].is_connected_)
    {
      connected_node_num++;
    }
  }
  return connected_node_num;
}

/**
 * Gets position of all nodes. Sets intensity of connected nodes to 10 if connected, and 
 * -1 if not. Push position of all nodes into cloud provided to function.
 * 
 * @param[out] cloud point cloud used for visualization of nodes.
 */
void KeyposeGraph::GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  cloud->clear();
  for (const auto& node : nodes_)
  {
    pcl::PointXYZI point;
    point.x = node.position_.x;
    point.y = node.position_.y;
    point.z = node.position_.z;
    if (node.is_connected_)
    {
      point.intensity = 10;
    }
    else
    {
      point.intensity = -1;
    }
    cloud->points.push_back(point);
  }
}

/**
 * Does a depth first search to look for all nodes that are connected to query node.
 * 
 * @param query_ind query node index.
 * @param[out] connected_node_indices indices of nodes that are connected to query node.
 * @param[out] constraints boolean vector indicating if a particular graph node can be considered for DFS.
 */
void KeyposeGraph::GetConnectedNodeIndices(int query_ind, std::vector<int>& connected_node_indices,
                                           std::vector<bool> constraints)
{
  if (nodes_.size() != constraints.size())
  {
    ROS_ERROR("KeyposeGraph::GetConnectedNodeIndices: constraints size not equal to node size");
    return;
  }
  if (query_ind < 0 || query_ind >= nodes_.size())
  {
    ROS_ERROR_STREAM("KeyposeGraph::GetConnectedNodeIndices: query_ind: " << query_ind << " out of range: [0, "
                                                                          << nodes_.size() << "]");
    return;
  }
  connected_node_indices.clear();
  std::vector<bool> visited(nodes_.size(), false);
  std::stack<int> dfs_stack;
  dfs_stack.push(query_ind);
  while (!dfs_stack.empty())
  {
    int current_ind = dfs_stack.top();
    connected_node_indices.push_back(current_ind);
    dfs_stack.pop();
    if (!visited[current_ind])
    {
      visited[current_ind] = true;
    }
    for (int i = 0; i < graph_[current_ind].size(); i++)
    {
      int neighbor_ind = graph_[current_ind][i];
      if (!visited[neighbor_ind] && constraints[neighbor_ind])
      {
        dfs_stack.push(neighbor_ind);
      }
    }
  }
}

/**
 * Checks all non-keypose nodes on keypose graph for collision. If non-keypose node is in collision, remove 
 * from graph. If non-keypose node is not in collision, check non-keypose node edges for collisions and prune 
 * accordingly as well.
 * 
 * @param robot_position unused //todo: seems to be unused.
 * @param viewpoint_manager for obtaining viewpoint information and checking collision.
 */
void KeyposeGraph::CheckLocalCollision(const geometry_msgs::Point& robot_position,
                                       const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager)
{
  // Get local planning horizon xy size
  int in_local_planning_horizon_count = 0;
  int collision_node_count = 0;
  int collision_edge_count = 0;
  int in_viewpoint_range_count = 0;
  Eigen::Vector3d viewpoint_resolution = viewpoint_manager->GetResolution();
  double max_z_diff = std::max(viewpoint_resolution.x(), viewpoint_resolution.y()) * 2;
  for (int i = 0; i < nodes_.size(); i++)
  {
    if (nodes_[i].is_keypose_)
    {
      continue;
    }

    Eigen::Vector3d node_position =
        Eigen::Vector3d(nodes_[i].position_.x, nodes_[i].position_.y, nodes_[i].position_.z);
    int viewpoint_ind = viewpoint_manager->GetViewPointInd(node_position);
    bool node_in_collision = false;
    
    // Only check nodes if closest viewpoint to node is valid and within z range.
    if (viewpoint_manager->InRange(viewpoint_ind) &&
        std::abs(viewpoint_manager->GetViewPointHeight(viewpoint_ind) - node_position.z()) < max_z_diff)
    {
      in_local_planning_horizon_count++;
      in_viewpoint_range_count++;
      // Case where viewpoint is in collision. Increments collision count and amends graph accordingly.
      if (viewpoint_manager->ViewPointInCollision(viewpoint_ind))
      {
        node_in_collision = true;
        collision_node_count++;
        // Delete all the associated edges as viewpoint is in collision.
        for (int j = 0; j < graph_[i].size(); j++)
        {
          int neighbor_ind = graph_[i][j];
          for (int k = 0; k < graph_[neighbor_ind].size(); k++)
          {
            if (graph_[neighbor_ind][k] == i)
            {
              graph_[neighbor_ind].erase(graph_[neighbor_ind].begin() + k);
              dist_[neighbor_ind].erase(dist_[neighbor_ind].begin() + k);
              k--;
            }
          }
        }
        graph_[i].clear();
        dist_[i].clear();
      }
      else
      {
        Eigen::Vector3d viewpoint_resolution = viewpoint_manager->GetResolution();
        double collision_check_resolution = std::min(viewpoint_resolution.x(), viewpoint_resolution.y()) / 2;
        // Check edge collision
        for (int j = 0; j < graph_[i].size(); j++)
        {
          int neighbor_ind = graph_[i][j];
          Eigen::Vector3d start_position = node_position;
          Eigen::Vector3d end_position = Eigen::Vector3d(
              nodes_[neighbor_ind].position_.x, nodes_[neighbor_ind].position_.y, nodes_[neighbor_ind].position_.z);
          std::vector<Eigen::Vector3d> interp_points;
          misc_utils_ns::LinInterpPoints(start_position, end_position, collision_check_resolution, interp_points);
          for (const auto& collision_check_position : interp_points)
          {
            int viewpoint_ind = viewpoint_manager->GetViewPointInd(collision_check_position);
            if (viewpoint_manager->InRange(viewpoint_ind))
            {
              if (viewpoint_manager->ViewPointInCollision(viewpoint_ind))
              {
                // todo: what is the point of this variable? It is not used anywhere.
                geometry_msgs::Point viewpoint_position = viewpoint_manager->GetViewPointPosition(viewpoint_ind);
                // Delete neighbors' edges
                for (int k = 0; k < graph_[neighbor_ind].size(); k++)
                {
                  if (graph_[neighbor_ind][k] == i)
                  {
                    collision_edge_count++;
                    graph_[neighbor_ind].erase(graph_[neighbor_ind].begin() + k);
                    dist_[neighbor_ind].erase(dist_[neighbor_ind].begin() + k);
                    k--;
                  }
                }
                // Delete the node's edge
                graph_[i].erase(graph_[i].begin() + j);
                dist_[i].erase(dist_[i].begin() + j);
                j--;
                break;
              }
            }
          }
        }
      }
    }
  }
}

/**
 * Updates cloud with nodes within keypose graph, with intensity sorted according to 
 * node order.
 */
void KeyposeGraph::UpdateNodes()
{
  nodes_cloud_->clear();
  for (int i = 0; i < nodes_.size(); i++)
  {
    pcl::PointXYZI point;
    point.x = nodes_[i].position_.x;
    point.y = nodes_[i].position_.y;
    point.z = nodes_[i].position_.z;
    point.intensity = i;
    nodes_cloud_->points.push_back(point);
  }
  if (!nodes_cloud_->points.empty())
  {
    kdtree_nodes_->setInputCloud(nodes_cloud_);
  }
}

/**
 * Checks for connectivity from first keypose node. Gets all nodes connected to first keypose node and 
 * populates a cloud with it. If keypose node is not valid or doesn't exist, take closest keypose node 
 * to robot's current position.
 * 
 * @param robot_position used only if there is no keypose node within the keypose graph.
 */
void KeyposeGraph::CheckConnectivity(const geometry_msgs::Point& robot_position)
{
  if (nodes_.empty())
  {
    return;
  }
  UpdateNodes();

  // The first keypose node is always connected, set all the others to be disconnected
  int first_keypose_node_ind = -1;
  bool found_connected = false;

  for (int i = 0; i < nodes_.size(); i++)
  {
    if (nodes_[i].is_keypose_)
    {
      first_keypose_node_ind = i;
      break;
    }
  }

  // Check the connectivity starting from the robot
  for (int i = 0; i < nodes_.size(); i++)
  {
    nodes_[i].is_connected_ = false;
  }
  // Gets all connected nodes to first keypose node if first keypose node is valid.
  if (first_keypose_node_ind >= 0 && first_keypose_node_ind < nodes_.size())
  {
    nodes_[first_keypose_node_ind].is_connected_ = true;
    connected_node_indices_.clear();
    std::vector<bool> constraint(nodes_.size(), true);
    GetConnectedNodeIndices(first_keypose_node_ind, connected_node_indices_, constraint);
  }
  // Attempts to get connected nodes to closest node to robot's current position.
  else
  {
    int robot_node_ind = -1;
    double robot_node_dist = DBL_MAX;
    GetClosestNodeIndAndDistance(robot_position, robot_node_ind, robot_node_dist);
    if (robot_node_ind >= 0 && robot_node_ind < nodes_.size())
    {
      nodes_[robot_node_ind].is_connected_ = true;
      connected_node_indices_.clear();
      std::vector<bool> constraint(nodes_.size(), true);
      GetConnectedNodeIndices(robot_node_ind, connected_node_indices_, constraint);
    }
    else
    {
      ROS_ERROR_STREAM("KeyposeGraph::CheckConnectivity: Cannot get closest robot node ind " << robot_node_ind);
    }
  }

  // Populates connected_nodes_cloud with first keypose node and all nodes connected to it.
  connected_nodes_cloud_->clear();
  for (int i = 0; i < connected_node_indices_.size(); i++)
  {
    int node_ind = connected_node_indices_[i];
    nodes_[node_ind].is_connected_ = true;
    pcl::PointXYZI point;
    point.x = nodes_[node_ind].position_.x;
    point.y = nodes_[node_ind].position_.y;
    point.z = nodes_[node_ind].position_.z;
    point.intensity = node_ind;
    connected_nodes_cloud_->points.push_back(point);
  }
  if (!connected_nodes_cloud_->points.empty())
  {
    kdtree_connected_nodes_->setInputCloud(connected_nodes_cloud_);
  }
}

/**
 * Adds keypose node onto the graph and adds edges to all nodes within range. 
 * Adds keypose node and connect it to either the last keypose (if in range), or to the closest keypose node. 
 * Checks through all nodes that are within range, and add edge to them if within range.
 * 
 * @param keypose contains keypose position, and uses covariance field to store keypose ID.
 * @param planning_env used for checking collision.
 */
int KeyposeGraph::AddKeyposeNode(const nav_msgs::Odometry& keypose, const planning_env_ns::PlanningEnv& planning_env)
{
  current_keypose_position_ = keypose.pose.pose.position;
  current_keypose_id_ = static_cast<int>(keypose.pose.covariance[0]);
  int new_node_ind = nodes_.size();
  int keypose_node_count = 0;
  for (int i = 0; i < nodes_.size(); i++)
  {
    if (nodes_[i].is_keypose_)
    {
      keypose_node_count++;
    }
  }
  // If no keyposes exist, simply add node and return index.
  if (nodes_.empty() || keypose_node_count == 0)
  {
    AddNode(current_keypose_position_, new_node_ind, current_keypose_id_, true);
    return new_node_ind;
  }

  else
  {
    double min_dist = DBL_MAX;
    int min_dist_ind = -1;
    double last_keypose_dist = DBL_MAX;
    int last_keypose_ind = -1;
    int max_keypose_id = 0;
    std::vector<int> in_range_node_indices;
    std::vector<double> in_range_node_dist;
    // Iterating through all nodes and getting nodes that are within a distance range.
    for (int i = 0; i < nodes_.size(); i++)
    {
      if (!allow_vertical_edge_)
      {
        if (std::abs(nodes_[i].position_.z - current_keypose_position_.z) > kAddEdgeVerticalThreshold)
        {
          continue;
        }
      }
      double dist = misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_,
                                                                                            current_keypose_position_);
      if (dist < min_dist && nodes_[i].is_keypose_)
      {
        min_dist = dist;
        min_dist_ind = i;
      }
      int keypose_id = nodes_[i].keypose_id_;
      if (keypose_id > max_keypose_id && nodes_[i].is_keypose_)
      {
        last_keypose_dist = dist;
        last_keypose_ind = i;
        max_keypose_id = keypose_id;
      }
      if (dist < kAddEdgeConnectDistThr)
      {
        in_range_node_indices.push_back(i);
        in_range_node_dist.push_back(dist);
      }
    }
    // If the closest keypose node is some distance away
    if (min_dist_ind >= 0 && min_dist_ind < nodes_.size())
    {
      if (min_dist > kAddNodeMinDist)
      {
        // If the last keypose is within range
        if (last_keypose_dist < kAddEdgeToLastKeyposeDistThr && last_keypose_ind >= 0 &&
            last_keypose_ind < nodes_.size())
        {
          // Add edge to the last keypose node
          AddNodeAndEdge(current_keypose_position_, new_node_ind, current_keypose_id_, true, last_keypose_ind,
                         last_keypose_dist);
        }
        else
        {
          // Add edge to the nearest node
          AddNodeAndEdge(current_keypose_position_, new_node_ind, current_keypose_id_, true, min_dist_ind, min_dist);
        }
        // Check other nodes that are within range. If not in collision, add edge from new node.
        if (!in_range_node_indices.empty())
        {
          for (int idx = 0; idx < in_range_node_indices.size(); idx++)
          {
            int in_range_ind = in_range_node_indices[idx];
            if (in_range_ind >= 0 && in_range_ind < nodes_.size())
            {
              // Collision check
              KeyposeNode neighbor_node = nodes_[in_range_ind];
              if (std::find(graph_[new_node_ind].begin(), graph_[new_node_ind].end(), in_range_ind) !=
                  graph_[new_node_ind].end())
                continue;
              double neighbor_node_dist = in_range_node_dist[idx];
              double diff_x = neighbor_node.position_.x - current_keypose_position_.x;
              double diff_y = neighbor_node.position_.y - current_keypose_position_.y;
              double diff_z = neighbor_node.position_.z - current_keypose_position_.z;
              int check_point_num = static_cast<int>(neighbor_node_dist / kAddEdgeCollisionCheckResolution);
              bool in_collision = false;
              for (int i = 0; i < check_point_num; i++)
              {
                // std::cout << "checking the " << i << " collision point" << std::endl;
                double check_point_x =
                    current_keypose_position_.x + kAddEdgeCollisionCheckResolution * i * diff_x / neighbor_node_dist;
                double check_point_y =
                    current_keypose_position_.y + kAddEdgeCollisionCheckResolution * i * diff_y / neighbor_node_dist;
                double check_point_z =
                    current_keypose_position_.z + kAddEdgeCollisionCheckResolution * i * diff_z / neighbor_node_dist;
                if (planning_env.InCollision(check_point_x, check_point_y, check_point_z))
                {
                  in_collision = true;
                  break;
                }
              }
              if (!in_collision)
              {
                AddEdge(new_node_ind, in_range_ind, neighbor_node_dist);
              }
            }
          }
        }
        return new_node_ind;
      }
      else
      {
        return min_dist_ind;
      }
    }
    else
    {
      ROS_ERROR_STREAM("KeyposeGraph::AddKeyposeNode: Nearest keypose ind out of range: " << min_dist_ind);
      return new_node_ind;
    }
  }
}

/**
 * Gets closest connected node to point. If node is within range and distance is less than a 
 * threshold, consider the position to be reachable.
 * 
 * @param point query point.
 * @param dist_threshold limit for point to be considered reachable.
 * @return boolean indicating if position is reachable.
 */
bool KeyposeGraph::IsPositionReachable(const geometry_msgs::Point& point, double dist_threshold)
{
  int closest_node_ind = 0;
  double closest_node_dist = DBL_MAX;
  GetClosestConnectedNodeIndAndDistance(point, closest_node_ind, closest_node_dist);
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size() && closest_node_dist < dist_threshold)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * Gets closest connected node to point. If node is within range and distance is less than a 
 * threshold, consider the position to be reachable.
 * 
 * @param point query point.
 * @return boolean indicating if position is reachable.
 */
bool KeyposeGraph::IsPositionReachable(const geometry_msgs::Point& point)
{
  int closest_node_ind = 0;
  double closest_node_dist = DBL_MAX;
  GetClosestConnectedNodeIndAndDistance(point, closest_node_ind, closest_node_dist);
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size() && closest_node_dist < kAddNonKeyposeNodeMinDist)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * Gets closest connected node to point.
 * 
 * @param point query point.
 * @return index of closest node.
 */
int KeyposeGraph::GetClosestNodeInd(const geometry_msgs::Point& point)
{
  int node_ind = 0;
  double min_dist = DBL_MAX;
  GetClosestNodeIndAndDistance(point, node_ind, min_dist);
  return node_ind;
}

/**
 * Gets index of closest connected node to point and its distance.
 * 
 * @param point query point.
 * @param[out] node_ind index of closest connected node.
 * @param[out] dist distance of closest connected node.
 */
void KeyposeGraph::GetClosestNodeIndAndDistance(const geometry_msgs::Point& point, int& node_ind, double& dist)
{
  node_ind = -1;
  dist = DBL_MAX;
  if (nodes_cloud_->points.empty())
  {
    // TODO: redundant
    node_ind = -1;
    dist = DBL_MAX;
    return;
  }
  pcl::PointXYZI search_point;
  search_point.x = point.x;
  search_point.y = point.y;
  search_point.z = point.z;
  std::vector<int> nearest_neighbor_node_indices(1);
  std::vector<float> nearest_neighbor_squared_dist(1);
  kdtree_nodes_->nearestKSearch(search_point, 1, nearest_neighbor_node_indices, nearest_neighbor_squared_dist);
  if (!nearest_neighbor_node_indices.empty() && nearest_neighbor_node_indices.front() >= 0 &&
      nearest_neighbor_node_indices.front() < nodes_cloud_->points.size())
  {
    node_ind = static_cast<int>(nodes_cloud_->points[nearest_neighbor_node_indices.front()].intensity);
    dist = sqrt(nearest_neighbor_squared_dist.front());
  }
  else
  {
    ROS_WARN_STREAM("KeyposeGraph::GetClosestNodeIndAndDistance: search for nearest neighbor failed with "
                    << nodes_cloud_->points.size() << " nodes.");
    if (!nearest_neighbor_node_indices.empty())
    {
      ROS_WARN_STREAM("Nearest neighbor node Ind: " << nearest_neighbor_node_indices.front());
    }
    for (int i = 0; i < nodes_.size(); i++)
    {
      geometry_msgs::Point node_position = nodes_[i].position_;
      double dist_to_query =
          misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(point, node_position);
      if (dist_to_query < dist)
      {
        dist = dist_to_query;
        node_ind = i;
      }
    }
  }
}

/**
 * Returns closest node index and its distance to query point. Finds closest node to query point using 
 * KD Tree.
 * 
 * @param point query point.
 * @param[out] node_ind index of node closest to point.
 * @param[out] dist distance of node to the point.
 */
void KeyposeGraph::GetClosestConnectedNodeIndAndDistance(const geometry_msgs::Point& point, int& node_ind, double& dist)
{
  if (connected_nodes_cloud_->points.empty())
  {
    node_ind = -1;
    dist = DBL_MAX;
    return;
  }
  pcl::PointXYZI search_point;
  search_point.x = point.x;
  search_point.y = point.y;
  search_point.z = point.z;
  std::vector<int> nearest_neighbor_node_indices(1);
  std::vector<float> nearest_neighbor_squared_dist(1);
  kdtree_connected_nodes_->nearestKSearch(search_point, 1, nearest_neighbor_node_indices,
                                          nearest_neighbor_squared_dist);
  if (!nearest_neighbor_node_indices.empty() && nearest_neighbor_node_indices.front() >= 0 &&
      nearest_neighbor_node_indices.front() < connected_nodes_cloud_->points.size())
  {
    node_ind = static_cast<int>(connected_nodes_cloud_->points[nearest_neighbor_node_indices.front()].intensity);
    dist = sqrt(nearest_neighbor_squared_dist.front());
  }
  else
  {
    ROS_WARN_STREAM("KeyposeGraph::GetClosestNodeInd: search for nearest neighbor failed with "
                    << connected_nodes_cloud_->points.size() << " connected nodes.");
    node_ind = -1;
    dist = 0;
  }
}

/**
 * Returns keypose id of the closest node to query point.
 * 
 * @param point query point.
 * @return keypose id.
 */
int KeyposeGraph::GetClosestKeyposeID(const geometry_msgs::Point& point)
{
  int closest_node_ind = GetClosestNodeInd(point);
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size())
  {
    return nodes_[closest_node_ind].keypose_id_;
  }
  else
  {
    return -1;
  }
}

/**
 * Get position of closest keypose node.
 * 
 * @param point query point.
 * @return closest keypose node to query point.
 */
geometry_msgs::Point KeyposeGraph::GetClosestNodePosition(const geometry_msgs::Point& point)
{
  int closest_node_ind = GetClosestNodeInd(point);
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size())
  {
    return nodes_[closest_node_ind].position_;
  }
  else
  {
    geometry_msgs::Point point;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    return point;
  }
}

/**
 * Finds keypose node closest to start point, and keypose node closest to target point, and 
 * performs A* search from start to end with a maximum path length as a constraint.
 * 
 * @param start_point start point.
 * @param target_point target point.
 * @param max_path_length constraint of path length.
 * @param get_path boolean signifying if path is needed.
 * @param path path to be returned.
 * @return success status of function.
 */
// TODO: Not sure if I'm missing something but this function does not return a boolean when nodes < 2
bool KeyposeGraph::GetShortestPathWithMaxLength(const geometry_msgs::Point& start_point,
                                                const geometry_msgs::Point& target_point, double max_path_length,
                                                bool get_path, nav_msgs::Path& path)
{
  // if there are less than 2 keypose nodes, just return start to end path.
  if (nodes_.size() < 2)
  {
    if (get_path)
    {
      geometry_msgs::PoseStamped start_pose;
      start_pose.pose.position = start_point;
      geometry_msgs::PoseStamped target_pose;
      target_pose.pose.position = target_point;
      path.poses.push_back(start_pose);
      path.poses.push_back(target_pose);
    }
    return misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(start_point, target_point);
  }
  int from_idx = 0;
  int to_idx = 0;
  double min_dist_to_start = DBL_MAX;
  double min_dist_to_target = DBL_MAX;
  // Getting closest node to start and target.
  for (int i = 0; i < nodes_.size(); i++)
  {
    if (allow_vertical_edge_)
    {
      double dist_to_start =
          misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, start_point);
      double dist_to_target =
          misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, target_point);
      // get keypose node closest to start point.
      if (dist_to_start < min_dist_to_start)
      {
        min_dist_to_start = dist_to_start;
        from_idx = i;
      }
      // get keypose node closest to target.
      if (dist_to_target < min_dist_to_target)
      {
        min_dist_to_target = dist_to_target;
        to_idx = i;
      }
    }
    else
    {
      // If vertical edges are not allowed, only allow points within a threshold.
      double z_diff_to_start = std::abs(nodes_[i].position_.z - start_point.z);
      double z_diff_to_target = std::abs(nodes_[i].position_.z - target_point.z);
      // TODO: parameterize this
      if (z_diff_to_start < 1.5)
      {
        double xy_dist_to_start =
            misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, start_point);
        if (xy_dist_to_start < min_dist_to_start)
        {
          min_dist_to_start = xy_dist_to_start;
          from_idx = i;
        }
      }
      // TODO: parameterize this
      if (z_diff_to_target < 1.5)
      {
        double xy_dist_to_target =
            misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, target_point);
        if (xy_dist_to_target < min_dist_to_target)
        {
          min_dist_to_target = xy_dist_to_target;
          to_idx = i;
        }
      }
    }
  }

  std::vector<geometry_msgs::Point> node_positions;
  for (int i = 0; i < nodes_.size(); i++)
  {
    node_positions.push_back(nodes_[i].position_);
  }
  std::vector<int> path_indices;
  double shortest_dist = DBL_MAX;
  bool found_path = misc_utils_ns::AStarSearchWithMaxPathLength(graph_, dist_, node_positions, from_idx, to_idx,
                                                                get_path, path_indices, shortest_dist, max_path_length);
  if (found_path && get_path)
  {
    path.poses.clear();
    for (const auto& ind : path_indices)
    {
      geometry_msgs::PoseStamped pose;
      pose.pose.position = nodes_[ind].position_;
      pose.pose.orientation.w = nodes_[ind].keypose_id_;
      pose.pose.orientation.x = ind;
      path.poses.push_back(pose);
    }
  }

  return found_path;
}

/**
 * Gets the shortest path from start to target points.
 * 
 * Uses A* search to check for shortest path from index closest to start point and index closest ot target point. 
 * Updates path if specified and returns the distance of the shortest path.
 * 
 * @param start_point start point.
 * @param target_point target point.
 * @param get_path boolean specifying if path is required.
 * @param[out] path shortest path.
 * @param use_connected_nodes boolean specifying if only connected nodes are to be used.
 */
double KeyposeGraph::GetShortestPath(const geometry_msgs::Point& start_point, const geometry_msgs::Point& target_point,
                                     bool get_path, nav_msgs::Path& path, bool use_connected_nodes)
{
  // Return straight path between start and target point if there are one or less nodes in the keypose graph only.
  if (nodes_.size() < 2)
  {
    if (get_path)
    {
      geometry_msgs::PoseStamped start_pose;
      start_pose.pose.position = start_point;
      geometry_msgs::PoseStamped target_pose;
      target_pose.pose.position = target_point;
      path.poses.push_back(start_pose);
      path.poses.push_back(target_pose);
    }
    return misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(start_point, target_point);
  }
  int from_idx = 0;
  int to_idx = 0;
  double min_dist_to_start = DBL_MAX;
  double min_dist_to_target = DBL_MAX;
  for (int i = 0; i < nodes_.size(); i++)
  {
    if (use_connected_nodes && !nodes_[i].is_connected_)
    {
      continue;
    }
    // Iterates through all nodes and find the shortest node from beginning / target.
    if (allow_vertical_edge_)
    {
      double dist_to_start =
          misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, start_point);
      double dist_to_target =
          misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, target_point);
      if (dist_to_start < min_dist_to_start)
      {
        min_dist_to_start = dist_to_start;
        from_idx = i;
      }
      if (dist_to_target < min_dist_to_target)
      {
        min_dist_to_target = dist_to_target;
        to_idx = i;
      }
    }
    else
    {
      double z_diff_to_start = std::abs(nodes_[i].position_.z - start_point.z);
      double z_diff_to_target = std::abs(nodes_[i].position_.z - target_point.z);
      // Looks for edges that are less than 1.5m.
      // TODO: parameterize this
      if (z_diff_to_start < 1.5)
      {
        double xy_dist_to_start =
            misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, start_point);
        if (xy_dist_to_start < min_dist_to_start)
        {
          min_dist_to_start = xy_dist_to_start;
          from_idx = i;
        }
      }
      if (z_diff_to_target < 1.5)
      {
        double xy_dist_to_target =
            misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, target_point);
        if (xy_dist_to_target < min_dist_to_target)
        {
          min_dist_to_target = xy_dist_to_target;
          to_idx = i;
        }
      }
    }
  }

  std::vector<geometry_msgs::Point> node_positions;
  for (int i = 0; i < nodes_.size(); i++)
  {
    node_positions.push_back(nodes_[i].position_);
  }
  std::vector<int> path_indices;
  double shortest_dist =
      misc_utils_ns::AStarSearch(graph_, dist_, node_positions, from_idx, to_idx, get_path, path_indices);
  if (get_path)
  {
    path.poses.clear();
    for (const auto& ind : path_indices)
    {
      geometry_msgs::PoseStamped pose;
      pose.pose.position = nodes_[ind].position_;
      pose.pose.orientation.w = nodes_[ind].keypose_id_;
      pose.pose.orientation.x = ind;
      path.poses.push_back(pose);
    }
  }

  return shortest_dist;
}

/**
 * Returns the very first keypose position within the keypose graph.
 */
geometry_msgs::Point KeyposeGraph::GetFirstKeyposePosition()
{
  geometry_msgs::Point point;
  point.x = 0;
  point.y = 0;
  point.z = 0;
  for (const auto& node : nodes_)
  {
    if (node.IsKeypose())
    {
      point = node.position_;
      break;
    }
  }
  return point;
}

/**
 * For a given keypose id, get the position of the keypose node.
 * 
 * @param keypose_id query keypose.
 */
geometry_msgs::Point KeyposeGraph::GetKeyposePosition(int keypose_id)
{
  geometry_msgs::Point point;
  point.x = 0;
  point.y = 0;
  point.z = 0;
  // TODO: can use normal find function c++.
  for (const auto& node : nodes_)
  {
    if (node.keypose_id_ == keypose_id)
    {
      point = node.position_;
      break;
    }
  }
  return point;
}

/**
 * Return positions of all keyposes within the keypose graph.
 * 
 * @param[out] positions output vector of keypose positions.
 */
void KeyposeGraph::GetKeyposePositions(std::vector<Eigen::Vector3d>& positions)
{
  positions.clear();
  for (const auto& node : nodes_)
  {
    if (node.IsKeypose())
    {
      Eigen::Vector3d position(node.position_.x, node.position_.y, node.position_.z);
      positions.push_back(position);
    }
  }
}

/**
 * Queries keypose graph for node position.
 * 
 * @param node_ind query node.
 */
geometry_msgs::Point KeyposeGraph::GetNodePosition(int node_ind)
{
  geometry_msgs::Point node_position;
  node_position.x = 0;
  node_position.y = 0;
  node_position.z = 0;
  if (node_ind >= 0 && node_ind < nodes_.size())
  {
    node_position = nodes_[node_ind].position_;
  }
  else
  {
    ROS_WARN_STREAM("KeyposeGraph::GetNodePosition: node_ind " << node_ind << " out of bound [0, " << nodes_.size() - 1
                                                               << "]");
  }
  return node_position;
}

}  // namespace keypose_graph_ns
