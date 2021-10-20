/**
 * @file viewpoint_manager.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that manages the viewpoints inside the local planning horizon
 * @version 0.1
 * @date 2020-06-10
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "viewpoint_manager/viewpoint_manager.h"

namespace viewpoint_manager_ns
{
/**
 * Reads parameters from ROS parameter server.
 * 
 * @param nh main ROS node's handle.
 * @return success status for reading parameters.
 */
bool ViewPointManagerParameter::ReadParameters(ros::NodeHandle& nh)
{
  kUseFrontier = misc_utils_ns::getParam<bool>(nh, "kUseFrontier", false);

  dimension_ = 2;

  kNumber.x() = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/number_x", 80);
  kNumber.y() = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/number_y", 80);
  kNumber.z() = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/number_z", 40);
  kViewPointNumber = kNumber.x() * kNumber.y() * kNumber.z();
  kRolloverStepsize = kNumber / 5;

  kResolution.x() = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/resolution_x", 0.5);
  kResolution.y() = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/resolution_y", 0.5);
  kResolution.z() = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/resolution_z", 0.5);
  kConnectivityHeightDiffThr = misc_utils_ns::getParam<double>(nh, "kConnectivityHeightDiffThr", 0.25);
  kViewPointCollisionMargin = misc_utils_ns::getParam<double>(nh, "kViewPointCollisionMargin", 0.5);
  kViewPointCollisionMarginZPlus = misc_utils_ns::getParam<double>(nh, "kViewPointCollisionMarginZPlus", 0.5);
  kViewPointCollisionMarginZMinus = misc_utils_ns::getParam<double>(nh, "kViewPointCollisionMarginZMinus", 0.5);
  kCollisionGridZScale = misc_utils_ns::getParam<double>(nh, "kCollisionGridZScale", 2.0);
  kCollisionGridResolution.x() = misc_utils_ns::getParam<double>(nh, "kCollisionGridResolutionX", 0.5);
  kCollisionGridResolution.y() = misc_utils_ns::getParam<double>(nh, "kCollisionGridResolutionY", 0.5);
  kCollisionGridResolution.z() = misc_utils_ns::getParam<double>(nh, "kCollisionGridResolutionZ", 0.5);
  kLineOfSightStopAtNearestObstacle = misc_utils_ns::getParam<bool>(nh, "kLineOfSightStopAtNearestObstacle", true);
  kCheckDynamicObstacleCollision = misc_utils_ns::getParam<bool>(nh, "kCheckDynamicObstacleCollision", true);
  kCollisionFrameCountMax = misc_utils_ns::getParam<int>(nh, "kCollisionFrameCountMax", 3);
  kViewPointHeightFromTerrain = misc_utils_ns::getParam<double>(nh, "kViewPointHeightFromTerrain", 0.75);
  kViewPointHeightFromTerrainChangeThreshold =
      misc_utils_ns::getParam<double>(nh, "kViewPointHeightFromTerrainChangeThreshold", 0.6);

  kCollisionPointThr = misc_utils_ns::getParam<int>(nh, "kCollisionPointThr", 3);

  for (int i = 0; i < dimension_; i++)
  {
    LocalPlanningHorizonSize(i) = kNumber(i) * kResolution(i);
  }

  kCollisionGridSize = Eigen::Vector3i::Ones();
  for (int i = 0; i < dimension_; i++)
  {
    kCollisionGridSize(i) =
        ceil((kNumber(i) * kResolution(i) + kViewPointCollisionMargin * 2) / kCollisionGridResolution(i));
  }

  kCoverageOcclusionThr = misc_utils_ns::getParam<double>(nh, "kCoverageOcclusionThr", 1.0);
  kCoverageDilationRadius = misc_utils_ns::getParam<double>(nh, "kCoverageDilationRadius", 1.0);
  kCoveragePointCloudResolution = misc_utils_ns::getParam<double>(nh, "kSurfaceCloudDwzLeafSize", 1.0);
  kSensorRange = misc_utils_ns::getParam<double>(nh, "kSensorRange", 10.0);
  kNeighborRange = misc_utils_ns::getParam<double>(nh, "kNeighborRange", 3.0);

  kVerticalFOVRatio = tan(M_PI / 15);
  kDiffZMax = kSensorRange * kVerticalFOVRatio;
  kInFovXYDistThreshold = 3 * (kCoveragePointCloudResolution / 2) / tan(M_PI / 15);
  kInFovZDiffThreshold = 3 * kCoveragePointCloudResolution;

  return true;
}

/**
 * Initializes KdTree and point cloud for viewpoint candidates and viewpoints in collision. Initializes a rolling grid. 
 * Sets origin to zero. Initializes a vector of viewpoints and graph index map according to the number of viewpoints. 
 * Initialize empty viewpoint with LiDAR model at (0,0,0) within each grid. 
 * 
 * //TODO: some functions
 * 
 * Sets local planning horizon size accortding to viewpoint number and resolution.
 * 
 * @param nh main ROS node's handle.
 */
ViewPointManager::ViewPointManager(ros::NodeHandle& nh) : initialized_(false)
{
  vp_.ReadParameters(nh);

  kdtree_viewpoint_candidate_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  kdtree_viewpoint_in_collision_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  viewpoint_candidate_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  viewpoint_in_collision_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

  grid_ = std::make_unique<rolling_grid_ns::RollingGrid>(vp_.kNumber);
  origin_ = Eigen::Vector3d::Zero();

  viewpoints_.resize(vp_.kViewPointNumber);
  for (int x = 0; x < vp_.kNumber.x(); x++)
  {
    for (int y = 0; y < vp_.kNumber.y(); y++)
    {
      for (int z = 0; z < vp_.kNumber.z(); z++)
      {
        Eigen::Vector3i sub(x, y, z);
        int ind = grid_->Sub2Ind(sub);
        viewpoints_[ind] = viewpoint_ns::ViewPoint();
      }
    }
  }

  graph_index_map_.resize(vp_.kViewPointNumber);
  for (auto& ind : graph_index_map_)
  {
    ind = -1;
  }

  ComputeConnectedNeighborIndices();
  ComputeInRangeNeighborIndices();
  GetCollisionCorrespondence();

  local_planning_horizon_size_ = Eigen::Vector3d::Zero();
  for (int i = 0; i < vp_.dimension_; i++)
  {
    local_planning_horizon_size_(i) = vp_.kNumber(i) * vp_.kResolution(i);
  }
}

/**
 * Called at initialization to get connected neighbors and their distances to each viewpoint.
 * 
 * Resizes vector of connected neighbor indices and connected neighbor distance to number of viewpoints. Iterates 
 * through all viewpoints and populates connected neighbor indices and distance with neighbors.
 */
void ViewPointManager::ComputeConnectedNeighborIndices()
{
  connected_neighbor_indices_.resize(vp_.kViewPointNumber);
  connected_neighbor_dist_.resize(vp_.kViewPointNumber);

  // Creating a bunch of neighbors based on different permutations from -1 to 1 around x, y, z.
  std::vector<Eigen::Vector3i> idx_addon;
  for (int x = -1; x <= 1; x++)
  {
    for (int y = -1; y <= 1; y++)
    {
      for (int z = -1; z <= 1; z++)
      {
        // 0, 0, 0 would be useless to add.
        if (x == 0 && y == 0 && z == 0)
          continue;
        idx_addon.push_back(Eigen::Vector3i(x, y, z));
      }
    }
  }

  // Iterate through all viewpoints
  for (int x = 0; x < vp_.kNumber.x(); x++)
  {
    for (int y = 0; y < vp_.kNumber.y(); y++)
    {
      for (int z = 0; z < vp_.kNumber.z(); z++)
      {
        Eigen::Vector3i sub(x, y, z);
        int ind = grid_->Sub2Ind(sub);
        // Create neighbors and populate connected neighbors indices and distance. 
        for (int i = 0; i < idx_addon.size(); i++)
        {
          Eigen::Vector3i neighbor_sub = sub + idx_addon[i];
          if (grid_->InRange(neighbor_sub))
          {
            connected_neighbor_indices_[ind].push_back(grid_->Sub2Ind(neighbor_sub));
            double dist = sqrt(vp_.kResolution.x() * vp_.kResolution.x() * std::abs(idx_addon[i].x()) +
                               vp_.kResolution.y() * vp_.kResolution.y() * std::abs(idx_addon[i].y()) +
                               vp_.kResolution.z() * vp_.kResolution.z() * std::abs(idx_addon[i].z()));
            connected_neighbor_dist_[ind].push_back(dist);
          }
        }
      }
    }
  }
}

/**
 * Called at initialization to compute neighbors in range for each neighbor.
 * 
 * Pushes all viewpoints into cloud. Initializes vector containing indices of neighbors that are in range. Does a 
 * radius search on the KD Tree for indices that are within range, and populates in range neighbor indices vector.
 * 
 */
void ViewPointManager::ComputeInRangeNeighborIndices()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < vp_.kViewPointNumber; i++)
  {
    Eigen::Vector3i sub = grid_->Ind2Sub(i);
    pcl::PointXYZ point;
    point.x = sub.x() * vp_.kResolution.x() + vp_.kResolution.x() / 2.0;
    point.y = sub.y() * vp_.kResolution.y() + vp_.kResolution.y() / 2.0;
    point.z = sub.z() * vp_.kResolution.z() + vp_.kResolution.z() / 2.0;
    cloud->points.push_back(point);
  }

  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree =
      pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  kdtree->setInputCloud(cloud);
  in_range_neighbor_indices_.resize(vp_.kViewPointNumber);
  std::vector<int> in_range_indices;
  std::vector<float> in_range_sqdist;
  for (int i = 0; i < in_range_neighbor_indices_.size(); i++)
  {
    pcl::PointXYZ point = cloud->points[i];
    kdtree->radiusSearch(point, vp_.kNeighborRange, in_range_indices, in_range_sqdist);
    for (const auto& ind : in_range_indices)
    {
      in_range_neighbor_indices_[i].push_back(ind);
    }
  }
}

/**
 * Called at initialization to initialize and populate the collision grid, as well as viewpoint cloud.
 * 
 * Initialize collision grid's origin, and creates a collision grid. Populates viewpoint cloud. Iterates through 
 * collision grid and searches for query points within the viewpoint cloud. Pushes points near query point into 
 * collision grid cell.
 * 
 */
void ViewPointManager::GetCollisionCorrespondence()
{
  misc_utils_ns::Timer timer("get collision grid correspondence");
  timer.Start();

  collision_grid_origin_ = Eigen::Vector3d::Zero();
  for (int i = 0; i < vp_.dimension_; i++)
  {
    collision_grid_origin_(i) -= vp_.kViewPointCollisionMargin;
  }
  std::vector<int> viewpoint_index_correspondence;
  collision_grid_ = std::make_unique<grid_ns::Grid<std::vector<int>>>(
      vp_.kCollisionGridSize, viewpoint_index_correspondence, collision_grid_origin_, vp_.kCollisionGridResolution, 2);
  collision_point_count_.resize(collision_grid_->GetCellNumber(), 0);

  pcl::PointCloud<pcl::PointXYZI>::Ptr viewpoint_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZI>());

  // Get viewpoint cloud
  // int count = 0;
  for (int x = 0; x < vp_.kNumber.x(); x++)
  {
    for (int y = 0; y < vp_.kNumber.y(); y++)
    {
      for (int z = 0; z < vp_.kNumber.z(); z++)
      {
        int ind = grid_->Sub2Ind(Eigen::Vector3i(x, y, z));
        pcl::PointXYZI point;
        point.x = (x + 0.5) * vp_.kResolution.x();
        point.y = (y + 0.5) * vp_.kResolution.y();
        point.z = (z + 0.5) * vp_.kResolution.z();
        point.z *= vp_.kCollisionGridZScale;
        point.intensity = ind;
        viewpoint_cloud->points.push_back(point);
      }
    }
  }
  // std::cout << "computing collision grid viewpoint cloud size: " << viewpoint_cloud->points.size() << std::endl;
  kdtree->setInputCloud(viewpoint_cloud);
  std::vector<int> nearby_viewpoint_indices;
  std::vector<float> nearby_viewpoint_sqdist;
  int count = 0;
  for (int x = 0; x < vp_.kCollisionGridSize.x(); x++)
  {
    for (int y = 0; y < vp_.kCollisionGridSize.y(); y++)
    {
      for (int z = 0; z < vp_.kCollisionGridSize.z(); z++)
      {
        Eigen::Vector3d query_point_position = collision_grid_->Sub2Pos(x, y, z);
        pcl::PointXYZI query_point;
        query_point.x = query_point_position.x();
        query_point.y = query_point_position.y();
        query_point.z = query_point_position.z();
        query_point.z *= vp_.kCollisionGridZScale;
        kdtree->radiusSearch(query_point, vp_.kViewPointCollisionMargin, nearby_viewpoint_indices,
                             nearby_viewpoint_sqdist);
        int grid_ind = collision_grid_->Sub2Ind(x, y, z);
        for (int i = 0; i < nearby_viewpoint_indices.size(); i++)
        {
          int ind = nearby_viewpoint_indices[i];
          int viewpoint_ind = (int)(viewpoint_cloud->points[ind].intensity);
          MY_ASSERT(viewpoint_ind >= 0 && viewpoint_ind < vp_.kViewPointNumber);
          collision_grid_->GetCell(grid_ind).push_back(viewpoint_ind);
        }
      }
    }
  }

  timer.Stop(false);
}

/**
 * Shifts origin_ (hence also local planning horizon) based on robot displacement from previous origin. Generates a 
 * grid of viewpoint positions from this origin. 
 * 
 * Theoreotically, shifts when robot displacement is >= kRolloverStepsize. However current implementation is 
 * kRolloverStepsize/2.
 * 
 * @param robot_position current ^x,<y,z of the robot with respect to starting position.
 * @param kNumber Number of grids in each axis. E.g. 80 by 80 by 40
 * @param kResolution displacement per grid in each axis. E.g. 1 grid == 0.5m in the x axis
 * @param kRolloverStepsize Split the space into steps each with this size. E.g. 5 grids in x direction will give 80/5
 * @return True if private variable origin_ (hence local planning horizon box) is shifted.
 */
bool ViewPointManager::UpdateRobotPosition(const Eigen::Vector3d& robot_position)
{
  robot_position_ = robot_position;
  // Initialization - initializes all possible viewpoints and resets them. 
  if (!initialized_)
  {
    initialized_ = true;
    UpdateOrigin();
    // lays out a grid of viewpoints from the origin referenced by the middle of each grid saved in viewpoints_[ind] 
    for (int x = 0; x < vp_.kNumber.x(); x++)
    {
      for (int y = 0; y < vp_.kNumber.y(); y++)
      {
        for (int z = 0; z < vp_.kNumber.z(); z++)
        {
          int ind = grid_->Sub2Ind(Eigen::Vector3i(x, y, z));
          geometry_msgs::Point position;
          position.x = origin_.x() + x * vp_.kResolution.x() + vp_.kResolution.x() / 2.0;
          position.y = origin_.y() + y * vp_.kResolution.y() + vp_.kResolution.y() / 2.0;
          position.z = robot_position.z();
          SetViewPointPosition(ind, position, true);
          ResetViewPoint(ind, true);
        }
      }
    }
  }
  Eigen::Vector3i robot_grid_sub;
  Eigen::Vector3d diff = robot_position_ - origin_; 
  // std::cout << "robot: " << robot_position_.x() << ',' << robot_position_.y() << "\norigin: " << origin_.x() << ',' << origin_.y() << "\ndiff: " << diff.x() << ',' << diff.y() << "\n\n";
  Eigen::Vector3i sub = Eigen::Vector3i::Zero();
  for (int i = 0; i < vp_.dimension_; i++)
  { // The displacement in terms of number of steps. E.g. [2,2,0]
    // TODO: theoreotically, result should be kept as double
    robot_grid_sub(i) = diff(i) > 0 ? static_cast<int>(diff(i) / (vp_.kRolloverStepsize(i) * vp_.kResolution(i))) : -1;
  }

  Eigen::Vector3i sub_diff = Eigen::Vector3i::Zero();
  for (int i = 0; i < vp_.dimension_; i++)
  { // The number of steps from previous robot position. E.g. [-1,0,0]. There are 5 steps across 1 dimension, previous robot position is in the middle (2.5) 
    // TODO: thereotically, both terms should be kept as a double and the result casted to int (floored)
    sub_diff(i) = (vp_.kNumber(i) / vp_.kRolloverStepsize(i)) / 2 - robot_grid_sub(i);
  }
  // std::cout << "diff: " << diff << "\n\n" << robot_grid_sub << "\n\n" << "sub_diff: " << sub_diff << "\n\n" << std::endl;

  if (sub_diff.x() == 0 && sub_diff.y() == 0 && sub_diff.z() == 0)
  {
    return false;
  }

  Eigen::Vector3i rollover_step;
  // Number of grids to shift by. = step size * number of steps
  rollover_step.x() = std::abs(sub_diff.x()) > 0 ?
                          vp_.kRolloverStepsize.x() * ((sub_diff.x() > 0) ? 1 : -1) * std::abs(sub_diff.x()) :
                          0;
  rollover_step.y() = std::abs(sub_diff.y()) > 0 ?
                          vp_.kRolloverStepsize.y() * ((sub_diff.y() > 0) ? 1 : -1) * std::abs(sub_diff.y()) :
                          0;
  rollover_step.z() = std::abs(sub_diff.z()) > 0 ?
                          vp_.kRolloverStepsize.z() * ((sub_diff.z() > 0) ? 1 : -1) * std::abs(sub_diff.z()) :
                          0;

  // std::cout << "rolling x: " << rollover_step.x() << " y: " << rollover_step.y() << " z: " << rollover_step.z()
  //           << std::endl;
  grid_->Roll(rollover_step);

  misc_utils_ns::Timer reset_timer("reset viewpoint");
  reset_timer.Start();

  // Shifts the origin (by displacement). Due to the way sub_diff was calculated, this is minus instead of plus. diff will now be relative to this new origin. Also shifts the local planning horizon. = number of grids * size of grid
  //   origin_ = origin_ - rollover_step.cast<double>() * vp_.kResolution;
  origin_.x() -= rollover_step.x() * vp_.kResolution.x();
  origin_.y() -= rollover_step.y() * vp_.kResolution.y();
  origin_.z() -= rollover_step.z() * vp_.kResolution.z();

  // Get the indices 
  grid_->GetUpdatedIndices(updated_viewpoint_indices_);
  // Similar to initialized_ part above to generate the grid of viewpoints from updated origin_. However, here we start with indices instead hence we need to transform it to get its x and y.  
  for (const auto& ind : updated_viewpoint_indices_)
  {
    MY_ASSERT(grid_->InRange(ind));
    Eigen::Vector3i sub = grid_->Ind2Sub(ind);
    geometry_msgs::Point new_position;
    new_position.x = origin_.x() + sub.x() * vp_.kResolution.x() + vp_.kResolution.x() / 2.0;
    new_position.y = origin_.y() + sub.y() * vp_.kResolution.y() + vp_.kResolution.y() / 2.0;
    new_position.z = robot_position_.z();
    SetViewPointPosition(ind, new_position);
    ResetViewPoint(ind);
  }
  reset_timer.Stop(false);
  return true;
}

/**
 * Sets origin(x,y,z; in displacement) taking initial robot position as the reference (0,0,0) 
 */
void ViewPointManager::UpdateOrigin()
{
  for (int i = 0; i < vp_.dimension_; i++)
  {
    origin_(i) = robot_position_(i) - (vp_.kResolution(i) * vp_.kNumber(i)) / 2.0;
  }
  std::cout << "robot: " << robot_position_ << "\n\n" << "origin: " << origin_ << "\n\n";
}

/**
 * Returns either viewpoint index or array index depending on use array index - if false, viewpoint index is already 
 * in array index form; if true, viewpoint index has to be converted to array index.
 * 
 * TODO: InRange check already implicitly done in GetArrayInd
 * @param viewpoint_ind original viewpoint index.
 * @param use_array_ind boolean indicating if we should return array index or viewpoint index.
 * @return viewpoint index or array index.
 */
int ViewPointManager::GetViewPointArrayInd(int viewpoint_ind, bool use_array_ind) const
{
  MY_ASSERT(grid_->InRange(viewpoint_ind));
  return (use_array_ind ? viewpoint_ind : grid_->GetArrayInd(viewpoint_ind));
}

/**
 * Gets viewpoint index given a viewpoint array index.
 * 
 * @param viewpoint_array_ind viewpoint array index.
 * @return viewpoint index.
 */
int ViewPointManager::GetViewPointInd(int viewpoint_array_ind) const
{
  return grid_->GetInd(viewpoint_array_ind);
}

/**
 * Given a position, return subspace of grid at that position. origin is at [0,0] 
 * 
 * Function achieves this by getting the displacement of the position from the origin and dividing by resolution. 
 * Sets subspace to -1 where difference is negative.
 * 
 * @param position query position.
 * @return Vector of subspace of grid, with dimensions set to -1 if invalid.
 */
Eigen::Vector3i ViewPointManager::GetViewPointSub(Eigen::Vector3d position)
{
  Eigen::Vector3d diff = position - origin_;
  Eigen::Vector3i sub = Eigen::Vector3i::Zero();
  for (int i = 0; i < vp_.dimension_; i++)
  {
    sub(i) = diff(i) > 0 ? static_cast<int>(diff(i) / vp_.kResolution(i)) : -1;
  }
  return sub;
}

/**
 * Gets subspace of grid at the position, checks if it is in range of the grid, and returns the index of the grid if it is.
 * 
 * @param position query position.
 * @return index of grid, if valid. Else -1.
 */
int ViewPointManager::GetViewPointInd(Eigen::Vector3d position)
{
  Eigen::Vector3i sub = GetViewPointSub(position);
  if (grid_->InRange(sub))
  {
    return grid_->Sub2Ind(sub);
  }
  else
  {
    return -1;
  }
}

/**
 * Iterates through viewpoint candidates. Sets visited viewpoint intensity to -1.0, and unvisted viewpoint intensity 
 * to the number of points the viewpoint covers. Adds small noise to the intensity based on viewpoint number,  
 * presumably to prevent viewpoints of similar intensity to look the same in the visualization cloud. Push viewpoints 
 * with updated intensity into the output visualization cloud.
 * 
 * @param[out] vis_cloud visualization cloud.
 */
void ViewPointManager::GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud)
{
  vis_cloud->clear();
  for (int i = 0; i < vp_.kViewPointNumber; i++)
  {
    if (IsViewPointCandidate(i, true))
    {
      geometry_msgs::Point position = GetViewPointPosition(i, true);
      pcl::PointXYZI vis_point;
      vis_point.x = position.x;
      vis_point.y = position.y;
      vis_point.z = position.z;
      if (ViewPointVisited(i, true))
      {
        vis_point.intensity = -1.0;
      }
      else
      {
        vis_point.intensity = GetViewPointCoveredPointNum(i, true);
        vis_point.intensity += i * 1.0 / 10000.0;
      }
      // if (viewpoints_[i].InCurrentFrameLineOfSight())
      // {
      //   vis_point.intensity = 100;
      // }
      // else
      // {
      //   vis_point.intensity = -1;
      // }
      vis_cloud->points.push_back(vis_point);
    }
  }
}

/**
 * Updates collision frame count within each viewpoint and collision point counts for each collision grid.
 * 
 * Goes through collision cloud and updates collision point count. 
 * If any collision grid is found to have more collision points than a certain threshold, and the point is valid within 
 * a z range, set all viewpoints within that collision grid to be in collision and reset their collision frame counts.
 * 
 * @param collision_cloud collision cloud to iterate through and check for collision.
 */
void ViewPointManager::CheckViewPointCollisionWithCollisionGrid(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& collision_cloud)
{
  // Increment viewpoint collision frame count if viewpoint is in collision.
  for (int i = 0; i < viewpoints_.size(); i++)
  {
    if (ViewPointInCollision(i, true))
    {
      AddViewPointCollisionFrameCount(i, true);
    }
  }
  // Resets collision_point_count and sets collision grid origin.
  std::fill(collision_point_count_.begin(), collision_point_count_.end(), 0);
  collision_grid_origin_ = origin_ - Eigen::Vector3d::Ones() * vp_.kViewPointCollisionMargin;
  collision_grid_->SetOrigin(collision_grid_origin_);
  for (const auto& point : collision_cloud->points)
  {
    Eigen::Vector3i collision_grid_sub = collision_grid_->Pos2Sub(point.x, point.y, point.z);
    if (collision_grid_->InRange(collision_grid_sub))
    {
      int collision_grid_ind = collision_grid_->Sub2Ind(collision_grid_sub);
      collision_point_count_[collision_grid_ind]++;
      // If a particular collision grid's collision points exceed a threshold, set valid viewpoints within the grid to 
      // be in collision, and reset collision frame counts.
      if (collision_point_count_[collision_grid_ind] >= vp_.kCollisionPointThr)
      {
        std::vector<int> collision_viewpoint_indices = collision_grid_->GetCellValue(collision_grid_ind);
        for (int i = 0; i < collision_viewpoint_indices.size(); i++)
        {
          int viewpoint_ind = collision_viewpoint_indices[i];
          MY_ASSERT(viewpoint_ind >= 0 && viewpoint_ind < vp_.kViewPointNumber);
          double z_diff = point.z - GetViewPointHeight(viewpoint_ind);
          if ((z_diff >= 0 && z_diff <= vp_.kViewPointCollisionMarginZPlus) ||
              (z_diff < 0 && z_diff >= -vp_.kViewPointCollisionMarginZMinus))
          {
            SetViewPointCollision(viewpoint_ind, true);
            ResetViewPointCollisionFrameCount(viewpoint_ind);
          }
        }
      }
    }
  }
}

/**
 * Checks if current viewpoint is valid, and that the height doesn't exceed viewpoint height requirements. Returns 
 * collision status of viewpoint.
 * 
 * @param position position of viewpoint.
 * @return collision status of viewpoint.
 */
bool ViewPointManager::InCollision(const Eigen::Vector3d& position)
{
  int viewpoint_ind = GetViewPointInd(position);
  bool node_in_collision = false;
  if (InRange(viewpoint_ind) && std::abs(GetViewPointHeight(viewpoint_ind) - position.z()) <
                                    std::max(vp_.kResolution.x(), vp_.kResolution.y()) * 2)
  {
    if (ViewPointInCollision(viewpoint_ind))
    {
      return true;
    }
  }
  return false;
}

/**
 * Get viewpoint index from position, and check if viewpoint is within current frame line of sight.
 * 
 * @param position position of viewpoint.
 * @return boolean indicating if viewpoint is within current frame line of sight.
 */
bool ViewPointManager::InCurrentFrameLineOfSight(const Eigen::Vector3d& position)
{
  int viewpoint_ind = GetViewPointInd(position);
  bool in_line_of_sight = false;
  if (InRange(viewpoint_ind))
  {
    if (ViewPointInCurrentFrameLineOfSight(viewpoint_ind))
    {
      return true;
    }
  }
  return false;
}

/**
 * Checks if viewpoints are inside the viewpoint boundary polygon or no go boundary polygon. If they are, set viewpoint  
 * to be in collision. 
 */
void ViewPointManager::CheckViewPointBoundaryCollision()
{
  // Check for the polygon boundary and nogo zones
  for (int i = 0; i < vp_.kViewPointNumber; i++)
  {
    geometry_msgs::Point viewpoint_position = GetViewPointPosition(i, true);
    if ((!viewpoint_boundary_.points.empty() &&
         !misc_utils_ns::PointInPolygon(viewpoint_position, viewpoint_boundary_)))
    {
      SetViewPointCollision(i, true, true);
      continue;
    }
    for (int j = 0; j < nogo_boundary_.size(); j++)
    {
      if (!nogo_boundary_[j].points.empty() && misc_utils_ns::PointInPolygon(viewpoint_position, nogo_boundary_[j]))
      {
        SetViewPointCollision(i, true, true);

        break;
      }
    }
  }
}

/**
 * Calls functions to check if viewpoints are in collision, or are within viewpoint boundary or no go boundary.
 * 
 * @param collision_cloud for checking collision.
 */
void ViewPointManager::CheckViewPointCollision(const pcl::PointCloud<pcl::PointXYZI>::Ptr& collision_cloud)
{
  CheckViewPointCollisionWithCollisionGrid(collision_cloud);
  CheckViewPointBoundaryCollision();
}

/**
 * Adds points from terrain cloud that have a greater intensity than the collision threshold into the collision cloud, 
 * then calls function to check which viewpoints are colliding.
 * 
 * @param terrain_cloud cloud containing terrain data.
 * @param collision_threshold threshold to consider terrain point in collision.
 */
void ViewPointManager::CheckViewPointCollisionWithTerrain(const pcl::PointCloud<pcl::PointXYZI>::Ptr& terrain_cloud,
                                                          double collision_threshold)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr collision_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  for (const auto& point : terrain_cloud->points)
  {
    if (point.intensity > collision_threshold)
    {
      collision_cloud->points.push_back(point);
    }
  }
  CheckViewPointCollisionWithCollisionGrid(collision_cloud);
}

/**
 * Raycasts cells from starting position to ending position (viewpoint subspace). Iterates through ray cast cells from 
 * viewpoint to starting position, and updates viewpoints within each raycast cell's collision status and in line of 
 * sight status. Iterates through ray cast cells from starting position to viewpoint position and checks for occlusion 
 * as well, updating viewpoints that have yet to be occluded as within line of sight. 
 * 
 * @param start_sub starting position.
 * @param end_sub viewpoint subspace.
 * @param max_sub maximum subspace for raycasting.
 * @param min_sub minimum subspace for raycasting.
 */
void ViewPointManager::CheckViewPointLineOfSightHelper(const Eigen::Vector3i& start_sub, const Eigen::Vector3i& end_sub,
                                                       const Eigen::Vector3i& max_sub, const Eigen::Vector3i& min_sub)
{
  if (end_sub == start_sub)
    return;
  int viewpoint_ind = grid_->Sub2Ind(end_sub);
  // TODO: viewpoint_position unused.
  geometry_msgs::Point viewpoint_position = GetViewPointPosition(viewpoint_ind);
  std::vector<Eigen::Vector3i> ray_cast_cells;
  misc_utils_ns::RayCast(start_sub, end_sub, max_sub, min_sub, ray_cast_cells);
  if (ray_cast_cells.size() > 1)
  {
    // Checks ray cast cells to the nearest obstacle, updating viewpoint in line of sight for cells and setting 
    // collision status to false for cells that are not in collision.
    if (vp_.kLineOfSightStopAtNearestObstacle)
    {
      bool occlude = false;
      for (int i = 1; i < ray_cast_cells.size(); i++)
      {
        int viewpoint_ind = grid_->Sub2Ind(ray_cast_cells[i]);
        if (ViewPointInCollision(viewpoint_ind) && GetViewPointCollisionFrameCount(viewpoint_ind) == 0)
        {
          occlude = true;
          break;
        }
        if (!occlude)
        {
          SetViewPointInLineOfSight(viewpoint_ind, true);
          if (vp_.kCheckDynamicObstacleCollision &&
              GetViewPointCollisionFrameCount(viewpoint_ind) > vp_.kCollisionFrameCountMax)

          {
            SetViewPointCollision(viewpoint_ind, false);
          }
        }
      }
    }
    else
    {
      bool hit_obstacle = false;
      bool in_line_of_sight = false;
      // Goes through ray cast cells from ending subspace to starting subspace to check if cells hit obstacle.
      for (int i = ray_cast_cells.size() - 1; i >= 0; i--)
      {
        int viewpoint_ind = grid_->Sub2Ind(ray_cast_cells[i]);
        if (ViewPointInCollision(viewpoint_ind) && GetViewPointCollisionFrameCount(viewpoint_ind) == 0)
        {
          hit_obstacle = true;
        }
        // For all subsequent cells after hitting the obstacle, set in line of sight to true.
        if (hit_obstacle && !ViewPointInCollision(viewpoint_ind))
        {
          in_line_of_sight = true;
        }
        // For viewpoints in collision after hitting obstacle, if collision frame count exceeds threshold, set in LOS
        // to true and viewpoint collision to false.
        if (hit_obstacle && ViewPointInCollision(viewpoint_ind) &&
            GetViewPointCollisionFrameCount(viewpoint_ind) > vp_.kCollisionFrameCountMax)
        {
          in_line_of_sight = true;
          if (vp_.kCheckDynamicObstacleCollision)
          {
            SetViewPointCollision(viewpoint_ind, false);
          }
        }
        if (in_line_of_sight)
        {
          SetViewPointInLineOfSight(viewpoint_ind, true);
        }
      }
      // If none of the ray cast cells hit an obstacle, set their corresponding viewpoints to be within line of sight.
      if (!hit_obstacle)
      {
        for (int i = ray_cast_cells.size() - 1; i >= 0; i--)
        {
          int viewpoint_ind = grid_->Sub2Ind(ray_cast_cells[i]);
          SetViewPointInLineOfSight(viewpoint_ind, true);
        }
      }
    }

    // Set in current frame line of sight
    bool occlude = false;
    for (int i = 1; i < ray_cast_cells.size(); i++)
    {
      int viewpoint_ind = grid_->Sub2Ind(ray_cast_cells[i]);
      if (ViewPointInCollision(viewpoint_ind) && GetViewPointCollisionFrameCount(viewpoint_ind) == 0)
      {
        occlude = true;
        break;
      }
      // Set all ray cast cells before first occlusion to be in current frame line of sight.
      if (!occlude)
      {
        SetViewPointInCurrentFrameLineOfSight(viewpoint_ind, true);
      }
    }
  }
}

/**
 * Updates line of sight for first and last viewpoints of each dimension.
 */
void ViewPointManager::CheckViewPointLineOfSight()
{
  if (!initialized_)
    return;

  // Initializing all viewpoints to NOT be in current frame line of sight.
  for (int i = 0; i < viewpoints_.size(); i++)
  {
    SetViewPointInCurrentFrameLineOfSight(i, false, true);
  }

  Eigen::Vector3i robot_sub = GetViewPointSub(robot_position_);
  MY_ASSERT(grid_->InRange(robot_sub));
  int robot_viewpoint_ind = grid_->Sub2Ind(robot_sub);
  SetViewPointInLineOfSight(robot_viewpoint_ind, true);
  SetViewPointInCurrentFrameLineOfSight(robot_viewpoint_ind, true);

  std::vector<bool> checked(vp_.kViewPointNumber, false);
  std::vector<Eigen::Vector3i> ray_cast_cells;
  Eigen::Vector3i max_sub(vp_.kNumber.x() - 1, vp_.kNumber.y() - 1, vp_.kNumber.z() - 1);
  Eigen::Vector3i min_sub(0, 0, 0);

  // Create indices array containing only first and last viewpoints in each dimension.
  int x_indices[2] = { 0, vp_.kNumber.x() - 1 };
  int y_indices[2] = { 0, vp_.kNumber.y() - 1 };
  int z_indices[2] = { 0, vp_.kNumber.z() - 1 };

  // Only first and last x viewpoint, but all viewpoints for y and z.
  // Checks if there is line of sight from robot position to first and last x viewpoints across all dimensions.
  for (int xi = 0; xi < 2; xi++)
  {
    for (int y = 0; y < vp_.kNumber.y(); y++)
    {
      for (int z = 0; z < vp_.kNumber.z(); z++)
      {
        int x = x_indices[xi];
        Eigen::Vector3i end_sub(x, y, z);
        int array_ind = grid_->GetArrayInd(end_sub);
        if (!checked[array_ind])
        {
          CheckViewPointLineOfSightHelper(robot_sub, end_sub, max_sub, min_sub);
          checked[array_ind] = true;
        }
      }
    }
  }

  // Only first and last y viewpoint, but all viewpoints for x and z.
  // Checks if there is line of sight from robot position to first and last y viewpoints across all dimensions.
  for (int x = 0; x < vp_.kNumber.x(); x++)
  {
    for (int yi = 0; yi < 2; yi++)
    {
      for (int z = 0; z < vp_.kNumber.z(); z++)
      {
        int y = y_indices[yi];
        Eigen::Vector3i end_sub(x, y, z);
        int array_ind = grid_->GetArrayInd(end_sub);
        if (!checked[array_ind])
        {
          CheckViewPointLineOfSightHelper(robot_sub, end_sub, max_sub, min_sub);
          checked[array_ind] = true;
        }
      }
    }
  }

  // Only first and last z viewpoint, but all viewpoints for x and y.
  // Checks if there is line of sight from robot position to first and last z viewpoints across all dimensions.
  for (int x = 0; x < vp_.kNumber.x(); x++)
  {
    for (int y = 0; y < vp_.kNumber.y(); y++)
    {
      for (int zi = 0; zi < 2; zi++)
      {
        int z = z_indices[zi];
        Eigen::Vector3i end_sub(x, y, z);
        int array_ind = grid_->GetArrayInd(end_sub);
        if (!checked[array_ind])
        {
          CheckViewPointLineOfSightHelper(robot_sub, end_sub, max_sub, min_sub);
          checked[array_ind] = true;
        }
      }
    }
  }
}

/**
 * Iterates through all viewpoints, and check if viewpoints are in robot field of view. Set line of sight to false if 
 * not within field of view. Sets viewpoint at robot position to be within field of view.
 */
void ViewPointManager::CheckViewPointInFOV()
{
  if (!initialized_)
    return;
  Eigen::Vector3i robot_sub = GetViewPointSub(robot_position_);
  MY_ASSERT(grid_->InRange(robot_sub));
  for (int i = 0; i < vp_.kViewPointNumber; i++)
  {
    geometry_msgs::Point viewpoint_position = GetViewPointPosition(i, true);
    if (!InRobotFOV(Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z)))
    {
      SetViewPointInLineOfSight(i, false, true);
    }
  }
  int robot_viewpoint_ind = grid_->Sub2Ind(robot_sub);
  SetViewPointInLineOfSight(robot_viewpoint_ind, true);
}

/**
 * Checks that the z-difference between point position and viewpoint position does not exceed the vertical FOV ratio 
 * multiplied by the xy-difference.
 * 
 * @param point_position position of point.
 * @param viewpoint_position position of viewpoint to check against.
 * @return true if z-diff does not exceed xy-diff * vertical FOV ratio.
 */
bool ViewPointManager::InFOV(const Eigen::Vector3d& point_position, const Eigen::Vector3d& viewpoint_position)
{
  Eigen::Vector3d diff = point_position - viewpoint_position;
  double xy_diff = sqrt(diff.x() * diff.x() + diff.y() * diff.y());
  double z_diff = std::abs(diff.z());
  if (z_diff < vp_.kVerticalFOVRatio * xy_diff)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * Checks that the z-difference between point position and viewpoint position does not exceed the vertical FOV ratio 
 * multiplied by the xy-difference. Also checks that xy diff is within SensorRange parameter.
 * 
 * @param point_position position of point.
 * @param viewpoint_position position of viewpoint to check against.
 * @return true if z-diff does not exceed xy diff and xy diff is within Sensor Range parameter.
 */
bool ViewPointManager::InFOVAndRange(const Eigen::Vector3d& point_position, const Eigen::Vector3d& viewpoint_position)
{
  Eigen::Vector3d diff = point_position - viewpoint_position;
  double z_diff = std::abs(diff.z());
  if (z_diff > vp_.kDiffZMax)
  {
    return false;
  }
  double xy_diff = sqrt(diff.x() * diff.x() + diff.y() * diff.y());
  if (xy_diff > vp_.kSensorRange)
  {
    return false;
  }
  if (z_diff < vp_.kVerticalFOVRatio * xy_diff)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * Checks if query position is within FOV of robot position.
 * 
 * @param position query position.
 */
bool ViewPointManager::InRobotFOV(const Eigen::Vector3d& position)
{
  return InFOV(position, robot_position_);
}

/**
 * Checks if viewpoint at current robot position is in collision. If it is, take next closest viewpoint that is 
 * collision free. Initialize all viewpoints to not be connected first. Does a DFS through all viewpoint and their 
 * neighbors to find connected viewpoints to viewpoint at robot position, updating viewpoint connectivity accordingly.
 */
void ViewPointManager::CheckViewPointConnectivity()
{
  if (!initialized_)
    return;
  Eigen::Vector3i robot_sub = GetViewPointSub(robot_position_);
  MY_ASSERT(grid_->InRange(robot_sub));
  int robot_ind = grid_->Sub2Ind(robot_sub);
  int robot_array_ind = grid_->GetArrayInd(robot_sub);
  // If viewpoint at robot's current position is in collision, look for the next closest without collision.
  if (ViewPointInCollision(robot_ind))
  {
    // std::cout << "ViewPointManager::CheckViewPointConnectivity: robot in collision" << std::endl;
    // return;
    // Find the nearest viewpoint that is not in collision
    bool found_collision_free_viewpoint = false;
    double min_dist_to_robot = DBL_MAX;
    for (int i = 0; i < vp_.kViewPointNumber; i++)
    {
      int array_ind = grid_->GetArrayInd(i);
      if (!ViewPointInCollision(i))
      {
        geometry_msgs::Point position = GetViewPointPosition(i);
        Eigen::Vector3d viewpoint_position(position.x, position.y, position.z);
        double dist_to_robot = (viewpoint_position - robot_position_).norm();
        if (dist_to_robot < min_dist_to_robot)
        {
          min_dist_to_robot = dist_to_robot;
          robot_ind = i;
          robot_array_ind = array_ind;
          found_collision_free_viewpoint = true;
        }
      }
    }
    if (!found_collision_free_viewpoint)
    {
      std::cout << "All viewpoints in collision, exisiting" << std::endl;
      return;
    }
  }
  // Initializes all viewpoints to be unconnected first.
  for (auto& viewpoint : viewpoints_)
  {
    viewpoint.SetConnected(false);
  }
  std::vector<bool> checked(vp_.kViewPointNumber, false);
  checked[robot_ind] = true;
  SetViewPointConnected(robot_ind, true);
  std::list<int> queue;
  queue.push_back(robot_ind);
  int connected_viewpoint_count = 1;
  // DFS to check if viewpoint is able to be connected. If it is, set it to be connected and push back neighboring indices.
  while (!queue.empty())
  {
    int cur_ind = queue.front();
    queue.pop_front();
    for (int i = 0; i < connected_neighbor_indices_[cur_ind].size(); i++)
    {
      int neighbor_ind = connected_neighbor_indices_[cur_ind][i];
      if (!grid_->InRange(neighbor_ind))
      {
        std::cout << "ViewPointManager::CheckViewPointConnectivity: neighbor ind out of bound" << std::endl;
        continue;
      }
      if (!checked[neighbor_ind] && !ViewPointInCollision(neighbor_ind) && ViewPointInLineOfSight(neighbor_ind))
      {
        if (std::abs(GetViewPointHeight(cur_ind) - GetViewPointHeight(neighbor_ind)) < vp_.kConnectivityHeightDiffThr)
        {
          SetViewPointConnected(neighbor_ind, true);
          connected_viewpoint_count++;
          queue.push_back(neighbor_ind);
        }
      }
      checked[neighbor_ind] = true;
    }
  }
}

/**
 * Iterates through all position within input positions. For position within local horizon, set viewpoint visited to be 
 * true, and set all neighbors of each point as visited.
 * 
 * @param positions input positions, presumably visited by robot.
 */
void ViewPointManager::UpdateViewPointVisited(const std::vector<Eigen::Vector3d>& positions)
{
  if (!initialized_)
    return;

  for (const auto& position : positions)
  {
    if (!InLocalPlanningHorizon(position))
    {
      continue;
    }
    Eigen::Vector3i viewpoint_sub = GetViewPointSub(position); // A scaled position vector from origin to position
    if (grid_->InRange(viewpoint_sub))
    {
      int viewpoint_ind = grid_->Sub2Ind(viewpoint_sub);
      SetViewPointVisited(viewpoint_ind, true);
      for (const auto& neighbor_viewpoint_ind : in_range_neighbor_indices_[viewpoint_ind])
      {
        MY_ASSERT(grid_->InRange(neighbor_viewpoint_ind));
        SetViewPointVisited(neighbor_viewpoint_ind, true);
        // TODO: neighbor_array_ind unused.
        int neighbor_array_ind = grid_->GetArrayInd(neighbor_viewpoint_ind);
      }
    }
  }
}

/**
 * Iterates through all viewpoints, and uses read-only grid world to check if viewpoint status is COVERED_BY_OTHERS. If 
 * so, set viewpoint visited to be true.
 * 
 * @param grid_world read-only grid world used to check if viewpoint has been visited.
 */
void ViewPointManager::UpdateViewPointVisited(std::unique_ptr<grid_world_ns::GridWorld> const& grid_world)
{
  for (int i = 0; i < viewpoints_.size(); i++)
  {
    geometry_msgs::Point viewpoint_position = GetViewPointPosition(i, true);
    int cell_ind = grid_world->GetCellInd(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z);
    if (grid_world->IndInBound((cell_ind)))
    {
      grid_world_ns::CellStatus cell_status = grid_world->GetCellStatus(cell_ind);
      if (cell_status == grid_world_ns::CellStatus::COVERED_BY_OTHERS)
      {
        SetViewPointVisited(i, true, true);
      }
    }
  }
}

/**
 * Gets closest viewpoint to current robot position. If there is no terrain cloud information OR robot viewpoint height 
 * is higher than terrin change threshold, manually sets height of robot viewpoint, updating robot viewpoint and its 
 * neighbors to the new computed height.
 * 
 * Iterates through terrain cloud and does a similar update for viewpoints corresponding to terrain cloud points. 
 * 
 * Finally iterates through all viewpoints and set viewpoint height to be same as their neighbors.
 * 
 * @param terrain_cloud terrain cloud.
 * @param terrain_height_threshold threshold to ignore points from terrain cloud.
 */
void ViewPointManager::SetViewPointHeightWithTerrain(const pcl::PointCloud<pcl::PointXYZI>::Ptr& terrain_cloud,
                                                     double terrain_height_threshold)
{
  // Set the height of the viewpoint nearby the robot to be the height of the robot, in case there is no terrain cloud
  // within the blind spot.
  if (!initialized_)
    return;
  Eigen::Vector3i robot_sub = GetViewPointSub(robot_position_);
  int robot_ind = grid_->Sub2Ind(robot_sub);
  MY_ASSERT(grid_->InRange(robot_sub));
  geometry_msgs::Point robot_viewpoint_position = GetViewPointPosition(robot_ind);

  if (!ViewPointHasTerrainHeight(robot_ind) ||
      std::abs(robot_viewpoint_position.z - robot_position_.z()) > vp_.kViewPointHeightFromTerrainChangeThreshold)
  {
    robot_viewpoint_position.z = robot_position_.z();
    SetViewPointPosition(robot_ind, robot_viewpoint_position);
    for (int i = 0; i < in_range_neighbor_indices_[robot_ind].size(); i++)
    {
      int neighbor_ind = in_range_neighbor_indices_[robot_ind][i];
      MY_ASSERT(grid_->InRange(neighbor_ind));
      if (!ViewPointHasTerrainHeight(neighbor_ind) ||
          std::abs(GetViewPointHeight(neighbor_ind) - robot_position_.z()) > 0.6)
      {
        SetViewPointHeight(neighbor_ind, robot_position_.z());
      }
    }
  }

  // Set the height of other viewpoints
  for (const auto& terrain_point : terrain_cloud->points)
  {
    if (terrain_point.intensity > terrain_height_threshold)
    {
      continue;
    }
    Eigen::Vector3i viewpoint_sub = GetViewPointSub(Eigen::Vector3d(terrain_point.x, terrain_point.y, terrain_point.z));
    if (grid_->InRange(viewpoint_sub))
    {
      int viewpoint_ind = grid_->Sub2Ind(viewpoint_sub);
      double target_height = terrain_point.z + vp_.kViewPointHeightFromTerrain;
      // If the viewpoint has not been set height with terrain points, or if there is a terrain point with a lower
      // height
      if (!ViewPointHasTerrainHeight(viewpoint_ind) || target_height < GetViewPointHeight(viewpoint_ind))
      {
        if (std::abs(target_height - GetViewPointHeight(viewpoint_ind)) >
            vp_.kViewPointHeightFromTerrainChangeThreshold)
        {
          ResetViewPoint(viewpoint_ind);
        }
        SetViewPointHeight(viewpoint_ind, target_height);
        SetViewPointHasTerrainHeight(viewpoint_ind, true);
      }
    }
  }

  // For viewpoints that are not set heights with terrain directly, use neighbors' heights
  for (int i = 0; i < vp_.kViewPointNumber; i++)
  {
    if (!ViewPointHasTerrainHeight(i))
    {
      for (const auto& neighbor_ind : in_range_neighbor_indices_[i])
      {
        MY_ASSERT(grid_->InRange(neighbor_ind));
        if (ViewPointHasTerrainHeight(neighbor_ind))
        {
          double neighbor_height = GetViewPointHeight(neighbor_ind);
          if (std::abs(neighbor_height - GetViewPointHeight(i)) > vp_.kViewPointHeightFromTerrainChangeThreshold)
          {
            geometry_msgs::Point viewpoint_position = GetViewPointPosition(i);
            viewpoint_position.z = neighbor_height;
            ResetViewPoint(i);
            SetViewPointPosition(i, viewpoint_position);
          }
          else
          {
            SetViewPointHeight(i, neighbor_height);
          }
        }
      }
    }
  }
}

/**
 * Resets viewpoint, sets attributes to default.
 * 
 * @param viewpoint_ind
 * @param use_array_ind
 */
void ViewPointManager::ResetViewPoint(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].Reset();
}

/**
 * Resets coverage of all viewpoints.
 */
void ViewPointManager::ResetViewPointCoverage()
{
  for (auto& viewpoint : viewpoints_)
  {
    viewpoint.ResetCoverage();
  }
}

/**
 * Gets viewpoint collision status.
 * @param viewpoint_ind index of viewpoint.
 * @param use_array_ind status indicating array index or regular index.
 * @return viewpoint connected status.
 */
bool ViewPointManager::ViewPointInCollision(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].InCollision();
}
/**
 * Sets viewpoint collision status.
 * @param viewpoint_ind index of viewpoint.
 * @param in_collision viewpoint collision status.
 * @param use_array_ind status indicating array index or regular index.
 */
void ViewPointManager::SetViewPointCollision(int viewpoint_ind, bool in_collision, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetInCollision(in_collision);
}
/**
 * Gets viewpoint in line of sight status.
 * @param viewpoint_ind viewpoint.
 * @param use_array_ind
 * @return viewpoint in line of sight status.
 */
bool ViewPointManager::ViewPointInLineOfSight(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].InLineOfSight();
}
/**
 * Sets viewpoint in line of sight status.
 * @param viewpoint_ind viewpoint.
 * @param in_line_of_sight status indicating candidature.
 * @param use_array_ind
 */
void ViewPointManager::SetViewPointInLineOfSight(int viewpoint_ind, bool in_line_of_sight, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetInLineOfSight(in_line_of_sight);
}
/**
 * Gets viewpoint connected status.
 * @param viewpoint_ind index of viewpoint.
 * @param use_array_ind status indicating array index or regular index.
 * @return viewpoint connected status.
 */
bool ViewPointManager::ViewPointConnected(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].Connected();
}
/**
 * Sets viewpoint connected status.
 * @param viewpoint_ind index of viewpoint.
 * @param connected viewpoint connected status.
 * @param use_array_ind status indicating array index or regular index.
 */
void ViewPointManager::SetViewPointConnected(int viewpoint_ind, bool connected, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetConnected(connected);
}
/**
 * Gets viewpoint visited status.
 * @param viewpoint_ind index of viewpoint.
 * @param use_array_ind status indicating array index or regular index.
 * @return viewpoint visited status.
 */
bool ViewPointManager::ViewPointVisited(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].Visited();
}
/**
 * Sets viewpoint visited status.
 * @param viewpoint_ind index of viewpoint.
 * @param visited viewpoint visited status.
 * @param use_array_ind status indicating array index or regular index.
 */
void ViewPointManager::SetViewPointVisited(int viewpoint_ind, bool visited, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetVisited(visited);
}
/**
 * Gets viewpoint selected status.
 * @param viewpoint_ind index of viewpoint.
 * @param use_array_ind status indicating array index or regular index.
 * @return viewpoint selected status.
 */
bool ViewPointManager::ViewPointSelected(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].Selected();
}
/**
 * Sets viewpoint to be selected.
 * @param viewpoint_ind index of viewpoint.
 * @param selected selected status of viewpoint.
 * @param use_array_ind status indicating array index or regular index.
 */
void ViewPointManager::SetViewPointSelected(int viewpoint_ind, bool selected, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetSelected(selected);
}
/**
 * Gets viewpoint candidature status. 
 * 
 * @param viewpoint_ind index of viewpoint.
 * @param use_array_ind status indicating array index or regular index.
 * @return viewpoint candidature status.
 */
bool ViewPointManager::IsViewPointCandidate(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].IsCandidate();
}
/**
 * Sets viewpoint candidate status.
 * @param viewpoint_ind index of viewpoint.
 * @param candidate status indicating candidature.
 * @param use_array_ind status indicating array index or regular index.
 */
void ViewPointManager::SetViewPointCandidate(int viewpoint_ind, bool candidate, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetCandidate(candidate);
}
/**
 * Gets viewpoint terrain height status. 
 * 
 * @param viewpoint_ind index of viewpoint.
 * @param use_array_ind status indicating array index or regular index.
 * @return viewpoint terrain height status.
 */
bool ViewPointManager::ViewPointHasTerrainHeight(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].HasTerrainHeight();
}
/**
 * Sets viewpoint terrain height status for a given viewpoint.
 * 
 * @param viewpoint_ind index of viewpoint.
 * @param has_terrain_height status indicating if viewpoint has terrain height.
 * @param use_array_ind status indicating array index or regular index.
 */
void ViewPointManager::SetViewPointHasTerrainHeight(int viewpoint_ind, bool has_terrain_height, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetHasTerrainHeight(has_terrain_height);
}
/**
 * Gets viewpoint status for if it is in exploring cell.
 * 
 * @param viewpoint_ind index of viewpoint.
 * @param use_array_ind status indicating array index or regular index.
 */
bool ViewPointManager::ViewPointInExploringCell(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].InExploringCell();
}
/**
 * Sets viewpoint status for if it is in exploring cell.
 * 
 * @param viewpoint_ind index of viewpoint.
 * @param in_exploring_cell status indicating if viewpoint is in exploring cell.
 * @param use_array_ind status indicating array index or regular index.
 */
void ViewPointManager::SetViewPointInExploringCell(int viewpoint_ind, bool in_exploring_cell, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetInExploringCell(in_exploring_cell);
}
/**
 * Gets viewpoint height.
 * 
 * @param viewpoint_ind index of viewpoint.
 * @param use_array_ind status indicating array index or regular index.
 */
double ViewPointManager::GetViewPointHeight(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].GetHeight();
}
/**
 * Sets viewpoint height.
 * 
 * @param viewpoint_ind index of viewpoint.
 * @param height viewpoint height.
 * @param use_array_ind status indicating array index or regular index.
 */
void ViewPointManager::SetViewPointHeight(int viewpoint_ind, double height, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetHeight(height);
}

/**
 * Gets viewpoint in current frame line of sight status.
 * 
 * @param viewpoint_ind index of viewpoint.
 * @param use_array_ind status indicating array index or regular index.
 * @return status indicating if viewpoint is in current frame line of sight.
 */
bool ViewPointManager::ViewPointInCurrentFrameLineOfSight(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].InCurrentFrameLineOfSight();
}
/**
 * Sets viewpoint status for in current frame line of sight.
 * 
 * @param viewpoint_ind index of viewpoint.
 * @param in_current_frame_line_of_sight status indicating if viewpoint is in current frame los.
 * @param use_array_ind status indicating array index or regular index.
 */
void ViewPointManager::SetViewPointInCurrentFrameLineOfSight(int viewpoint_ind, bool in_current_frame_line_of_sight,
                                                             bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetInCurrentFrameLineOfSight(in_current_frame_line_of_sight);
}
/**
 * Gets viewpoint's position.
 * 
 * @param viewpoint_ind index of viewpoint.
 * @param use_array_ind status indicating array index or regular index.
 * @return viewpoint position.
 */
geometry_msgs::Point ViewPointManager::GetViewPointPosition(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].GetPosition();
}
/**
 * Sets viewpoint's position.
 * 
 * @param viewpoint_ind index of viewpoint.
 * @param position viewpoint position.
 * @param use_array_ind status indicating array index or regular index.
 */
void ViewPointManager::SetViewPointPosition(int viewpoint_ind, geometry_msgs::Point position, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetPosition(position);
}
/**
 * Gets viewpoint's cell index.
 * 
 * @param viewpoint_ind index of viewpoint.
 * @param use_array_ind status indicating array index or regular index.
 * @return viewpoint cell index.
 */
int ViewPointManager::GetViewPointCellInd(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].GetCellInd();
}
/**
 * Sets viewpoint's cell index.
 * 
 * @param viewpoint_ind index of viewpoint.
 * @param cell_ind viewpoint cell index to set.
 * @param use_array_ind status indicating array index or regular index.
 */
void ViewPointManager::SetViewPointCellInd(int viewpoint_ind, int cell_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetCellInd(cell_ind);
}
/**
 * Gets viewpoint's collision frame count.
 * 
 * @param viewpoint_ind index of viewpoint.
 * @param use_array_ind status indicating array index or regular index.
 * @return viewpoint collision frame count.
 */
int ViewPointManager::GetViewPointCollisionFrameCount(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].GetCollisionFrameCount();
}
/**
 * Increments viewpoint's collision frame counter.
 * 
 * @param viewpoint_ind index of viewpoint.
 * @param use_array_ind status indicating array index or regular index.
 */
void ViewPointManager::AddViewPointCollisionFrameCount(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].AddCollisionFrame();
}
/**
 * Resets viewpoint's collision frame counter.
 * 
 * @param viewpoint_ind index of viewpoint.
 * @param use_array_ind status indicating array index or regular index.
 */
void ViewPointManager::ResetViewPointCollisionFrameCount(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].ResetCollisionFrameCount();
}
/**
 * Resets viewpoint's covered point list.
 * 
 * @param viewpoint_ind index of viewpoint.
 * @param use_array_ind status indicating array index or regular index.
 */
void ViewPointManager::ResetViewPointCoveredPointList(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].ResetCoveredPointList();
  viewpoints_[array_ind].ResetCoveredFrontierPointList();
}
/**
 * Adds input point to input viewpoint's list of covered points, effectively reclassifying 
 * point from uncovered to covered.
 * 
 * @param viewpoint_ind index of query view point.
 * @param point_ind index of point.
 * @param use_array_ind viewpoint_ind is given in index or array index.
 */
void ViewPointManager::AddUncoveredPoint(int viewpoint_ind, int point_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].AddCoveredPoint(point_ind);
}
/**
 * Adds input frontier point to input viewpoint's list of covered points, effectively reclassifying 
 * point from uncovered to covered.
 * 
 * @param viewpoint_ind index of query view point.
 * @param point_ind index of frontier point.
 * @param use_array_ind viewpoint_ind is given in index or array index.
 */
void ViewPointManager::AddUncoveredFrontierPoint(int viewpoint_ind, int point_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].AddCoveredFrontierPoint(point_ind);
}
/**
 * Gets the list of covered points of the viewpoint being queried within this function. 
 * Covered points refer to points in viewpoints not yet visited that do provide coverage.
 * 
 * @param viewpoint_ind index of query view point.
 * @param use_array_ind viewpoint_ind is given in index or array index.
 * @return covered point list of viewpoint being queried.
 */
const std::vector<int>& ViewPointManager::GetViewPointCoveredPointList(int viewpoint_ind, bool use_array_ind) const
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].GetCoveredPointList();
}
/**
 * Gets the list of covered frontier points of the viewpoint being queried within this function. 
 * Covered points refer to points in viewpoints not yet visited that do provide coverage.
 * 
 * @param viewpoint_ind index of query view point.
 * @param use_array_ind viewpoint_ind is given in index or array index.
 * @return covered point list of viewpoint being queried.
 */
const std::vector<int>& ViewPointManager::GetViewPointCoveredFrontierPointList(int viewpoint_ind,
                                                                               bool use_array_ind) const
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].GetCoveredFrontierPointList();
}
/**
 * Getter for getting viewpoint's list of covered points.
 * 
 * @param viewpoint_ind input viewpoint
 * @param use_array_ind viewpoint_ind is given in index or array index.
 */
int ViewPointManager::GetViewPointCoveredPointNum(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].GetCoveredPointNum();
}

/**
 * Getter for getting viewpoint's list of covered frontier points.
 * 
 * @param viewpoint_ind input viewpoint
 * @param use_array_ind viewpoint_ind is given in index or array index.
 */
int ViewPointManager::GetViewPointCoveredFrontierPointNum(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].GetCoveredFrontierPointNum();
}

/**
 * Iterates through a list of points that the input viewpoint covers, and increments a counter of 
 * covered points if the point hasn't been covered in the input point list. Returns total number of
 * covered points.
 * 
 * @param point_list
 * @param viewpoint_index
 * @param use_array_ind
 * @return The number of covered points.
 */
int ViewPointManager::GetViewPointCoveredPointNum(const std::vector<bool>& point_list, int viewpoint_index,
                                                  bool use_array_ind)
{
  int covered_point_num = 0;
  for (const auto& point_ind : GetViewPointCoveredPointList(viewpoint_index, use_array_ind))
  {
    MY_ASSERT(misc_utils_ns::InRange<bool>(point_list, point_ind));
    if (!point_list[point_ind])
    {
      covered_point_num++;
    }
  }
  return covered_point_num;
}
/**
 * Iterates through a list of points that the input viewpoint covers, and increments a counter of 
 * covered points if the frontier point hasn't been covered in the frontier point list. Returns total number of
 * covered points.
 * 
 * @param point_list
 * @param viewpoint_index
 * @param use_array_ind
 * @return The number of covered points.
 */
int ViewPointManager::GetViewPointCoveredFrontierPointNum(const std::vector<bool>& frontier_point_list,
                                                          int viewpoint_index, bool use_array_ind)
{
  int covered_frontier_point_num = 0;
  for (const auto& point_ind : GetViewPointCoveredFrontierPointList(viewpoint_index, use_array_ind))
  {
    MY_ASSERT(misc_utils_ns::InRange<bool>(frontier_point_list, point_ind));
    if (!frontier_point_list[point_ind])
    {
      covered_frontier_point_num++;
    }
  }
  return covered_frontier_point_num;
}

/**
 * Iterates through list of covered points for a viewpoint, and sets point list to true at covered point indices.
 * 
 * @param[out] point_list boolean list that is true at covered points the viewpoint covers.
 * @param viewpoint_ind index of viewpoint.
 * @param use_array_ind status indicating array index or regular index.
 */
void ViewPointManager::UpdateViewPointCoveredPoint(std::vector<bool>& point_list, int viewpoint_index,
                                                   bool use_array_ind)
{
  for (const auto& point_ind : GetViewPointCoveredPointList(viewpoint_index, use_array_ind))
  {
    MY_ASSERT(misc_utils_ns::InRange<bool>(point_list, point_ind));
    point_list[point_ind] = true;
  }
}
/**
 * Iterates through list of covered frontier points for a viewpoint, and sets point list to true at covered point 
 * indices.
 * 
 * @param[out] point_list boolean list that is true at covered points the viewpoint covers.
 * @param viewpoint_ind index of viewpoint.
 * @param use_array_ind status indicating array index or regular index.
 */
void ViewPointManager::UpdateViewPointCoveredFrontierPoint(std::vector<bool>& frontier_point_list, int viewpoint_index,
                                                           bool use_array_ind)
{
  for (const auto& point_ind : GetViewPointCoveredFrontierPointList(viewpoint_index, use_array_ind))
  {
    MY_ASSERT(misc_utils_ns::InRange<bool>(frontier_point_list, point_ind));
    frontier_point_list[point_ind] = true;
  }
}

/**
 * Updates view point candidates into the viewpoint candidate cloud or the viewpoint collision cloud. 
 * Generates KD Trees for both clouds, and creates a graph out of the new candidate viewpoints.
 * 
 * @return number of new viewpoint candidates.
 */
int ViewPointManager::GetViewPointCandidate()
{
  viewpoint_candidate_cloud_->clear();
  viewpoint_in_collision_cloud_->clear();
  candidate_indices_.clear();
  // Populates viewpoint_candidate_cloud_ and viewpoint_in_collision_cloud_ depending on viewpoint status.
  for (int i = 0; i < vp_.kViewPointNumber; i++)
  {
    SetViewPointCandidate(i, false);
    // Point meets ideal viewpoint conditions, populate viewpoint_candidate_cloud_.
    if (!ViewPointInCollision(i) && ViewPointInLineOfSight(i) && ViewPointConnected(i))
    {
      SetViewPointCandidate(i, true);
      candidate_indices_.push_back(i);
      geometry_msgs::Point viewpoint_position = GetViewPointPosition(i);
      pcl::PointXYZI point;
      point.x = viewpoint_position.x;
      point.y = viewpoint_position.y;
      point.z = viewpoint_position.z;
      viewpoint_candidate_cloud_->points.push_back(point);
    }
    // Point is in collision, populate viewpoint_candidate_cloud_.
    // TODO: else if to reduce unnecessary check.
    if (ViewPointInCollision(i))
    {
      geometry_msgs::Point viewpoint_position = GetViewPointPosition(i);
      pcl::PointXYZI point;
      point.x = viewpoint_position.x;
      point.y = viewpoint_position.y;
      point.z = viewpoint_position.z;
      point.intensity = GetViewPointCollisionFrameCount(i);
      viewpoint_in_collision_cloud_->points.push_back(point);
    }
  }
  // std::cout << "candidate viewpoint num: " << candidate_indices_.size() << std::endl;
  if (!candidate_indices_.empty())
  {
    kdtree_viewpoint_candidate_->setInputCloud(viewpoint_candidate_cloud_);
  }

  if (!viewpoint_in_collision_cloud_->points.empty())
  {
    kdtree_viewpoint_in_collision_->setInputCloud(viewpoint_in_collision_cloud_);
  }

  // Construct a graph of all the viewpoints
  GetCandidateViewPointGraph(candidate_viewpoint_graph_, candidate_viewpoint_dist_, candidate_viewpoint_position_);

  return candidate_indices_.size();
}

/**
 * Uses A* Search to get the shortest path from starting viewpoint to target viewpoint.
 * 
 * @param start_viewpoint_ind index of starting viewpoint.
 * @param target_viewpoint_ind index of ending viewpoint.
 * @return path with poses of view point positions from start to target.
 */
nav_msgs::Path ViewPointManager::GetViewPointShortestPath(int start_viewpoint_ind, int target_viewpoint_ind)
{
  nav_msgs::Path path;
  if (!InRange(start_viewpoint_ind))
  {
    ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath start viewpoint ind: " << start_viewpoint_ind
                                                                                       << " not in range");
    return path;
  }
  if (!InRange(target_viewpoint_ind))
  {
    ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath target viewpoint ind: " << target_viewpoint_ind
                                                                                        << " not in range");
    return path;
  }

  int start_graph_ind = graph_index_map_[start_viewpoint_ind];
  int target_graph_ind = graph_index_map_[target_viewpoint_ind];

  std::vector<int> path_graph_indices;
  double path_length =
      misc_utils_ns::AStarSearch(candidate_viewpoint_graph_, candidate_viewpoint_dist_, candidate_viewpoint_position_,
                                 start_graph_ind, target_graph_ind, true, path_graph_indices);
  if (path_graph_indices.size() >= 2)
  {
    for (int i = 0; i < path_graph_indices.size(); i++)
    {
      int graph_idx = path_graph_indices[i];
      int ind = candidate_indices_[graph_idx];
      geometry_msgs::PoseStamped pose;
      pose.pose.position = GetViewPointPosition(ind);
      path.poses.push_back(pose);
    }
  }
  return path;
}

/**
 * Overloaded function that converts input parameters into indexes and conducts A* Search to get the shortest path from 
 * starting viewpoint to target viewpoint.
 * 
 * @param start_position start position.
 * @param target_position target position.
 * @return path with poses of view point positions from start to target.
 */
nav_msgs::Path ViewPointManager::GetViewPointShortestPath(const Eigen::Vector3d& start_position,
                                                          const Eigen::Vector3d& target_position)
{
  nav_msgs::Path path;
  if (!InLocalPlanningHorizon(start_position))
  {
    ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath start position " << start_position.transpose()
                                                                                 << " not in local planning horizon");
    return path;
  }
  if (!InLocalPlanningHorizon(target_position))
  {
    ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath target position " << target_position.transpose()
                                                                                  << " not in local planning horizon");
    return path;
  }
  int start_viewpoint_ind = GetNearestCandidateViewPointInd(start_position);
  int target_viewpoint_ind = GetNearestCandidateViewPointInd(target_position);

  return GetViewPointShortestPath(start_viewpoint_ind, target_viewpoint_ind);
}

/**
 * 
 * 
 * Start and target position must be within local planning horizon. Gets closest candidate viewpoint to both start and 
 * target position. Uses A* to search for a path within max path length constraint. If a valid path is found, push into 
 * output variable path.
 * 
 * @param start_position start position.
 * @param target_position target position.
 * @param max_path_length constraint for path length between start and target.
 * @param[out] path output path, populated when a valid path is found with A* between the start and target position.
 */
bool ViewPointManager::GetViewPointShortestPathWithMaxLength(const Eigen::Vector3d& start_position,
                                                             const Eigen::Vector3d& target_position,
                                                             double max_path_length, nav_msgs::Path& path)
{
  if (!InLocalPlanningHorizon(start_position))
  {
    ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath start position " << start_position.transpose()
                                                                                 << " not in local planning horizon");
    return false;
  }
  if (!InLocalPlanningHorizon(target_position))
  {
    ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath target position " << target_position.transpose()
                                                                                  << " not in local planning horizon");
    return false;
  }
  int start_viewpoint_ind = GetNearestCandidateViewPointInd(start_position);
  int target_viewpoint_ind = GetNearestCandidateViewPointInd(target_position);
  int start_graph_ind = graph_index_map_[start_viewpoint_ind];
  int target_graph_ind = graph_index_map_[target_viewpoint_ind];

  std::vector<int> path_graph_indices;
  double shortest_path_dist = 0;
  bool found_path = misc_utils_ns::AStarSearchWithMaxPathLength(
      candidate_viewpoint_graph_, candidate_viewpoint_dist_, candidate_viewpoint_position_, start_graph_ind,
      target_graph_ind, true, path_graph_indices, shortest_path_dist, max_path_length);

  if (found_path && path_graph_indices.size() >= 2)
  {
    for (int i = 0; i < path_graph_indices.size(); i++)
    {
      int graph_idx = path_graph_indices[i];
      int ind = candidate_indices_[graph_idx];
      geometry_msgs::PoseStamped pose;
      pose.pose.position = GetViewPointPosition(ind);
      path.poses.push_back(pose);
    }
  }
  return found_path;
}

/**
 * For all viewpoint candidates, update viewpoint in exploring cell status.
 * 
 * @param grid_world
 */
void ViewPointManager::UpdateCandidateViewPointCellStatus(std::unique_ptr<grid_world_ns::GridWorld> const& grid_world)
{
  for (const auto& ind : candidate_indices_)
  {
    int cell_ind = GetViewPointCellInd(ind);
    if (grid_world->IndInBound(cell_ind))
    {
      grid_world_ns::CellStatus cell_status = grid_world->GetCellStatus(cell_ind);
      if (cell_status == grid_world_ns::CellStatus::UNSEEN || cell_status == grid_world_ns::CellStatus::EXPLORING)
      {
        SetViewPointInExploringCell(ind, true);
      }
      else
      {
        SetViewPointInExploringCell(ind, false);
      }
    }
    else
    {
      ROS_WARN_STREAM("ViewPointManager::UpdateCandidateViewPointCellStatus: cell ind " << cell_ind << " out of bound");
    }
  }
}

/**
 * Constructs a candidate view point graph, while simultaneously updating distances and positions.
 * 
 * @param[out] graph candidate viewpoint graph.
 * @param[out] dist candidate viewpoints' distances from neighboring candidates.
 * @param[out] positions candidate viewpoints' positions.
 */
void ViewPointManager::GetCandidateViewPointGraph(std::vector<std::vector<int>>& graph,
                                                  std::vector<std::vector<double>>& dist,
                                                  std::vector<geometry_msgs::Point>& positions)
{
  graph.clear();
  dist.clear();
  positions.clear();
  if (candidate_indices_.empty())
  {
    return;
  }
  graph.resize(candidate_indices_.size());
  dist.resize(graph.size());

  for (int i = 0; i < candidate_indices_.size(); i++)
  {
    int ind = candidate_indices_[i];
    graph_index_map_[ind] = i;
  }

  // Build the graph
  for (int i = 0; i < candidate_indices_.size(); i++)
  {
    int cur_ind = candidate_indices_[i];
    positions.push_back(GetViewPointPosition(cur_ind));
    for (int j = 0; j < connected_neighbor_indices_[cur_ind].size(); j++)
    {
      int neighbor_ind = connected_neighbor_indices_[cur_ind][j];
      double neighbor_dist = connected_neighbor_dist_[cur_ind][j];
      if (IsViewPointCandidate(neighbor_ind))
      {
        graph[i].push_back(graph_index_map_[neighbor_ind]);
        dist[i].push_back(neighbor_dist);
      }
    }
  }
}

/**
 * Finds closest index to position. If index is a viewpoint candidate, return it. 
 * If not, iterate through eligible candidate viewpoints and return the closest candidate. 
 * If all fails, return -1.
 * 
 * @param position query position.
 * @return closest viewpoint candidate to query position.
 */
int ViewPointManager::GetNearestCandidateViewPointInd(const Eigen::Vector3d& position)
{
  int viewpoint_ind = GetViewPointInd(position);
  if (InRange(viewpoint_ind))
  {
    if (IsViewPointCandidate(viewpoint_ind))
    {
      return viewpoint_ind;
    }
  }
  if (!candidate_indices_.empty())
  {
    // Find the closest viewpoint that is a candidate viewpoint
    double min_dist = DBL_MAX;
    int min_dist_ind = -1;
    geometry_msgs::Point query_position;
    query_position.x = position.x();
    query_position.y = position.y();
    query_position.z = position.z();
    for (const auto& cur_viewpoint_ind : candidate_indices_)
    {
      geometry_msgs::Point cur_position = GetViewPointPosition(cur_viewpoint_ind);
      double dist =
          misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(cur_position, query_position);
      if (dist < min_dist)
      {
        min_dist = dist;
        min_dist_ind = cur_viewpoint_ind;
      }
    }
    return min_dist_ind;
  }
  else
  {
    std::cout << "Candidate viewpoint empty, can't find nearest candidate viewpoints to the position" << std::endl;
    return -1;
  }
}

/**
 * Populates cloud with viewpoints in collision.
 * 
 * @param[out] cloud output point cloud populated with viewpoints in collision.
 */
void ViewPointManager::GetCollisionViewPointVisCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  cloud->clear();
  for (const auto& point : viewpoint_in_collision_cloud_->points)
  {
    cloud->points.push_back(point);
  }
}

/**
 * Checks if position is in local planning horizon. 
 * 
 * Gets index of closest viewpoint to position. If index of closest viewpoint is within z range, and viewpoint is 
 * either a candidate, or in collision, viewpoint is considered to be within local planning horizon. 
 * 
 * @param position query position.
 * @return true if poistion is within local planning horizon, false otherwise.
 */
bool ViewPointManager::InLocalPlanningHorizon(const Eigen::Vector3d& position)
{
  int viewpoint_ind = GetViewPointInd(position);
  if (InRange(viewpoint_ind))
  {
    // TODO: this doesn't seem to change, would it be better to calculate once during initialization?
    double max_z_diff = std::max(vp_.kResolution.x(), vp_.kResolution.y()) * 2;
    geometry_msgs::Point viewpoint_position = GetViewPointPosition(viewpoint_ind);
    if (std::abs(viewpoint_position.z - position.z()) < max_z_diff &&
        (IsViewPointCandidate(viewpoint_ind) || ViewPointInCollision(viewpoint_ind)))
    {
      return true;
    }
  }
  return false;
}

}  // namespace viewpoint_manager_ns