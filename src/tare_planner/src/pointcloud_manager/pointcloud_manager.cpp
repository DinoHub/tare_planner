/**
 * @file pointcloud_manager.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a grid of pointclouds
 * @version 0.1
 * @date 2019-12-08
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "../../include/pointcloud_manager/pointcloud_manager.h"

namespace pointcloud_manager_ns
{
/**
 * Default constructor for the point cloud manager that initializes robot position, origin (bottom left of grid), 
 * indexes, a point cloud grid and an occupancy cloud grid, both containing a point cloud at each cell.
 */
PointCloudManager::PointCloudManager(int row_num, int col_num, int level_num, int max_cell_point_num, double cell_size,
                                     double cell_height, int neighbor_cell_num)
  : kRowNum(row_num)
  , kColNum(col_num)
  , kLevelNum(level_num)
  , kMaxCellPointNum(max_cell_point_num)
  , kCellSize(cell_size)
  , kCellHeight(cell_height)
  , kNeighborCellNum(neighbor_cell_num)
  , kCloudDwzFilterLeafSize(0.2)
  , initialized_(false)
{
  robot_position_.x = 0.0;
  robot_position_.y = 0.0;
  robot_position_.z = 0.0;

  origin_.x = robot_position_.x - (kCellSize * kRowNum) / 2;
  origin_.y = robot_position_.y - (kCellSize * kColNum) / 2;
  origin_.z = robot_position_.z - (kCellHeight * kLevelNum) / 2;

  cur_row_idx_ = kRowNum / 2;
  cur_col_idx_ = kColNum / 2;
  cur_level_idx_ = kLevelNum / 2;

  Eigen::Vector3i pointcloud_grid_size(kRowNum, kColNum, kLevelNum);
  Eigen::Vector3d pointcloud_grid_origin(origin_.x, origin_.y, origin_.z);
  Eigen::Vector3d pointcloud_grid_resolution(kCellSize, kCellSize, kCellHeight);
  PCLCloudTypePtr cloud_ptr_tmp;
  pointcloud_grid_ = std::make_unique<grid_ns::Grid<PCLCloudTypePtr>>(
      pointcloud_grid_size, cloud_ptr_tmp, pointcloud_grid_origin, pointcloud_grid_resolution, 3);

  pcl::PointCloud<pcl::PointXYZI>::Ptr occupancy_cloud_ptr_tmp;
  occupancy_cloud_grid_ = std::make_unique<grid_ns::Grid<pcl::PointCloud<pcl::PointXYZI>::Ptr>>(
      pointcloud_grid_size, occupancy_cloud_ptr_tmp, pointcloud_grid_origin, pointcloud_grid_resolution, 3);

  for (int i = 0; i < pointcloud_grid_->GetCellNumber(); i++)
  {
    pointcloud_grid_->GetCell(i) = PCLCloudTypePtr(new PCLCloudType);
    // pointcloud_grid_->GetCell(i)->points.reserve(kMaxCellPointNum);
  }

  for (int i = 0; i < occupancy_cloud_grid_->GetCellNumber(); i++)
  {
    occupancy_cloud_grid_->GetCell(i) = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  }

  cloud_dwz_filter_.setLeafSize(kCloudDwzFilterLeafSize, kCloudDwzFilterLeafSize, kCloudDwzFilterLeafSize);
  rolled_in_occupancy_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
}

/**
 * Based on current robot position, update origin in point cloud grid and occupancy cloud grid.
 */
void PointCloudManager::UpdateOrigin()
{
  origin_.x = robot_position_.x - (kCellSize * kRowNum) / 2;
  origin_.y = robot_position_.y - (kCellSize * kColNum) / 2;
  origin_.z = robot_position_.z - (kCellHeight * kLevelNum) / 2;
  pointcloud_grid_->SetOrigin(Eigen::Vector3d(origin_.x, origin_.y, origin_.z));
  occupancy_cloud_grid_->SetOrigin(Eigen::Vector3d(origin_.x, origin_.y, origin_.z));
}

/**
 * Updates robot position and checks if viewpoints contain rollover. 
 * 
 * Collects indices of neighbors based on current robot position. Viewpoint rollover is set to be true 
 * if there is a change in neighbor indices. New neighbor indices are populated with neighbor indices 
 * not found in previous run. Sets origin of neighbor cells based on new robot position.
 * 
 * @param robot_position current robot position.
 * @return true if viewpoints do rollover.
 */
bool PointCloudManager::UpdateRobotPosition(const geometry_msgs::Point& robot_position)
{
  robot_position_ = robot_position;
  if (!initialized_)
  {
    initialized_ = true;
    UpdateOrigin();
  }

  Eigen::Vector3i robot_cell_sub =
      pointcloud_grid_->Pos2Sub(Eigen::Vector3d(robot_position_.x, robot_position_.y, robot_position_.z));

  // Get neighbor indices
  prev_neighbor_indices_ = neighbor_indices_;
  neighbor_indices_.clear();
  int row_idx;
  int col_idx;
  int level_idx;
  int N = kNeighborCellNum / 2;
  for (int i = -N; i <= N; i++)
  {
    for (int j = -N; j <= N; j++)
    {
      for (int k = -N; k <= N; k++)
      {
        Eigen::Vector3i neighbor_sub;
        neighbor_sub.x() = robot_cell_sub.x() + i;
        neighbor_sub.y() = robot_cell_sub.y() + j;
        neighbor_sub.z() = robot_cell_sub.z() + k;
        if (pointcloud_grid_->InRange(neighbor_sub))
        {
          int ind = pointcloud_grid_->Sub2Ind(neighbor_sub);
          neighbor_indices_.push_back(ind);
        }
      }
    }
  }

  std::vector<int> indices_diff;
  misc_utils_ns::SetDifference(neighbor_indices_, prev_neighbor_indices_, indices_diff);
  bool rolling = false;
  if (!indices_diff.empty())
  {
    new_neighbor_indices_ = indices_diff;
    rolling = true;
  }

  Eigen::Vector3i neighbor_cell_min_sub = robot_cell_sub - Eigen::Vector3i(N, N, N);
  neighbor_cells_origin_ =
      pointcloud_grid_->Sub2Pos(neighbor_cell_min_sub) - Eigen::Vector3d(kCellSize / 2, kCellSize / 2, kCellHeight / 2);

  return rolling;
}

/**
 * Collects neighbors from pointcloud grid into output point cloud.
 * 
 * @param[out] cloud_out output point cloud.
 */
void PointCloudManager::GetPointCloud(PCLCloudType& cloud_out)
{
  cloud_out.clear();
  for (const auto& neighbor_ind : neighbor_indices_)
  {
    cloud_out += *(pointcloud_grid_->GetCell(neighbor_ind));
  }
}

/**
 * Remove neighbors from occupancy cloud grid.
 */
void PointCloudManager::ClearNeighborCellOccupancyCloud()
{
  for (const auto& neighbor_ind : neighbor_indices_)
  {
    occupancy_cloud_grid_->GetCell(neighbor_ind)->clear();
  }
}

/**
 * Rolled in occupancy cloud refers to the occupancy cloud that has been "rolled into" the current grid.
 * 
 * Resets rolled in cloud, gets indices of new neighbors, queries the occupancy cloud grid for the new neighbors, and 
 * adds them to the rolled in occupancy cloud.  
 * 
 * @return pointer to rolled in occupancy cloud, which contains occupancy cloud cells of new neighbors.
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudManager::GetRolledInOccupancyCloud()
{
  rolled_in_occupancy_cloud_->clear();
  for (const auto& ind : new_neighbor_indices_)
  {
    *rolled_in_occupancy_cloud_ += *(occupancy_cloud_grid_->GetCell(ind));
  }
  return rolled_in_occupancy_cloud_;
}

/**
 * Clear occupancy cloud, iterate through occupancy cloud grid, and add each cell of occupancy cloud grid into the 
 * occupancy cloud argument.
 * 
 * @param[out] occupancy_cloud cloud to populate with current occupancy cloud grids.
 */
void PointCloudManager::GetOccupancyCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& occupancy_cloud)
{
  occupancy_cloud->clear();
  int cell_num = occupancy_cloud_grid_->GetCellNumber();
  for (int i = 0; i < cell_num; i++)
  {
    *occupancy_cloud += *(occupancy_cloud_grid_->GetCell(i));
  }
}

/**
 * Iterates through all points in the incoming occupancy cloud, converts the point into an index, and pushes point back 
 * into the corresponding occupancy cloud grid's cell.
 * 
 * @param occupancy_cloud incoming occupancy cloud.
 */
void PointCloudManager::StoreOccupancyCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& occupancy_cloud)
{
  for (const auto& point : occupancy_cloud->points)
  {
    Eigen::Vector3i sub = occupancy_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
    if (!occupancy_cloud_grid_->InRange(sub))
    {
      continue;
    }
    int ind = occupancy_cloud_grid_->Sub2Ind(sub);
    occupancy_cloud_grid_->GetCell(ind)->points.push_back(point);
  }
}

/**
 * Iterates through all cells, and pushes the center of each cell in red into the output marker.
 * Populates all neighbor cells with green.
 * 
 * @param[out] marker output marker.
 */
void PointCloudManager::GetMarker(visualization_msgs::Marker& marker)
{
  marker.points.clear();
  marker.colors.clear();
  marker.scale.x = kCellSize;
  marker.scale.y = kCellSize;
  marker.scale.z = kCellHeight;

  for (int i = 0; i < kRowNum; i++)
  {
    for (int j = 0; j < kColNum; j++)
    {
      for (int k = 0; k < kLevelNum; k++)
      {
        geometry_msgs::Point cell_center;
        cell_center.x = i * kCellSize + kCellSize / 2 + origin_.x;
        cell_center.y = j * kCellSize + kCellSize / 2 + origin_.y;
        cell_center.z = k * kCellHeight + kCellHeight / 2 + origin_.z;
        marker.points.push_back(cell_center);
        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 1.0;
        marker.colors.push_back(color);
      }
    }
  }
  for (const auto& ind : neighbor_indices_)
  {
    marker.colors[ind].r = 0.0;
    marker.colors[ind].g = 1.0;
    marker.colors[ind].b = 0.0;
  }
}

/**
 * Gets all neighbors and push their positions, with intensity set to index value, into the visualization cloud.
 * 
 * @param[out] vis_cloud output visualization cloud.
 */
void PointCloudManager::GetVisualizationPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud)
{
  vis_cloud->clear();
  for (const auto& ind : new_neighbor_indices_)
  {
    Eigen::Vector3d position = pointcloud_grid_->Ind2Pos(ind);
    pcl::PointXYZI point;
    point.x = position.x();
    point.y = position.y();
    point.z = position.z();
    point.intensity = ind;
    vis_cloud->points.push_back(point);
  }
}

/**
 * Iterates through neighbor indices, and updates total number of points. If index is less than  the number of points, 
 * update cloud index to current neighbor index, and cloud point index to the query index - number of points within the 
 * current neighbor index cell.
 * 
 * @param index query index.
 * @param[out] cloud_index
 * @param[out] cloud_point_index
 */
void PointCloudManager::GetCloudPointIndex(int index, int& cloud_index, int& cloud_point_index)
{
  cloud_index = -1;
  cloud_point_index = -1;
  int point_num = 0;
  for (int i = 0; i < neighbor_indices_.size(); i++)
  {
    int ind = neighbor_indices_[i];
    point_num += pointcloud_grid_->GetCell(ind)->points.size();

    if (index < point_num)
    {
      cloud_index = ind;
      int prev_point_num = point_num - pointcloud_grid_->GetCell(ind)->points.size();
      cloud_point_index = index - prev_point_num;
      break;
    }
  }
  // TODO: seems like debugging, remove?
  if (index > point_num || cloud_index == -1 || cloud_point_index == -1)
  {
    std::cout << "index: " << index << " point num: " << point_num << std::endl;
    for (int i = 0; i < neighbor_indices_.size(); i++)
    {
      int ind = neighbor_indices_[i];
      std::cout << "cloud " << ind << " size: " << pointcloud_grid_->GetCell(ind)->points.size() << std::endl;
    }
  }
}
/**
 * Counts the number of points within the pointcloud grid.
 * 
 * @return number of points within pointcloud grid.
 */
int PointCloudManager::GetAllPointNum()
{
  int num = 0;
  for (int i = 0; i < pointcloud_grid_->GetCellNumber(); ++i)
  {
    num += pointcloud_grid_->GetCell(i)->points.size();
  }
  return num;
}

/**
 * Sets all points within the point cloud grid to be red.
 */
void PointCloudManager::UpdateOldCloudPoints()
{
  for (int i = 0; i < pointcloud_grid_->GetCellNumber(); ++i)
  {
    int point_num = pointcloud_grid_->GetCell(i)->points.size();
    for (int j = 0; j < point_num; ++j)
    {
      pointcloud_grid_->GetCell(i)->points[j].r = 255;
    }
  }
}

/**
 * Iterates through all points within the pointcloud grid and sets 
 * their green values to 255 as long as it is greater than 0.
 * 
 * A green value of 255 indicates the point is covered.
 */
void PointCloudManager::UpdateCoveredCloudPoints()
{
  for (int i = 0; i < pointcloud_grid_->GetCellNumber(); ++i)
  {
    int point_num = pointcloud_grid_->GetCell(i)->points.size();
    for (int j = 0; j < point_num; ++j)
    {
      if (pointcloud_grid_->GetCell(i)->points[j].g > 0)
      {
        pointcloud_grid_->GetCell(i)->points[j].g = 255;
      }
    }
  }
}

/**
 * Iterates through a specific point within the pointcloud grid and sets 
 * its value to 255.
 * 
 * A green value of 255 indicates the point is covered.
 */
void PointCloudManager::UpdateCoveredCloudPoints(int cloud_index, int point_index)
{
  int cloud_num = pointcloud_grid_->GetCellNumber();
  MY_ASSERT(cloud_index >= 0 && cloud_index < cloud_num);
  int point_num = pointcloud_grid_->GetCell(cloud_index)->points.size();
  MY_ASSERT(point_index >= 0 && point_index < point_num);
  pointcloud_grid_->GetCell(cloud_index)->points[point_index].g = 255;
}

}  // namespace pointcloud_manager_ns
