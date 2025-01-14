/**
 * @file planning_env.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that manages the world representation using point clouds
 * @version 0.1
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <planning_env/planning_env.h>
#include <viewpoint_manager/viewpoint_manager.h>

namespace planning_env_ns
{
/**
 * Reads parameters from ROS parameter server.
 * 
 * @param nh main ROS node's handle.
 */
void PlanningEnvParameters::ReadParameters(ros::NodeHandle& nh)
{
  kSurfaceCloudDwzLeafSize = misc_utils_ns::getParam<double>(nh, "kSurfaceCloudDwzLeafSize", 0.2);
  kCollisionCloudDwzLeafSize = misc_utils_ns::getParam<double>(nh, "kCollisionCloudDwzLeafSize", 0.2);
  kKeyposeGraphCollisionCheckRadius =
      misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddEdgeCollisionCheckRadius", 0.4);
  kKeyposeGraphCollisionCheckPointNumThr =
      misc_utils_ns::getParam<int>(nh, "keypose_graph/kAddEdgeCollisionCheckPointNumThr", 1);

  kKeyposeCloudStackNum = misc_utils_ns::getParam<int>(nh, "kKeyposeCloudStackNum", 5);

  kPointCloudRowNum = misc_utils_ns::getParam<int>(nh, "kPointCloudRowNum", 20);
  kPointCloudColNum = misc_utils_ns::getParam<int>(nh, "kPointCloudColNum", 20);
  kPointCloudLevelNum = misc_utils_ns::getParam<int>(nh, "kPointCloudLevelNum", 10);
  kMaxCellPointNum = misc_utils_ns::getParam<int>(nh, "kMaxCellPointNum", 100000);
  kPointCloudCellSize = misc_utils_ns::getParam<double>(nh, "kPointCloudCellSize", 24.0);
  kPointCloudCellHeight = misc_utils_ns::getParam<double>(nh, "kPointCloudCellHeight", 3.0);
  kPointCloudManagerNeighborCellNum = misc_utils_ns::getParam<int>(nh, "kPointCloudManagerNeighborCellNum", 5);
  kCoverCloudZSqueezeRatio = misc_utils_ns::getParam<double>(nh, "kCoverCloudZSqueezeRatio", 2.0);

  kUseFrontier = misc_utils_ns::getParam<bool>(nh, "kUseFrontier", false);
  kFrontierClusterTolerance = misc_utils_ns::getParam<double>(nh, "kFrontierClusterTolerance", 1.0);
  kFrontierClusterMinSize = misc_utils_ns::getParam<int>(nh, "kFrontierClusterMinSize", 30);

  kUseCoverageBoundaryOnFrontier = misc_utils_ns::getParam<bool>(nh, "kUseCoverageBoundaryOnFrontier", false);
  kUseCoverageBoundaryOnObjectSurface = misc_utils_ns::getParam<bool>(nh, "kUseCoverageBoundaryOnObjectSurface", false);

  int viewpoint_number = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/number_x", 40);
  double viewpoint_resolution = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/resolution_x", 1.0);
  double local_planning_horizon_half_size = viewpoint_number * viewpoint_resolution / 2;
  double sensor_range = misc_utils_ns::getParam<double>(nh, "kSensorRange", 15);

  kExtractFrontierRange.x() = local_planning_horizon_half_size + sensor_range * 2;
  kExtractFrontierRange.y() = local_planning_horizon_half_size + sensor_range * 2;
  kExtractFrontierRange.z() = 2;
}

/**
 * Initializes cloud stacks and point clouds. 
 * 
 * Initializes the pointcloud_manager_ with its necessary parameters, rolling_occupancy_grid_, and necessary 
 * vertical surface and frontier extractors.
 */
PlanningEnv::PlanningEnv(ros::NodeHandle nh, ros::NodeHandle nh_private, std::string world_frame_id)
  : keypose_cloud_count_(0)
  , vertical_surface_extractor_()
  , vertical_frontier_extractor_()
  , robot_position_update_(false)
{
  parameters_.ReadParameters(nh_private);
  keypose_cloud_stack_.resize(parameters_.kKeyposeCloudStackNum);
  for (int i = 0; i < keypose_cloud_stack_.size(); i++)
  {
    keypose_cloud_stack_[i].reset(new pcl::PointCloud<PlannerCloudPointType>());
  }

  vertical_surface_cloud_stack_.resize(parameters_.kKeyposeCloudStackNum);
  for (int i = 0; i < vertical_surface_cloud_stack_.size(); i++)
  {
    vertical_surface_cloud_stack_[i].reset(new pcl::PointCloud<PlannerCloudPointType>());
  }
  keypose_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "keypose_cloud", world_frame_id);
  stacked_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "stacked_cloud", world_frame_id);
  stacked_vertical_surface_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(
      nh, "stacked_vertical_surface_cloud", world_frame_id);

  stacked_vertical_surface_cloud_kdtree_ =
      pcl::KdTreeFLANN<PlannerCloudPointType>::Ptr(new pcl::KdTreeFLANN<PlannerCloudPointType>());
  vertical_surface_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "coverage_cloud", world_frame_id);

  diff_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "diff_cloud", world_frame_id);

  collision_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

  terrain_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "terrain_cloud", world_frame_id);

  planner_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "planner_cloud", world_frame_id);
  pointcloud_manager_ = std::make_unique<pointcloud_manager_ns::PointCloudManager>(
      parameters_.kPointCloudRowNum, parameters_.kPointCloudColNum, parameters_.kPointCloudLevelNum,
      parameters_.kMaxCellPointNum, parameters_.kPointCloudCellSize, parameters_.kPointCloudCellHeight,
      parameters_.kPointCloudManagerNeighborCellNum);
  pointcloud_manager_->SetCloudDwzFilterLeafSize() = parameters_.kSurfaceCloudDwzLeafSize;

  rolling_occupancy_grid_ = std::make_unique<rolling_occupancy_grid_ns::RollingOccupancyGrid>(nh_private);

  squeezed_planner_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(
      nh, "squeezed_planner_cloud", world_frame_id);
  squeezed_planner_cloud_kdtree_ =
      pcl::KdTreeFLANN<PlannerCloudPointType>::Ptr(new pcl::KdTreeFLANN<PlannerCloudPointType>());

  uncovered_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "uncovered_cloud", world_frame_id);
  uncovered_frontier_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "uncovered_frontier_cloud", world_frame_id);
  frontier_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "frontier_cloud", world_frame_id);
  filtered_frontier_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "filtered_frontier_cloud", world_frame_id);
  occupied_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "occupied_cloud", world_frame_id);
  free_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "free_cloud", world_frame_id);
  unknown_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "unknown_cloud", world_frame_id);

  rolling_occupancy_grid_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "rolling_occupancy_grid_cloud", world_frame_id);

  rolling_frontier_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "rolling_frontier_cloud", world_frame_id);

  rolling_filtered_frontier_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "rolling_filtered_frontier_cloud", world_frame_id);

  rolled_in_occupancy_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "rolled_in_occupancy_cloud", world_frame_id);
  rolled_out_occupancy_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "rolled_out_occupancy_cloud", world_frame_id);

  pointcloud_manager_occupancy_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "pointcloud_manager_occupancy_cloud_", world_frame_id);

  kdtree_frontier_cloud_ = pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>);
  kdtree_rolling_frontier_cloud_ = pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>);

  // Todo: parameterize
  vertical_surface_extractor_.SetRadiusThreshold(0.2);
  vertical_surface_extractor_.SetZDiffMax(2.0);
  vertical_surface_extractor_.SetZDiffMin(parameters_.kSurfaceCloudDwzLeafSize);
  vertical_frontier_extractor_.SetNeighborThreshold(2);

  Eigen::Vector3d rolling_occupancy_grid_resolution = rolling_occupancy_grid_->GetResolution();
  double vertical_frontier_neighbor_search_radius =
      std::max(rolling_occupancy_grid_resolution.x(), rolling_occupancy_grid_resolution.y());
  vertical_frontier_neighbor_search_radius =
      std::max(vertical_frontier_neighbor_search_radius, rolling_occupancy_grid_resolution.z());
  vertical_frontier_extractor_.SetRadiusThreshold(vertical_frontier_neighbor_search_radius);
  double z_diff_max = vertical_frontier_neighbor_search_radius * 5;
  double z_diff_min = vertical_frontier_neighbor_search_radius;
  vertical_frontier_extractor_.SetZDiffMax(z_diff_max);
  vertical_frontier_extractor_.SetZDiffMin(z_diff_min);
  vertical_frontier_extractor_.SetNeighborThreshold(2);
}

/**
 * Updates collision cloud by resetting the collision_cloud_, adding all vertical surface clouds within the 
 * vertical_surface_cloud_stack_ into the collision_cloud_, then downsizing it.
 */
void PlanningEnv::UpdateCollisionCloud()
{
  collision_cloud_->clear();
  for (int i = 0; i < parameters_.kKeyposeCloudStackNum; i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::copyPointCloud<PlannerCloudPointType, pcl::PointXYZI>(*vertical_surface_cloud_stack_[i], *cloud_tmp);
    *(collision_cloud_) += *cloud_tmp;
  }
  collision_cloud_downsizer_.Downsize(collision_cloud_, parameters_.kCollisionCloudDwzLeafSize,
                                      parameters_.kCollisionCloudDwzLeafSize, parameters_.kCollisionCloudDwzLeafSize);
}

/**
 * Publishes the filtered_frontier_cloud_.
 * 
 * Gets frontier cloud from the rolling_occupancy_grid_. Limits cloud to within coverage boundary, and extracts vertical 
 * surfaces into the filtered_frontier_cloud_ through the vertical_frontier_extractor_. Clusters the frontiers, and 
 * extracts relevant indices into inliers if number of clusters exceed kFrontierClusterMinSize.
 */
void PlanningEnv::UpdateFrontiers()
{
  if (parameters_.kUseFrontier)
  {
    prev_robot_position_ = robot_position_;
    // Populates frontier cloud using the occupancy grid.
    rolling_occupancy_grid_->GetFrontier(frontier_cloud_->cloud_, robot_position_, parameters_.kExtractFrontierRange);

    if (!frontier_cloud_->cloud_->points.empty())
    {
      if (parameters_.kUseCoverageBoundaryOnFrontier)
      {
        // Ensures cloud is within coverage boundary.
        GetCoverageCloudWithinBoundary<pcl::PointXYZI>(frontier_cloud_->cloud_);
      }
      // Extracts vertical surfaces into filtered frontier cloud.
      vertical_frontier_extractor_.ExtractVerticalSurface<pcl::PointXYZI, pcl::PointXYZI>(
          frontier_cloud_->cloud_, filtered_frontier_cloud_->cloud_);
    }

    // Cluster frontiers
    if (!filtered_frontier_cloud_->cloud_->points.empty())
    {
      kdtree_frontier_cloud_->setInputCloud(filtered_frontier_cloud_->cloud_);
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
      // Using EuclideanClusterExtraction to extract indices into cluster_indices.
      ec.setClusterTolerance(parameters_.kFrontierClusterTolerance);
      ec.setMinClusterSize(1);
      ec.setMaxClusterSize(10000);
      ec.setSearchMethod(kdtree_frontier_cloud_);
      ec.setInputCloud(filtered_frontier_cloud_->cloud_);
      ec.extract(cluster_indices);

      pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
      int cluster_count = 0;
      // For all cluster indices, if there number of frontier clusters exceed threshold, add into inliers.
      for (int i = 0; i < cluster_indices.size(); i++)
      {
        if (cluster_indices[i].indices.size() < parameters_.kFrontierClusterMinSize)
        {
          continue;
        }
        for (int j = 0; j < cluster_indices[i].indices.size(); j++)
        {
          int point_ind = cluster_indices[i].indices[j];
          filtered_frontier_cloud_->cloud_->points[point_ind].intensity = cluster_count;
          inliers->indices.push_back(point_ind);
        }
        cluster_count++;
      }
      // Uses ExtractIndices to extract inlier indices from the filtered frontier cloud, then publishes.
      pcl::ExtractIndices<pcl::PointXYZI> extract;
      extract.setInputCloud(filtered_frontier_cloud_->cloud_);
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter(*(filtered_frontier_cloud_->cloud_));
      filtered_frontier_cloud_->Publish();
    }
  }
}

/**
 * Copies input cloud into terrain_cloud_.
 * 
 * @param cloud cloud to be copied into terrain_cloud_.
 */
void PlanningEnv::UpdateTerrainCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  if (cloud->points.empty())
  {
    ROS_WARN("Terrain cloud empty");
  }
  else
  {
    terrain_cloud_->cloud_ = cloud;
  }
}

/**
 * Given an input point (x,y,z), checks if it is in collision.
 * 
 * Conducts radius search of stacked_vertical_surface_kdtree_ within kKeyposeGraphCollisionCheckRadius. If the number 
 * of neighbors exceed a threshold, consider the point to be in collision.
 * 
 * @param x point's x-value
 * @param y point's y-value
 * @param z point's z-value
 * @return collision status
 */
bool PlanningEnv::InCollision(double x, double y, double z) const
{
  if (stacked_cloud_->cloud_->points.empty())
  {
    ROS_WARN("PlanningEnv::InCollision(): collision cloud empty, not checking collision");
    return false;
  }
  PlannerCloudPointType check_point;
  check_point.x = x;
  check_point.y = y;
  check_point.z = z;
  std::vector<int> neighbor_indices;
  std::vector<float> neighbor_sqdist;
  stacked_vertical_surface_cloud_kdtree_->radiusSearch(check_point, parameters_.kKeyposeGraphCollisionCheckRadius,
                                                       neighbor_indices, neighbor_sqdist);
  if (neighbor_indices.size() > parameters_.kKeyposeGraphCollisionCheckPointNumThr)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * Given the robot's viewpoint and viewpoint_manager, iterate through planner_cloud_ and change all covered point 
 * colors to green within the planner cloud.
 * 
 * Iterates through planner_cloud_. For each point, checks if point is within vertical FOV and satisfies equation 2. 
 * 
 * If the point doesn't meet above criteria, the algorithm iterates through ViewPoint candidates 
 * and checks if the ViewPoint can see the point - if visible, point is considered covered.
 * 
 * All covered points have their colors changed to green and are added to covered_point_indices.
 * 
 * Creates a squeezed_planner_cloud_, dilated by a ratio in the z-axis, and converts it into 
 * squeezed_planner_cloud_kdtree_. Searches squeezed_planner_cloud_kdtree_ within coverage_dilation_radius to collect 
 * indices of points near covered points and mark them as covered as well (by changing color to green).
 * 
 * Finally, iterates through all points within the planner cloud, and updates all points that are covered 
 * within the pointcloud manager.
 * 
 * @param robot_viewpoint manages what a robot is able to perceive.
 * @param viewpoint_manager manages viewpoints within the environment.
 */
void PlanningEnv::UpdateCoveredArea(const lidar_model_ns::LiDARModel& robot_viewpoint,
                                    const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager)
{
  if (planner_cloud_->cloud_->points.empty())
  {
    std::cout << "Planning cloud empty, cannot update covered area" << std::endl;
    return;
  }
  geometry_msgs::Point robot_position = robot_viewpoint.getPosition();
  double sensor_range = viewpoint_manager->GetSensorRange();
  double coverage_occlusion_thr = viewpoint_manager->GetCoverageOcclusionThr();
  double coverage_dilation_radius = viewpoint_manager->GetCoverageDilationRadius();
  std::vector<int> covered_point_indices;
  double vertical_fov_ratio = 0.3;  // bigger fov than viewpoints
  double diff_z_max = sensor_range * vertical_fov_ratio;
  double xy_dist_threshold = 3 * (parameters_.kSurfaceCloudDwzLeafSize / 2) / 0.3;
  double z_diff_threshold = 3 * parameters_.kSurfaceCloudDwzLeafSize;
  for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[i];
    if (point.g > 0)
    {
      planner_cloud_->cloud_->points[i].g = 255;
      continue;
    }
    if (std::abs(point.z - robot_position.z) < diff_z_max)
    {
      if (misc_utils_ns::InFOVSimple(Eigen::Vector3d(point.x, point.y, point.z),
                                     Eigen::Vector3d(robot_position.x, robot_position.y, robot_position.z),
                                     vertical_fov_ratio, sensor_range, xy_dist_threshold, z_diff_threshold))
      {
        if (robot_viewpoint.CheckVisibility<PlannerCloudPointType>(point, coverage_occlusion_thr))
        {
          planner_cloud_->cloud_->points[i].g = 255;
          covered_point_indices.push_back(i);
          continue;
        }
      }
    }
    // mark covered by visited viewpoints
    for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
    {
      if (viewpoint_manager->ViewPointVisited(viewpoint_ind))
      {
        if (viewpoint_manager->VisibleByViewPoint<PlannerCloudPointType>(point, viewpoint_ind))
        {
          planner_cloud_->cloud_->points[i].g = 255;
          covered_point_indices.push_back(i);
          break;
        }
      }
    }
  }

  // Dilate the covered area
  squeezed_planner_cloud_->cloud_->clear();
  for (const auto& point : planner_cloud_->cloud_->points)
  {
    PlannerCloudPointType squeezed_point = point;
    squeezed_point.z = point.z / parameters_.kCoverCloudZSqueezeRatio;
    squeezed_planner_cloud_->cloud_->points.push_back(squeezed_point);
  }
  squeezed_planner_cloud_kdtree_->setInputCloud(squeezed_planner_cloud_->cloud_);

  for (const auto& ind : covered_point_indices)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[ind];
    std::vector<int> nearby_indices;
    // TODO: nearby_sqdist is unused.
    std::vector<float> nearby_sqdist;
    squeezed_planner_cloud_kdtree_->radiusSearch(point, coverage_dilation_radius, nearby_indices, nearby_sqdist);
    if (!nearby_indices.empty())
    {
      for (const auto& idx : nearby_indices)
      {
        MY_ASSERT(idx >= 0 && idx < planner_cloud_->cloud_->points.size());
        planner_cloud_->cloud_->points[idx].g = 255;
      }
    }
  }

  for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[i];
    if (point.g > 0)
    {
      int cloud_idx = 0;
      int cloud_point_idx = 0;
      pointcloud_manager_->GetCloudPointIndex(i, cloud_idx, cloud_point_idx);
      pointcloud_manager_->UpdateCoveredCloudPoints(cloud_idx, cloud_point_idx);
    }
  }
}

/**
 * This function looks for uncovered points - points that are coverable by viewpoints, but have yet to be 
 * covered, maintains a tally of them, and adds them to the uncovered_cloud_ and uncovered_frontier_cloud_.
 * 
 * Iterates through all points within the planner_cloud_. For each point, iterate through all viewpoint 
 * candidates. If the viewpoint candidate has not been visited, and the current point is visible by the 
 * viewpoint, add uncovered point number to viewpoint's covered point list and push point to uncovered_cloud.
 * 
 * Does the same for frontier points, if kUseFrontier is true.
 * 
 * @param viewpoint_manager reference to viewpoint_manager for assessing viewpoint information.
 * @param[out] uncovered_point_num tally of uncovered points actually coverable by viewpoint.
 * @param[out] uncovered_frontier_point_num tally of uncovered frontier points actually coverable by viewpoint.
 */
void PlanningEnv::GetUncoveredArea(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                                   int& uncovered_point_num, int& uncovered_frontier_point_num)
{
  // Clear viewpoint covered point list
  for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
  {
    viewpoint_manager->ResetViewPointCoveredPointList(viewpoint_ind);
  }

  // Get uncovered points
  uncovered_cloud_->cloud_->clear();
  uncovered_frontier_cloud_->cloud_->clear();
  uncovered_point_num = 0;
  uncovered_frontier_point_num = 0;
  for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[i];
    if (point.g > 0)
    {
      continue;
    }
    bool observed = false;
    for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
    {
      if (!viewpoint_manager->ViewPointVisited(viewpoint_ind))
      {
        if (viewpoint_manager->VisibleByViewPoint<PlannerCloudPointType>(point, viewpoint_ind))
        {
          viewpoint_manager->AddUncoveredPoint(viewpoint_ind, uncovered_point_num);
          observed = true;
        }
      }
    }
    if (observed)
    {
      pcl::PointXYZI uncovered_point;
      uncovered_point.x = point.x;
      uncovered_point.y = point.y;
      uncovered_point.z = point.z;
      uncovered_point.intensity = i;
      uncovered_cloud_->cloud_->points.push_back(uncovered_point);
      uncovered_point_num++;
    }
  }

  // Check uncovered frontiers
  if (parameters_.kUseFrontier)
  {
    for (int i = 0; i < filtered_frontier_cloud_->cloud_->points.size(); i++)
    {
      pcl::PointXYZI point = filtered_frontier_cloud_->cloud_->points[i];
      bool observed = false;
      for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
      {
        if (!viewpoint_manager->ViewPointVisited(viewpoint_ind))
        {
          if (viewpoint_manager->VisibleByViewPoint<pcl::PointXYZI>(point, viewpoint_ind))
          {
            viewpoint_manager->AddUncoveredFrontierPoint(viewpoint_ind, uncovered_frontier_point_num);
            observed = true;
          }
        }
      }
      if (observed)
      {
        pcl::PointXYZI uncovered_frontier_point;
        uncovered_frontier_point.x = point.x;
        uncovered_frontier_point.y = point.y;
        uncovered_frontier_point.z = point.z;
        uncovered_frontier_point.intensity = i;
        uncovered_frontier_cloud_->cloud_->points.push_back(uncovered_frontier_point);
        uncovered_frontier_point_num++;
      }
    }
  }
}

void PlanningEnv::GetVisualizationPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud)
{
  pointcloud_manager_->GetVisualizationPointCloud(vis_cloud);
}

void PlanningEnv::PublishStackedCloud()
{
  stacked_cloud_->Publish();
}

void PlanningEnv::PublishUncoveredCloud()
{
  uncovered_cloud_->Publish();
}

void PlanningEnv::PublishUncoveredFrontierCloud()
{
  uncovered_frontier_cloud_->Publish();
}

}  // namespace planning_env_ns