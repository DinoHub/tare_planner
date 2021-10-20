/**
 * @file viewpoint.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a viewpoint
 * @version 0.1
 * @date 2019-11-04
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "viewpoint/viewpoint.h"

namespace viewpoint_ns
{
/**
 * Default constructor for viewpoints.
 * 
 * @param x x value for LiDAR model.
 * @param y y value for LiDAR model.
 * @param z z value for LiDAR model.
 */
ViewPoint::ViewPoint(double x, double y, double z)
  : lidar_model_(x, y, z)
  , in_collision_(false)
  , in_line_of_sight_(false)
  , connected_(false)
  , visited_(false)
  , selected_(false)
  , is_candidate_(false)
  , has_terrain_height_(false)
  , has_terrain_neighbor_(false)
  , in_exploring_cell_(false)
  , cell_ind_(-1)
  , collision_frame_count_(0)
  , terrain_height_(0.0)
{
}

/**
 * Calls default constructor with point information extrapolated into default constructor arguments.
 * 
 * @param position geometry_msg containing point (x, y, z) information.
 */
ViewPoint::ViewPoint(const geometry_msgs::Point& position) : ViewPoint(position.x, position.y, position.z)
{
}

/**
 * Reset everything.
 */
void ViewPoint::Reset()
{
  in_collision_ = false;
  in_line_of_sight_ = false;
  connected_ = false;
  visited_ = false;
  selected_ = false;
  is_candidate_ = false;
  has_terrain_height_ = false;
  has_terrain_neighbor_ = false;
  in_exploring_cell_ = false;
  cell_ind_ = -1;
  lidar_model_.ResetCoverage();
  covered_point_list_.clear();
  covered_frontier_point_list_.clear();
  collision_frame_count_ = 0;
  terrain_height_ = 0.0;
}

/**
 * Only reset coverage on the LiDAR model, and remove all covered (frontier) point lists.
 */
void ViewPoint::ResetCoverage()
{
  lidar_model_.ResetCoverage();
  covered_point_list_.clear();
  covered_frontier_point_list_.clear();
}
}  // namespace viewpoint_ns