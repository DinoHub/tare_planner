/**
 * @file lidar_model.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements the sensor model of a LiDAR
 * @version 0.1
 * @date 2019-09-26
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <string>
#include <vector>
#include <cmath>
// ROS
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <utils/misc_utils.h>

namespace lidar_model_ns
{
class LiDARModel
{
public:
  static double pointcloud_resolution_;
  explicit LiDARModel(double px = 0.0, double py = 0.0, double pz = 0.0, double rw = 1.0, double rx = 0.0,
                      double ry = 0.0, double rz = 0.0);
  explicit LiDARModel(const geometry_msgs::Pose& pose);
  ~LiDARModel() = default;

  /**
   * @brief Updates viewpoint coverage of robot, from LiDAR point of view.
   * 
   * Iterates through neighbors of the point, and updates covered voxel with distance to point if current distance to 
   * point is less than previous distance to point, or if reset was set to true for that particular point. Changes 
   * reset status to false for covered points.
   * 
   * @tparam PointType type of point.
   * @param point point to update coverage with.
   */
  template <class PointType>
  void UpdateCoverage(const PointType& point)
  {
    double distance_to_point = misc_utils_ns::PointXYZDist<PointType, geometry_msgs::Point>(point, pose_.position);

    if (isZero(distance_to_point))
      return;

    double dx = point.x - pose_.position.x;
    double dy = point.y - pose_.position.y;
    double dz = point.z - pose_.position.z;

    int horizontal_angle = GetHorizontalAngle(dx, dy);
    int vertical_angle = GetVerticalAngle(dz, distance_to_point);

    int horizontal_neighbor_num = GetHorizontalNeighborNum(distance_to_point);
    int vertical_neighbor_num = GetVerticalNeighborNum(distance_to_point);

    for (int n = -horizontal_neighbor_num; n <= horizontal_neighbor_num; n++)
    {
      int column_index = horizontal_angle + n;
      if (!ColumnIndexInRange(column_index))
        continue;
      for (int m = -vertical_neighbor_num; m <= vertical_neighbor_num; m++)
      {
        int row_index = vertical_angle + m;
        if (!RowIndexInRange(row_index))
          continue;
        int ind = sub2ind(row_index, column_index);
        double previous_distance_to_point = covered_voxel_[ind];
        if (isZero(previous_distance_to_point) || distance_to_point < previous_distance_to_point || reset_[ind])
        {
          covered_voxel_[ind] = distance_to_point;
          reset_[ind] = false;
        }
      }
    }
  }

  /**
   * Checks if point is visible from LiDAR pose. 
   * 
   * Calculates a horizontal angle and vertical angle from LiDAR pose and point. Gets number of horizontal/vertical 
   * neighbors, and iterates through them, returning true if there exists a point within covered_voxel_ which 
   * was closer to LiDAR than current query point, OR point was tagged to be reset_.
   * 
   * @tparam PointType signifies the type of the query point.
   * @param point query point.
   * @param occlusion_threshold threshold.
   * @return true/false on whether point is visible given LiDAR pose.
   */
  template <class PointType>
  bool CheckVisibility(const PointType& point, double occlusion_threshold) const
  {
    double distance_to_point = misc_utils_ns::PointXYZDist<PointType, geometry_msgs::Point>(point, pose_.position);

    if (isZero(distance_to_point))
      return false;

    double dx = point.x - pose_.position.x;
    double dy = point.y - pose_.position.y;
    double dz = point.z - pose_.position.z;

    int horizontal_angle = GetHorizontalAngle(dx, dy);
    int vertical_angle = GetVerticalAngle(dz, distance_to_point);

    int horizontal_neighbor_num = GetHorizontalNeighborNum(distance_to_point);
    int vertical_neighbor_num = GetVerticalNeighborNum(distance_to_point);

    for (int n = -horizontal_neighbor_num; n <= horizontal_neighbor_num; n++)
    {
      int column_index = horizontal_angle + n;
      if (!ColumnIndexInRange(column_index))
        continue;
      for (int m = -vertical_neighbor_num; m <= vertical_neighbor_num; m++)
      {
        int row_index = vertical_angle + m;
        if (!RowIndexInRange(row_index))
          continue;
        int ind = sub2ind(row_index, column_index);
        float previous_distance_to_point = covered_voxel_[ind];
        // Distance has to be less than previous distance + occlusion threshold AND reset has to be false
        // or reset just has to be true for point to be considered as visible.
        if ((!isZero(previous_distance_to_point) &&
             distance_to_point < previous_distance_to_point + occlusion_threshold && !reset_[ind]) ||
            reset_[ind])
        {
          return true;
        }
      }
    }
    return false;
  }

  void ResetCoverage();

  void GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& visualization_cloud, double resol = 0.2,
                             double max_range = 25.0) const;

  static void setCloudDWZResol(double cloud_dwz_resol)
  {
    pointcloud_resolution_ = cloud_dwz_resol * kCloudInflateRatio;
  }

  void setPose(const geometry_msgs::Pose& pose)
  {
    pose_ = pose;
  }
  geometry_msgs::Pose getPose()
  {
    return pose_;
  }
  void setPosition(const geometry_msgs::Point& position)
  {
    pose_.position = position;
  }
  geometry_msgs::Point getPosition() const
  {
    return pose_.position;
  }
  void SetHeight(double height)
  {
    pose_.position.z = height;
  }

private:
  /**
   * @brief convert subscripts to linear indices
   *
   * @param row_index row index
   * @param column_index column index
   * @return int linear index
   */
  int sub2ind(int row_index, int column_index) const;
  /**
   * @brief convert linear indices to subscripts
   *
   * @param ind linear index
   * @param row_index row index
   * @param column_index column index
   */
  void ind2sub(int ind, int& row_index, int& column_index) const;
  /**
   * @brief whether a number is close to zero
   *
   * @param x input number
   * @return true
   * @return false
   */
  bool isZero(double x) const
  {
    return std::abs(x) < kEpsilon;
  }

  /**
   * @brief Get the Horizontal Angle object 
   * 
   * atan2 gives angle in Euclidean plane between positive x axis and the ray to the point, from the robot.
   * Divides it by the reesolution to get horizontal angle. 
   * 
   * @param dx
   * @param dy
   * @return int
   */
  inline int GetHorizontalAngle(double dx, double dy) const
  {
    double horizontal_angle = (misc_utils_ns::ApproxAtan2(dy, dx) * kToDegreeConst + 180) / kHorizontalResolution;
    return static_cast<int>(round(horizontal_angle));
  }
  /**
   * @brief Get the Vertical Angle object
   * 
   * acos(dz / distance to point) gives angle in Euclidean point between positive y axis and ray to the point 
   * from the robot. 
   * 
   * @param dz
   * @param distance_to_point
   * @return int
   */
  inline int GetVerticalAngle(double dz, double distance_to_point) const
  {
    double vertical_angle =
        (acos(dz / distance_to_point) * kToDegreeConst + kVerticalAngleOffset) / kVerticalResolution;
    return static_cast<int>(round(vertical_angle));
  }
  /**
   * @brief Get the Horizontal Neighbor Num object 
   * 
   * Gets total number of horizontal neighbors by getting the number of degrees covered in horizontal direction, 
   * converting it to degrees, and dividing it by horizontal resolution to get number of points within the horizontal 
   * direction. Divides by two to account for both positive and negative direction. 
   * 
   * kHorizontalResolution (horizontal points per degree)
   * 
   * @param distance_to_point
   * @return number of horizontal neighbors, divided by 2.
   */
  inline int GetHorizontalNeighborNum(double distance_to_point) const
  {
    return static_cast<int>(ceil(pointcloud_resolution_ / distance_to_point * kToDegreeConst / kHorizontalResolution)) /
           2;
  }
  /**
   * @brief Get the Vertical Neighbor Num object
   * 
   * Gets total number of vertical neighbors by getting the number of degrees covered in vertical direction, 
   * converting it to degrees, and dividing it by vertical resolution to get number of points within the vertical 
   * direction. Divides by two to account for both positive and negative direction. 
   * 
   * kVerticalResolution (horizontal points per degree)
   * 
   * @param distance_to_point
   * @return number of vertical neighbors, divided by 2.
   */
  inline int GetVerticalNeighborNum(double distance_to_point) const
  {
    return static_cast<int>(ceil(pointcloud_resolution_ / distance_to_point * kToDegreeConst / kVerticalResolution)) /
           2;
  }
  /**
   * Checks that row is within range.
   * 
   * @param row_index query row index.
   * @return true if row index is within range, false otherwise.
   */
  inline bool RowIndexInRange(int row_index) const
  {
    return row_index >= 0 && row_index < kVerticalVoxelSize;
  }
  /**
   * Checks that column is within range.
   * 
   * @param column_index query column index.
   * @return true if column index is within range, false otherwise.
   */
  inline bool ColumnIndexInRange(int column_index) const
  {
    return column_index >= 0 && column_index < kHorizontalVoxelSize;
  }

  // Constant converting radian to degree
  static const double kToDegreeConst;
  // Constant converting degree to radian
  static const double kToRadianConst;
  // Threshold for checking if a number is close to zero
  static const double kEpsilon;
  // Ratio for inflating the cloud
  static const double kCloudInflateRatio;
  // Horizontal field-of-view in degrees
  static const int kHorizontalFOV = 360;
  // Vertical field-of-view in degrees
  static const int kVerticalFOV = 24;
  // Horizontal resolution
  static const int kHorizontalResolution = 2;
  // Vertical resolution
  static const int kVerticalResolution = 2;
  // Horizontal dimension of the voxel grid
  static const int kHorizontalVoxelSize = kHorizontalFOV / kHorizontalResolution;
  // Vertical dimension of the voxel grid
  static const int kVerticalVoxelSize = kVerticalFOV / kVerticalResolution;
  // Vertical angle offset, eg, angle [75, 105] -> indices [0, 30]
  static const int kVerticalAngleOffset = -(90 - kVerticalFOV / 2);
  // The distance that a ray can reach from the direction determined by the horizontal angle and vertical angle
  std::array<float, kHorizontalVoxelSize * kVerticalVoxelSize> covered_voxel_;
  // Whether a voxel is reset
  std::array<bool, kHorizontalVoxelSize * kVerticalVoxelSize> reset_;
  // Pose of the lidar model
  geometry_msgs::Pose pose_;
};
}  // namespace lidar_model_ns