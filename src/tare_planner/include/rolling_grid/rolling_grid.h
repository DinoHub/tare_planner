/**
 * @file rolling_grid.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a rolling 3D grid
 * @version 0.1
 * @date 2020-06-10
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <vector>
#include <algorithm>
#include <cmath>

#include <Eigen/Core>

#include <grid/grid.h>
#include <utils/misc_utils.h>

namespace rolling_grid_ns
{
class RollingGrid
{
public:
  explicit RollingGrid(const Eigen::Vector3i& size);
  ~RollingGrid() = default;
  /**
   * Checks if input point is within grid size/local planning horizon defined by kNumber
   */
  bool InRange(Eigen::Vector3i sub) const
  {
    return grid0_->InRange(sub);
  }
  /**
   * Checks if input point is within grid size/local planning horizon defined by kNumber
   */
  bool InRange(int ind) const
  {
    return grid0_->InRange(ind);
  }
  /**
   * Converts ind (1D int) to sub (3D vector).
   */
  Eigen::Vector3i Ind2Sub(int ind) const
  {
    return grid0_->Ind2Sub(ind);
  }
  /**
   * Converts sub (3D vector) to ind (1D int) according to a flattened grid
   */
  int Sub2Ind(Eigen::Vector3i sub) const
  {
    return grid0_->Sub2Ind(sub);
  }
  /**
   * Returns the updated grid's cell value at the input point
   */
  int GetArrayInd(Eigen::Vector3i sub) const
  {
    MY_ASSERT(InRange(sub));
    if (which_grid_)
    {
      return grid1_->GetCellValue(sub);
    }
    else
    {
      return grid0_->GetCellValue(sub);
    }
  }
  /**
   * Returns the updated grid's cell value at the input point.
   * 
   * TODO: convert sub2ind instead of ind2sub since GetCellValue() works in ind space. InRange works in either. 
   * which_grid_ check can be in ind space instead.
   */
  int GetArrayInd(int ind) const
  {
    MY_ASSERT(InRange(ind));
    Eigen::Vector3i sub = grid0_->Ind2Sub(ind);
    return GetArrayInd(sub);
  }
  /**
   * Returns the index of the rolling grid, where the input (array_ind) is at. 
   */
  int GetInd(int array_ind) const
  {
    MY_ASSERT(InRange(array_ind));
    return array_ind_to_ind_[array_ind];
  }

  void Roll(const Eigen::Vector3i& roll_dir);
  void GetUpdatedIndices(std::vector<int>& updated_indices) const;
  void GetRolledOutIndices(const Eigen::Vector3i& roll_dir, std::vector<int>& rolled_out_indices);
  void GetUpdatedArrayIndices(std::vector<int>& updated_array_indices) const;

private:
  Eigen::Vector3i size_;
  std::unique_ptr<grid_ns::Grid<int>> grid0_;
  std::unique_ptr<grid_ns::Grid<int>> grid1_;
  std::vector<int> updated_indices_;
  std::vector<int> array_ind_to_ind_;
  bool which_grid_;
  /**
   * Given current index and no. of grids to shfit by, returns new index.
   * 
   * If index > roll, subtract directly. Else, wrap around by adding max_idx
   * @param cur_idx Current grid index E.g. 35
   * @param roll_step Number of grids to roll (shift) by E.g 31
   * @param max_idx Grid size in this dimension E.g. 40
   */
  inline int GetFromIdx(int cur_idx, int roll_step, int max_idx) const
  {
    return cur_idx <= roll_step - 1 ? max_idx - roll_step + cur_idx : cur_idx - roll_step;
  }
  void RollHelper(const std::unique_ptr<grid_ns::Grid<int>>& grid_in,
                  const std::unique_ptr<grid_ns::Grid<int>>& grid_out, Eigen::Vector3i roll_dir);

  void GetRolledInIndices(const Eigen::Vector3i& roll_dir);
  void GetRolledOutIndices(const Eigen::Vector3i& roll_dir);
  void GetIndices(std::vector<int>& indices, Eigen::Vector3i start_idx, Eigen::Vector3i end_idx) const;
};
}  // namespace rolling_grid_ns