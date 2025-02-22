/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_bindings/fast_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

// A collection of values which can be added and later removed, and the maximum
// of the current values in the collection can be retrieved.
// All of it in (amortized) O(1).
class SlidingWindowMaximum {
 public:
  void AddValue(const float value) {
    while (!non_ascending_maxima_.empty() &&
           value > non_ascending_maxima_.back()) {
      non_ascending_maxima_.pop_back();
    }
    non_ascending_maxima_.push_back(value);
  }

  void RemoveValue(const float value) {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK(!non_ascending_maxima_.empty());
    DCHECK_LE(value, non_ascending_maxima_.front());
    if (value == non_ascending_maxima_.front()) {
      non_ascending_maxima_.pop_front();
    }
  }

  float GetMaximum() const {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK_GT(non_ascending_maxima_.size(), 0);
    return non_ascending_maxima_.front();
  }

  void CheckIsEmpty() const { CHECK_EQ(non_ascending_maxima_.size(), 0); }

 private:
  // Maximum of the current sliding window at the front. Then the maximum of the
  // remaining window that came after this values first occurrence, and so on.
  std::deque<float> non_ascending_maxima_;
};

}  // namespace

proto::FastCorrelativeScanMatcherOptions2D
CreateFastCorrelativeScanMatcherOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::FastCorrelativeScanMatcherOptions2D options;
  options.set_linear_search_window(
      parameter_dictionary->GetDouble("linear_search_window"));
  options.set_angular_search_window(
      parameter_dictionary->GetDouble("angular_search_window"));
  options.set_branch_and_bound_depth(
      parameter_dictionary->GetInt("branch_and_bound_depth"));
  return options;
}


bool convolve2DFast(unsigned char* in, unsigned char* out, int dataSizeX, int dataSizeY, float* kernel, int kernelSizeX,
    int kernelSizeY)
{
    int i, j, m, n, x, y, t;
    unsigned char **inPtr, *outPtr, *ptr;
    int kCenterX, kCenterY;
    int rowEnd, colEnd;  // ending indice for section divider
    float sum;           // temp accumulation buffer
    int k, kSize;

    // check validity of params
    if (!in || !out || !kernel)
        return false;
    if (dataSizeX <= 0 || kernelSizeX <= 0)
        return false;

    // find center position of kernel (half of kernel size)
    kCenterX = kernelSizeX >> 1;
    kCenterY = kernelSizeY >> 1;
    kSize = kernelSizeX * kernelSizeY;  // total kernel size

    // allocate memeory for multi-cursor
    inPtr = new unsigned char*[kSize];
    if (!inPtr)
        return false;  // allocation error

    // set initial position of multi-cursor, NOTE: it is swapped instead of kernel
    ptr = in + (dataSizeX * kCenterY + kCenterX);  // the first cursor is shifted (kCenterX, kCenterY)
    for (m = 0, t = 0; m < kernelSizeY; ++m)
    {
        for (n = 0; n < kernelSizeX; ++n, ++t)
        {
            inPtr[t] = ptr - n;
        }
        ptr -= dataSizeX;
    }

    // init working  pointers
    outPtr = out;

    rowEnd = dataSizeY - kCenterY;  // bottom row partition divider
    colEnd = dataSizeX - kCenterX;  // right column partition divider

    // convolve rows from index=0 to index=kCenterY-1
    y = kCenterY;
    for (i = 0; i < kCenterY; ++i)
    {
        // partition #1 ***********************************
        x = kCenterX;
        for (j = 0; j < kCenterX; ++j)  // column from index=0 to index=kCenterX-1
        {
            sum = 0;
            t = 0;
            for (m = 0; m <= y; ++m)
            {
                for (n = 0; n <= x; ++n)
                {
                    sum += *inPtr[t] * kernel[t];
                    ++t;
                }
                t += (kernelSizeX - x - 1);  // jump to next row
            }

            // store output
            *outPtr = (unsigned char)((float)fabs(sum) + 0.5f);
            ++outPtr;
            ++x;
            for (k = 0; k < kSize; ++k)
                ++inPtr[k];  // move all cursors to next
        }

        // partition #2 ***********************************
        for (j = kCenterX; j < colEnd; ++j)  // column from index=kCenterX to index=(dataSizeX-kCenterX-1)
        {
            sum = 0;
            t = 0;
            for (m = 0; m <= y; ++m)
            {
                for (n = 0; n < kernelSizeX; ++n)
                {
                    sum += *inPtr[t] * kernel[t];
                    ++t;
                }
            }

            // store output
            *outPtr = (unsigned char)((float)fabs(sum) + 0.5f);
            ++outPtr;
            ++x;
            for (k = 0; k < kSize; ++k)
                ++inPtr[k];  // move all cursors to next
        }

        // partition #3 ***********************************
        x = 1;
        for (j = colEnd; j < dataSizeX; ++j)  // column from index=(dataSizeX-kCenter) to index=(dataSizeX-1)
        {
            sum = 0;
            t = x;
            for (m = 0; m <= y; ++m)
            {
                for (n = x; n < kernelSizeX; ++n)
                {
                    sum += *inPtr[t] * kernel[t];
                    ++t;
                }
                t += x;  // jump to next row
            }

            // store output
            *outPtr = (unsigned char)((float)fabs(sum) + 0.5f);
            ++outPtr;
            ++x;
            for (k = 0; k < kSize; ++k)
                ++inPtr[k];  // move all cursors to next
        }

        ++y;  // add one more row to convolve for next run
    }

    // convolve rows from index=kCenterY to index=(dataSizeY-kCenterY-1)
    for (i = kCenterY; i < rowEnd; ++i)  // number of rows
    {
        // partition #4 ***********************************
        x = kCenterX;
        for (j = 0; j < kCenterX; ++j)  // column from index=0 to index=kCenterX-1
        {
            sum = 0;
            t = 0;
            for (m = 0; m < kernelSizeY; ++m)
            {
                for (n = 0; n <= x; ++n)
                {
                    sum += *inPtr[t] * kernel[t];
                    ++t;
                }
                t += (kernelSizeX - x - 1);
            }

            // store output
            *outPtr = (unsigned char)((float)fabs(sum) + 0.5f);
            ++outPtr;
            ++x;
            for (k = 0; k < kSize; ++k)
                ++inPtr[k];  // move all cursors to next
        }

        // partition #5 ***********************************
        for (j = kCenterX; j < colEnd; ++j)  // column from index=kCenterX to index=(dataSizeX-kCenterX-1)
        {
            sum = 0;
            t = 0;
            for (m = 0; m < kernelSizeY; ++m)
            {
                for (n = 0; n < kernelSizeX; ++n)
                {
                    sum += *inPtr[t] * kernel[t];
                    ++inPtr[t];  // in this partition, all cursors are used to convolve. moving cursors to next is safe
                                 // here
                    ++t;
                }
            }

            // store output
            *outPtr = (unsigned char)((float)fabs(sum) + 0.5f);
            ++outPtr;
            ++x;
        }

        // partition #6 ***********************************
        x = 1;
        for (j = colEnd; j < dataSizeX; ++j)  // column from index=(dataSizeX-kCenter) to index=(dataSizeX-1)
        {
            sum = 0;
            t = x;
            for (m = 0; m < kernelSizeY; ++m)
            {
                for (n = x; n < kernelSizeX; ++n)
                {
                    sum += *inPtr[t] * kernel[t];
                    ++t;
                }
                t += x;
            }

            // store output
            *outPtr = (unsigned char)((float)fabs(sum) + 0.5f);
            ++outPtr;
            ++x;
            for (k = 0; k < kSize; ++k)
                ++inPtr[k];  // move all cursors to next
        }
    }

    // convolve rows from index=(dataSizeY-kCenterY) to index=(dataSizeY-1)
    y = 1;
    for (i = rowEnd; i < dataSizeY; ++i)  // number of rows
    {
        // partition #7 ***********************************
        x = kCenterX;
        for (j = 0; j < kCenterX; ++j)  // column from index=0 to index=kCenterX-1
        {
            sum = 0;
            t = kernelSizeX * y;

            for (m = y; m < kernelSizeY; ++m)
            {
                for (n = 0; n <= x; ++n)
                {
                    sum += *inPtr[t] * kernel[t];
                    ++t;
                }
                t += (kernelSizeX - x - 1);
            }

            // store output
            *outPtr = (unsigned char)((float)fabs(sum) + 0.5f);
            ++outPtr;
            ++x;
            for (k = 0; k < kSize; ++k)
                ++inPtr[k];  // move all cursors to next
        }

        // partition #8 ***********************************
        for (j = kCenterX; j < colEnd; ++j)  // column from index=kCenterX to index=(dataSizeX-kCenterX-1)
        {
            sum = 0;
            t = kernelSizeX * y;
            for (m = y; m < kernelSizeY; ++m)
            {
                for (n = 0; n < kernelSizeX; ++n)
                {
                    sum += *inPtr[t] * kernel[t];
                    ++t;
                }
            }

            // store output
            *outPtr = (unsigned char)((float)fabs(sum) + 0.5f);
            ++outPtr;
            ++x;
            for (k = 0; k < kSize; ++k)
                ++inPtr[k];
        }

        // partition #9 ***********************************
        x = 1;
        for (j = colEnd; j < dataSizeX; ++j)  // column from index=(dataSizeX-kCenter) to index=(dataSizeX-1)
        {
            sum = 0;
            t = kernelSizeX * y + x;
            for (m = y; m < kernelSizeY; ++m)
            {
                for (n = x; n < kernelSizeX; ++n)
                {
                    sum += *inPtr[t] * kernel[t];
                    ++t;
                }
                t += x;
            }

            // store output
            *outPtr = (unsigned char)((float)fabs(sum) + 0.5f);
            ++outPtr;
            ++x;
            for (k = 0; k < kSize; ++k)
                ++inPtr[k];  // move all cursors to next
        }

        ++y;  // the starting row index is increased
    }

    return true;
}


PrecomputationGrid2D::PrecomputationGrid2D(
    const Grid2D& grid, const CellLimits& limits, const int width,
    std::vector<float>* reusable_intermediate_grid)
    : offset_(-width + 1, -width + 1),
      wide_limits_(limits.num_x_cells + width - 1,
                   limits.num_y_cells + width - 1),
      min_score_(1.f - grid.GetMaxCorrespondenceCost()),
      max_score_(1.f - grid.GetMinCorrespondenceCost()),
      cells_(wide_limits_.num_x_cells * wide_limits_.num_y_cells) {
  CHECK_GE(width, 1);
  CHECK_GE(limits.num_x_cells, 1);
  CHECK_GE(limits.num_y_cells, 1);
  const int stride = wide_limits_.num_x_cells;
  // First we compute the maximum probability for each (x0, y) achieved in the
  // span defined by x0 <= x < x0 + width.
  std::vector<float>& intermediate = *reusable_intermediate_grid;
  intermediate.resize(wide_limits_.num_x_cells * limits.num_y_cells);
  for (int y = 0; y != limits.num_y_cells; ++y) {
    SlidingWindowMaximum current_values;
    current_values.AddValue(
        1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(0, y))));
    for (int x = -width + 1; x != 0; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      if (x + width < limits.num_x_cells) {
        current_values.AddValue(1.f - std::abs(grid.GetCorrespondenceCost(
                                          Eigen::Array2i(x + width, y))));
      }
    }
    for (int x = 0; x < limits.num_x_cells - width; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(
          1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(x, y))));
      current_values.AddValue(1.f - std::abs(grid.GetCorrespondenceCost(
                                        Eigen::Array2i(x + width, y))));
    }
    for (int x = std::max(limits.num_x_cells - width, 0);
         x != limits.num_x_cells; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(
          1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(x, y))));
    }
    current_values.CheckIsEmpty();
  }
  // For each (x, y), we compute the maximum probability in the width x width
  // region starting at each (x, y) and precompute the resulting bound on the
  // score.
  for (int x = 0; x != wide_limits_.num_x_cells; ++x) {
    SlidingWindowMaximum current_values;
    current_values.AddValue(intermediate[x]);
    for (int y = -width + 1; y != 0; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      if (y + width < limits.num_y_cells) {
        current_values.AddValue(intermediate[x + (y + width) * stride]);
      }
    }
    for (int y = 0; y < limits.num_y_cells - width; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
      current_values.AddValue(intermediate[x + (y + width) * stride]);
    }
    for (int y = std::max(limits.num_y_cells - width, 0);
         y != limits.num_y_cells; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
    }
    current_values.CheckIsEmpty();
  }
  // auto in = cells_;
  // Eigen::Matrix3f kernel;
  // kernel << 1/16.f, 1/8.f, 1/16.f,
  //   1/8.f, 1/4.f, 1/8.f,
  //   1/16.f, 1/8.f, 1/16.f;
  // convolve2DFast(in.data(), cells_.data(), wide_limits_.num_x_cells, wide_limits_.num_y_cells, kernel.data(), 3, 3);
}

uint8 PrecomputationGrid2D::ComputeCellValue(const float probability) const {
  const int cell_value = common::RoundToInt(
      (probability - min_score_) * (255.f / (max_score_ - min_score_)));
  CHECK_GE(cell_value, 0);
  CHECK_LE(cell_value, 255);
  return cell_value;
}

PrecomputationGridStack2D::PrecomputationGridStack2D(
    const Grid2D& grid,
    const proto::FastCorrelativeScanMatcherOptions2D& options) {
  CHECK_GE(options.branch_and_bound_depth(), 1);
  const int max_width = 1 << (options.branch_and_bound_depth() - 1);
  precomputation_grids_.reserve(options.branch_and_bound_depth());
  std::vector<float> reusable_intermediate_grid;
  const CellLimits limits = grid.limits().cell_limits();
  reusable_intermediate_grid.reserve((limits.num_x_cells + max_width - 1) *
                                     limits.num_y_cells);
  for (int i = 0; i != options.branch_and_bound_depth(); ++i) {
    const int width = 1 << i;
    precomputation_grids_.emplace_back(grid, limits, width + 1, &reusable_intermediate_grid);
  }
}

FastCorrelativeScanMatcher2D::FastCorrelativeScanMatcher2D(
    const Grid2D& grid,
    const proto::FastCorrelativeScanMatcherOptions2D& options)
    : options_(options),
      limits_(grid.limits()),
      precomputation_grid_stack_(
          absl::make_unique<PrecomputationGridStack2D>(grid, options)) {}

FastCorrelativeScanMatcher2D::~FastCorrelativeScanMatcher2D() {}

bool FastCorrelativeScanMatcher2D::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const std::vector<Eigen::Vector2f>& point_cloud, const float min_score, float* score,
    transform::Rigid2d* pose_estimate, Eigen::Matrix3d& covariance) const {
  const SearchParameters search_parameters(options_.linear_search_window(),
                                           options_.angular_search_window(),
                                           point_cloud, limits_.resolution());
  return MatchWithSearchParameters(search_parameters, initial_pose_estimate,
                                   point_cloud, min_score, score,
                                   pose_estimate, covariance);
}

// bool FastCorrelativeScanMatcher2D::MatchFullSubmap(
//     const sensor::PointCloud& point_cloud, float min_score, float* score,
//     transform::Rigid2d* pose_estimate) const {
//   // Compute a search window around the center of the submap that includes it
//   // fully.
//   const SearchParameters search_parameters(
//       1e6 * limits_.resolution(),  // Linear search window, 1e6 cells/direction.
//       M_PI,  // Angular search window, 180 degrees in both directions.
//       point_cloud, limits_.resolution());
//   const transform::Rigid2d center = transform::Rigid2d::Translation(
//       limits_.max() - 0.5 * limits_.resolution() *
//                           Eigen::Vector2d(limits_.cell_limits().num_y_cells,
//                                           limits_.cell_limits().num_x_cells));
//   return MatchWithSearchParameters(search_parameters, center, point_cloud,
//                                    min_score, score, pose_estimate);
// }

bool FastCorrelativeScanMatcher2D::MatchWithSearchParameters(
    SearchParameters search_parameters,
    const transform::Rigid2d& initial_pose_estimate,
    const std::vector<Eigen::Vector2f>& point_cloud, float min_score, float* score,
    transform::Rigid2d* pose_estimate, Eigen::Matrix3d& covariance) const {
  CHECK(score != nullptr);
  CHECK(pose_estimate != nullptr);

  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const std::vector<DiscreteScan2D> discrete_scans = GenerateRotatedScans(
      point_cloud,
      search_parameters,
      initial_pose_estimate,
      limits_);
  search_parameters.ShrinkToFit(discrete_scans, limits_.cell_limits());

  const std::vector<Candidate2D> lowest_resolution_candidates =
      ComputeLowestResolutionCandidates(discrete_scans, search_parameters);
  const Candidate2D best_candidate = BranchAndBound(
      discrete_scans, search_parameters, lowest_resolution_candidates,
      precomputation_grid_stack_->max_depth(), min_score);
  if (best_candidate.score > min_score) {
    *score = best_candidate.score;
    auto pose = best_candidate.ToPose(search_parameters);
    *pose_estimate = transform::Rigid2d(
        initial_pose_estimate.translation() + pose.head<2>(),
        initial_rotation * Eigen::Rotation2Dd(pose[2]));

    Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
    Eigen::Vector3d u = Eigen::Vector3d::Zero();
    double s = 0.0;
    constexpr int w_size = 5;
    const int scan_window = std::min(w_size, std::min(best_candidate.scan_index, static_cast<int>(discrete_scans.size()) - 1 - best_candidate.scan_index));
    const auto& grid = precomputation_grid_stack_->Get(0);
    for (int i = best_candidate.x_index_offset - w_size; i <= best_candidate.x_index_offset + w_size; i++) {
      for (int j = best_candidate.y_index_offset - w_size; j <= best_candidate.y_index_offset + w_size; j++) {
        for (int k = best_candidate.scan_index - scan_window; k <= best_candidate.scan_index + scan_window; k++) {
          int sum = 0;
          Eigen::Array2i offset(i, j);
          for (const Eigen::Array2i& xy_index : discrete_scans[k]) {
            sum += grid.GetValue(xy_index + offset);
          }
          float score = grid.ToScore(sum / static_cast<float>(discrete_scans[k].size()));
          Eigen::Vector3d x = Candidate2D::ToPose(i, j, k, search_parameters);
          K += x * x.transpose() * score;
          u += x * score;
          s += score;
        }
      }
    }
    double demon = 1.0 / s;
    covariance = demon * K - demon * demon * (u * u.transpose());
    return true;
  }
  return false;
}

std::vector<Candidate2D>
FastCorrelativeScanMatcher2D::ComputeLowestResolutionCandidates(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters) const {
  std::vector<Candidate2D> lowest_resolution_candidates =
      GenerateLowestResolutionCandidates(search_parameters);
  ScoreCandidates(
      precomputation_grid_stack_->Get(precomputation_grid_stack_->max_depth()),
      discrete_scans, search_parameters, &lowest_resolution_candidates);
  return lowest_resolution_candidates;
}

std::vector<Candidate2D>
FastCorrelativeScanMatcher2D::GenerateLowestResolutionCandidates(
    const SearchParameters& search_parameters) const {
  const int linear_step_size = 1 << precomputation_grid_stack_->max_depth();
  int num_candidates = 0;
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    const int num_lowest_resolution_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + linear_step_size) /
        linear_step_size;
    const int num_lowest_resolution_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + linear_step_size) /
        linear_step_size;
    num_candidates += num_lowest_resolution_linear_x_candidates *
                      num_lowest_resolution_linear_y_candidates;
  }
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         x_index_offset += linear_step_size) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           y_index_offset += linear_step_size) {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

void FastCorrelativeScanMatcher2D::ScoreCandidates(
    const PrecomputationGrid2D& precomputation_grid,
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate2D>* const candidates) const {
  for (Candidate2D& candidate : *candidates) {
    int sum = 0;
    for (const Eigen::Array2i& xy_index :
         discrete_scans[candidate.scan_index]) {
      const Eigen::Array2i proposed_xy_index(
          xy_index.x() + candidate.x_index_offset,
          xy_index.y() + candidate.y_index_offset);
      sum += precomputation_grid.GetValue(proposed_xy_index);
    }
    candidate.score = precomputation_grid.ToScore(
        sum / static_cast<float>(discrete_scans[candidate.scan_index].size()));
    // Eigen::Vector3d x = candidate.ToPose(search_parameters);
    // K += x * x.transpose() * candidate.score;
    // u += x * candidate.score;
    // s += candidate.score;
  }
  std::sort(candidates->begin(), candidates->end(),
            std::greater<Candidate2D>());
}

Candidate2D FastCorrelativeScanMatcher2D::BranchAndBound(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    const std::vector<Candidate2D>& candidates, const int candidate_depth,
    float min_score) const {
  if (candidate_depth == 0) {
    // Return the best candidate.
    return *candidates.begin();
  }

  Candidate2D best_high_resolution_candidate(0, 0, 0, search_parameters);
  best_high_resolution_candidate.score = min_score;
  for (const Candidate2D& candidate : candidates) {
    if (candidate.score <= min_score) {
      break;
    }
    std::vector<Candidate2D> higher_resolution_candidates;
    const int half_width = 1 << (candidate_depth - 1);
    for (int x_offset : {0, half_width}) {
      if (candidate.x_index_offset + x_offset >
          search_parameters.linear_bounds[candidate.scan_index].max_x) {
        break;
      }
      for (int y_offset : {0, half_width}) {
        if (candidate.y_index_offset + y_offset >
            search_parameters.linear_bounds[candidate.scan_index].max_y) {
          break;
        }
        higher_resolution_candidates.emplace_back(
            candidate.scan_index, candidate.x_index_offset + x_offset,
            candidate.y_index_offset + y_offset, search_parameters);
      }
    }
    ScoreCandidates(precomputation_grid_stack_->Get(candidate_depth - 1),
                    discrete_scans, search_parameters,
                    &higher_resolution_candidates);
    best_high_resolution_candidate = std::max(
        best_high_resolution_candidate,
        BranchAndBound(discrete_scans, search_parameters,
                       higher_resolution_candidates, candidate_depth - 1,
                       best_high_resolution_candidate.score));
  }
  return best_high_resolution_candidate;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
