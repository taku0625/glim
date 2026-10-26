#pragma once

#include <memory>
#include <vector>

#include <glim/odometry/odometry_estimation_base.hpp>

namespace gtsam_points {
class GaussianVoxelMapCPU;
class IncrementalFixedLagSmootherExtWithFallback;
}  // namespace gtsam_points

namespace glim {

class CloudCovarianceEstimation;

/**
 * @brief Parameters for OdometryEstimationCT_VGICP
 */
struct OdometryEstimationCT_VGICPParams {
public:
  OdometryEstimationCT_VGICPParams();
  ~OdometryEstimationCT_VGICPParams();

public:
  int num_threads;  ///< Number of threads

  double voxel_resolution;  ///< Gaussian voxel resolution
  int voxel_lru_thresh;     ///< Voxel LRU cache threshold

  double location_consistency_inf_scale;  ///< Weight for location consistency constraints
  double constant_velocity_inf_scale;     ///< Weight for constant velocity constraints
  int lm_max_iterations;                  ///< Maximum number of iterations for LM optimization

  // iSAM2 params
  double smoother_lag;    ///< Fixed-lag smoothing window [sec]
  bool use_isam2_dogleg;  ///< If true, use dogleg optimizer
  double isam2_relinearize_skip;
  double isam2_relinearize_thresh;
};

/**
 * @brief LiDAR-only odometry estimation based on CT-VGICP scan-to-model matching
 */
class OdometryEstimationCT_VGICP : public OdometryEstimationBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationCT_VGICP(const OdometryEstimationCT_VGICPParams& params = OdometryEstimationCT_VGICPParams());
  virtual ~OdometryEstimationCT_VGICP() override;

  virtual bool requires_imu() const override { return false; }

  virtual EstimationFrame::ConstPtr insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) override;

private:
  using Params = OdometryEstimationCT_VGICPParams;
  Params params;

  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;

  int marginalized_cursor;
  std::vector<EstimationFrame::Ptr> frames;                         ///< Estimation frames
  std::shared_ptr<gtsam_points::GaussianVoxelMapCPU> target_voxelmap;  ///< Target Gaussian voxel map
  EstimationFrame::ConstPtr target_voxelmap_frame;                  ///< Target Gaussian voxel map points (just for visualization)

  // Optimizer
  using FixedLagSmootherExt = gtsam_points::IncrementalFixedLagSmootherExtWithFallback;
  std::unique_ptr<FixedLagSmootherExt> smoother;

  std::shared_ptr<void> tbb_task_arena;
};

}  // namespace glim
