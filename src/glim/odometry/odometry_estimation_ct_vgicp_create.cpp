#include <glim/odometry/odometry_estimation_ct_vgicp.hpp>

extern "C" glim::OdometryEstimationBase* create_odometry_estimation_module() {
  glim::OdometryEstimationCT_VGICPParams params;
  return new glim::OdometryEstimationCT_VGICP(params);
}