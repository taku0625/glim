#include <gtest/gtest.h>
#include <gtsam_points/ann/kdtree.hpp>
#include <glim/odometry/odometry_estimation_base.hpp>
#include <glim/odometry/odometry_estimation_ct_vgicp.hpp>
#include <glim/util/config.hpp>

namespace {
std::string module_path;

// A deterministic 3D scene with samples distributed over the whole scan.
glim::PreprocessedFrame::Ptr make_frame(int index) {
  auto frame = std::make_shared<glim::PreprocessedFrame>();
  frame->stamp = index * 0.1;
  frame->scan_end_time = frame->stamp + 0.09;
  for (int x = 0; x < 5; x++) {
    for (int y = 0; y < 5; y++) {
      for (int z = 0; z < 5; z++) {
        frame->points.emplace_back(2.0 * x + 0.5, 2.0 * y + 0.5, 2.0 * z + 0.5, 1.0);
      }
    }
  }
  const double translation = index * 0.02;
  for (size_t i = 0; i < frame->points.size(); i++) {
    frame->times.emplace_back(0.09 * i / (frame->points.size() - 1));
    frame->points[i].x() -= translation;
  }
  frame->k_neighbors = 8;
  gtsam_points::KdTree tree(frame->points.data(), frame->points.size());
  for (const auto& point : frame->points) {
    size_t indices[8];
    double distances[8];
    const auto found = tree.knn_search(point.data(), 8, indices, distances);
    if (found != 8) {
      throw std::runtime_error("failed to construct test neighborhood");
    }
    for (auto neighbor : indices) {
      frame->neighbors.emplace_back(neighbor);
    }
  }
  return frame;
}

TEST(OdometryCTVGICP, LoadsModuleAndTracksWithoutIMU) {
  auto odometry = glim::OdometryEstimationBase::load_module(module_path);
  ASSERT_TRUE(odometry);
  EXPECT_FALSE(odometry->requires_imu());
  std::vector<glim::EstimationFrame::ConstPtr> marginalized;
  for (int index = 0; index < 15; index++) {
    const auto result = odometry->insert_frame(make_frame(index), marginalized);
    ASSERT_TRUE(result);
    ASSERT_TRUE(result->T_world_lidar.matrix().allFinite());
    EXPECT_NEAR(result->T_world_lidar.translation().x(), index * 0.02, 0.02);
    EXPECT_LT(result->T_world_lidar.translation().tail<2>().norm(), 0.02);
    EXPECT_LT((result->T_world_lidar.linear() - Eigen::Matrix3d::Identity()).norm(), 0.02);
    EXPECT_EQ(result->frame_id, glim::FrameID::LIDAR);
  }
  // The configured one-second window must actually marginalize old scans.
  EXPECT_FALSE(marginalized.empty());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  if (argc != 3) {
    return 2;
  }
  auto config = glim::GlobalConfig::instance(argv[1]);
  config->override_param<std::string>("global", "config_odometry", "config_odometry_ct_vgicp.json");
  module_path = argv[2];
  return RUN_ALL_TESTS();
}
