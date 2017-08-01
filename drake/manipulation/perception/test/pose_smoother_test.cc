#include "drake/manipulation/perception/pose_smoother.h"

#include <memory>
#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace manipulation {
namespace perception {
namespace test {

class PoseSmootherTest : public ::testing::Test {
 public:
  void Initialize(
      double max_linear_velocity, double max_angular_velocity,
      int filter_window_size, double optitrack_lcm_status_period) {
    dut_ = std::make_unique<PoseSmoother>(
        max_linear_velocity, max_angular_velocity, filter_window_size,
        optitrack_lcm_status_period);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);

    EXPECT_EQ(dut_->get_num_input_ports(), 1);
    EXPECT_EQ(dut_->get_num_output_ports(), 1);
  }
 private:
  std::unique_ptr<PoseSmoother> dut_;
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
};

TEST_F(PoseSmootherTest, InvalidObjectTest) {
  EXPECT_NO_THROW(Initialize(1.5, 0.5, 4, 200));
}

} // namespace test
} // namespace perception
} // namespace manipulation
} // namespace drake