#include "optitrack/optitrack_frame_t.hpp"
#include "lcm/lcm-cpp.hpp"
#include <string>

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

class TestOptitrackData{

 public:
  void TestOptitrackData(lcm::LCM* lcm, std::string frames_channel,
                         std::string descriptions_channel) :
      lcm_(lcm) {
    lcm_subscriptions_.push_back(
        lcm_->subscribe(channel, &HandleOptitrackFrames, this));
    lcm_subscriptions_.push_back(
        lcm_->subscribe(channel, &HandleOptitrackDataDescription, this));
  }

  void HandleOptitrackFrameMsg(const lcm::ReceiveBuffer* rbuf,
                               const std::string& chan,
                               const optitrack_frame_t* optitrack_msg) {
    // drake log something.
  }

  void HandleOptitrackDataDescription(const lcm::ReceiveBuffer* rbuf,
                                      const std::string& chan,
                                      const optitrack_frame_t* optitrack_msg) {
    // drake log something.
  }

 private:
  lcm::LCM* lcm_;
  std::list<lcm::Subscription*> lcm_subscriptions_;
};


int do_main() {

  lcm::LCM lcm;
  TestOptitrackData(&lcm, "OPTITRACK_FRAMES", "OPTITRACK_DATA_DESCRIPTIONS");

  return 0;
}

} // namespace
} // namespace kuka_iiwa_arm
} // namesapce examples
} // namespace drake

int main(int argc, char *argv[]) {
  return drake::examples::kuka_iiwa_arm::do_main();
}



