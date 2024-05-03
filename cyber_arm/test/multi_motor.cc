#include "cyber_arm/cybergear.hpp"

using namespace std::chrono_literals;

int main() {
  xiaomi::CyberGear m_tip(124);
  xiaomi::CyberGear m_arm(125);
  m_tip.SetZeroPosition();
  m_arm.SetZeroPosition();

  for (int i = 0; i < 1000; ++i) {
    m_tip.SendMotionCommand(0.0, 0.0, 0.0, 1.0, 0.0);
    m_arm.SendMotionCommand(0.0, 0.0, 0.0, 1.0, 0.0);
    std::this_thread::sleep_for(5ms);
  }

  return 0;
}
