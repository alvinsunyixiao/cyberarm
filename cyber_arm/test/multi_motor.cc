#include <cmath>

#include "cyber_arm/cybergear.hpp"

using namespace std::chrono_literals;

int main() {
  xiaomi::CyberGear m_arm0(127, xiaomi::POSITION_MODE);
  xiaomi::CyberGear m_arm1(126, xiaomi::POSITION_MODE);
  xiaomi::CyberGear m_arm2(125, xiaomi::POSITION_MODE);

  m_arm0.SetZeroPosition();
  m_arm1.SetZeroPosition();
  m_arm2.SetZeroPosition();
  std::this_thread::sleep_for(10ms);

  m_arm0.ConfigurePositionMode(1.0);
  m_arm1.ConfigurePositionMode();
  m_arm2.ConfigurePositionMode();
  std::this_thread::sleep_for(10ms);

  m_arm0.SendPositionCommand(0.0);
  m_arm1.SendPositionCommand(0.0);
  m_arm2.SendPositionCommand(0.0);
  std::this_thread::sleep_for(5s);

  m_arm0.SendPositionCommand(-M_PI / 4.);
  std::this_thread::sleep_for(5s);

  m_arm0.SendPositionCommand(M_PI / 4.);
  std::this_thread::sleep_for(5s);

  return 0;
}
