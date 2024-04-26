#include <cstdio>

#include "cyberarm/cybergear.hpp"

using namespace std::chrono_literals;

int main() {
  xiaomi::CyberGear motor(124, xiaomi::POSITION_MODE);

  motor.SetZeroPosition();
  motor.ConfigurePositionMode();

  motor.SendPositionCommand(1.0);
  std::this_thread::sleep_for(5s);
  motor.SendPositionCommand(0.0);

  return 0;
}
