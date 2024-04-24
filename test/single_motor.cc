#include <iostream>

#include "cyberarm/cybergear.hpp"

int main() {
  xiaomi::CyberGear motor(124);
  motor.SetZeroPosition();
  motor.SendMotionCommand(0.0, 0.0, 0.0, 1.0, 0.0);
  return 0;
}
