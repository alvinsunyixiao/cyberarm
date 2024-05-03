#include <cstdio>

#include "cyber_arm/cybergear.hpp"

using namespace std::chrono_literals;

int main() {
  xiaomi::CyberGear motor(124);
  cyber_msgs::msg::CybergearState motor_state;
  motor.SetZeroPosition();

  for (int i = 0; i < 1000; ++i) {
    motor.SendMotionCommand(0.0, 0.0, 0.0, 1.0, 0.0);
    motor_state = motor.GetState();
    printf("position: %.4f velocity: %.4f torque: %.4f temperature: %.4f\n",
      motor_state.position, motor_state.velocity, motor_state.torque, motor_state.temperature);
    std::this_thread::sleep_for(5ms);
  }

  return 0;
}
