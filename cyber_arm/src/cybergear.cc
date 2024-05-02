#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can/raw.h>
#include <fcntl.h>
#include <poll.h>

#include <unistd.h>
#include <cstring>
#include <functional>
#include <iostream>

#include "cyberarm/cybergear.hpp"

namespace xiaomi {

using cyber_msgs::msg::CybergearState;

static uint16_t float_to_uint16(float x, float x_min, float x_max){
  float span = x_max - x_min;
  float offset = x_min;
  if(x > x_max) x=x_max;
  else if(x < x_min) x= x_min;
  return static_cast<uint16_t>(((x - offset) * ((float)(0xffff)) / span));
}

static float uint16_to_float(uint16_t x, float x_min, float x_max) {
  float span = static_cast<float>(0xffff);
  float offset = x_max - x_min;
  return offset * static_cast<float>(x) / span + x_min;
}


CyberGear::CyberGear(uint8_t can_id, control_mode_t mode, const std::string& can_if) : can_id_(can_id), ctrl_mode_(mode) {
  // socket initialization
  sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  strcpy(can_ifr_.ifr_name, can_if.c_str());
  ioctl(sock_, SIOGIFINDEX, &can_ifr_);

  // bind socket to CAN interface
  can_addr_.can_family = AF_CAN;
  can_addr_.can_ifindex = can_ifr_.ifr_ifindex;
  if (bind(sock_, (struct sockaddr*)&can_addr_, sizeof(can_addr_)) == -1) {
    std::cerr << "CAN socket bind failure" << std::endl;
  }

  // set socket to non blocking IO
  int flags = fcntl(sock_, F_GETFL, 0);
  if (flags < 0) {
    std::cerr << "Error getting socket flags" << std::endl;
  }
  if (fcntl(sock_, F_SETFL, flags | O_NONBLOCK) < 0) {
    std::cerr << "Error setting nonblocking flag on socket" << std::endl;
  }

  // set CAN rx filter
  struct can_filter filter;
  filter.can_id = can_id_ << 8;
  filter.can_mask = 0xff << 8;
  if (setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
    std::cerr << "CAN filter init failure" << std::endl;
  }

  // initialize CAN frame
  can_tx_frame_.can_dlc = 8;

  // initialize rx loop
  rx_loop_ = std::thread(std::bind(&CyberGear::RxLoop, this));

  SetRunMode(mode);
  Start();
}

CyberGear::~CyberGear() {
  Stop();

  rx_should_exit_.store(true);
  rx_loop_.join();

  close(sock_);
}

void CyberGear::Start() {
  Transmit(MOTOR_ENABLE);
  enabled_ = true;
}

void CyberGear::Stop(bool clear_err) {
  memset(can_tx_frame_.data, 0, sizeof(can_tx_frame_.data));
  can_tx_frame_.data[0] = clear_err;
  Transmit(MOTOR_STOP);
  enabled_ = false;
}

void CyberGear::SetZeroPosition() {
  can_tx_frame_.data[0] = 1;
  Transmit(SET_ZERO_POSITION);
}

void CyberGear::SetRunMode(control_mode_t mode) {
  if (!enabled_) {
    SetMotorParameter<uint8_t>(RUN_MODE, mode);
    ctrl_mode_ = mode;
  } else {
    std::cerr << "Cannot change mode while running" << std::endl;
  }
}

void CyberGear::ConfigurePositionMode(float max_velocity,
                                      float max_current,
                                      float position_kp,
                                      float velocity_kp,
                                      float velocity_ki) {
  SetMotorParameter(LIMIT_CUR, max_current);
  SetMotorParameter(LIMIT_SPD, max_velocity);
  SetMotorParameter(LOC_KP, position_kp);
  SetMotorParameter(SPD_KP, velocity_kp);
  SetMotorParameter(SPD_KI, velocity_ki);
}

void CyberGear::SendMotionCommand(float torque, float position, float velocity, float kp, float kd) {
  uint16_t torque_uint = float_to_uint16(torque, T_MIN, T_MAX);
  uint16_t position_uint = float_to_uint16(position, P_MIN, P_MAX);
  uint16_t velocity_uint = float_to_uint16(velocity, V_MIN, V_MAX);
  uint16_t kp_uint = float_to_uint16(kp, KP_MIN, KP_MAX);
  uint16_t kd_uint = float_to_uint16(kd, KD_MIN, KD_MAX);

  can_tx_frame_.data[0] = position_uint >> 8;
  can_tx_frame_.data[1] = position_uint;
  can_tx_frame_.data[2] = velocity_uint >> 8;
  can_tx_frame_.data[3] = velocity_uint;
  can_tx_frame_.data[4] = kp_uint >> 8;
  can_tx_frame_.data[5] = kp_uint;
  can_tx_frame_.data[6] = kd_uint >> 8;
  can_tx_frame_.data[7] = kd_uint;

  Transmit(MOTION_CONTROL, torque_uint);
}

void CyberGear::SendPositionCommand(float position) {
  SetMotorParameter(LOC_REF, position);
}

CybergearState CyberGear::GetState() const {
  std::lock_guard<std::mutex> lock(state_mtx_);
  return state_;
}

void CyberGear::Transmit(communication_type_t comm_type, uint16_t header_data) {
  can_tx_frame_.can_id = CAN_EFF_FLAG | (comm_type << 24) | (header_data << 8) | can_id_;
  if (write(sock_, &can_tx_frame_, sizeof(can_tx_frame_)) != sizeof(can_tx_frame_)) {
    std::cerr << "CAN transmit failure" << std::endl;
  }
}

void CyberGear::RxLoop() {
  struct pollfd pfd;
  pfd.fd = sock_;
  pfd.events = POLLIN;

  while (!rx_should_exit_.load()) {
    if (poll(&pfd, 1, 1000) > 0 && pfd.revents & POLLIN) {
      if (read(sock_, &can_rx_frame_, sizeof(can_rx_frame_)) < 0) {
        std::cerr << "CAN read error (shouldn't get here)" << std::endl;
        continue;
      }

      if (((can_rx_frame_.can_id >> 8) & 0xff) != can_id_) {
        std::cerr << "CAN ID does not match (should not get here)" << std::endl;
        continue;
      }

      {
        std::lock_guard<std::mutex> lock(state_mtx_);
        state_.position = uint16_to_float(
          can_rx_frame_.data[0] << 8 | can_rx_frame_.data[1], P_MIN, P_MAX);
        state_.velocity = uint16_to_float(
          can_rx_frame_.data[2] << 8 | can_rx_frame_.data[3], V_MIN, V_MAX);
        state_.torque = uint16_to_float(
          can_rx_frame_.data[4] << 8 | can_rx_frame_.data[5], T_MIN, T_MAX);
        state_.temperature = (
          can_rx_frame_.data[6] << 8 | can_rx_frame_.data[7]) * TEMPERATURE_GAIN;
        error_code_ = (can_rx_frame_.can_id >> 16) & 0x1f;
      }
    }
  }

  std::cout << "RX Loop successfully stopped for CAN ID: " << (int)can_id_ << std::endl;
}

}  // namespace xiaomi
