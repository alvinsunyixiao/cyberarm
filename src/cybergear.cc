#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can/raw.h>

#include <unistd.h>
#include <cstring>
#include <iostream>

#include "cyberarm/cybergear.hpp"

namespace xiaomi {

static uint16_t float_to_uint16(float x, float x_min, float x_max){
  float span = x_max - x_min;
  float offset = x_min;
  if(x > x_max) x=x_max;
  else if(x < x_min) x= x_min;
  return static_cast<uint16_t>(((x - offset) * ((float)(0xffff)) / span));
}


CyberGear::CyberGear(int can_id, const std::string& can_if, control_mode_t mode) : can_id_(can_id) {
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

  // initialize CAN frame
  can_tx_frame_.can_dlc = 8;

  SetRunMode(mode);
  Start();
}

CyberGear::~CyberGear() {
  close(sock_);
}

void CyberGear::Start() {
  Transmit(MotorEnable);
}

void CyberGear::Stop(bool clear_err) {
  memset(can_tx_frame_.data, 0, sizeof(can_tx_frame_.data));
  can_tx_frame_.data[0] = clear_err;
  Transmit(MotorStop);
}

void CyberGear::SetZeroPosition() {
  can_tx_frame_.data[0] = 1;
  Transmit(SetPosZero);
}

void CyberGear::SetRunMode(control_mode_t mode) {
  SetMotorParameter<uint8_t>(PARAM_RUN_MODE, mode);
}

void CyberGear::SendMotionCommand(float torque, float position, float speed, float kp, float kd) {
  uint16_t torque_uint = float_to_uint16(torque, T_MIN, T_MAX);
  uint16_t position_uint = float_to_uint16(position, P_MIN, P_MAX);
  uint16_t speed_uint = float_to_uint16(speed, V_MIN, V_MAX);
  uint16_t kp_uint = float_to_uint16(kp, KP_MIN, KP_MAX);
  uint16_t kd_uint = float_to_uint16(kd, KD_MIN, KD_MAX);

  can_tx_frame_.data[0] = position_uint >> 8;
  can_tx_frame_.data[1] = position_uint;
  can_tx_frame_.data[2] = speed_uint >> 8;
  can_tx_frame_.data[3] = speed_uint;
  can_tx_frame_.data[4] = kp_uint >> 8;
  can_tx_frame_.data[5] = kp_uint;
  can_tx_frame_.data[6] = kd_uint >> 8;
  can_tx_frame_.data[7] = kd_uint;

  Transmit(MotionControl, torque_uint);
}

void CyberGear::Transmit(communication_type_t comm_type, uint16_t header_data) {
  can_tx_frame_.can_id = CAN_EFF_FLAG | (comm_type << 24) | (header_data << 8) | can_id_;
  if (write(sock_, &can_tx_frame_, sizeof(can_tx_frame_)) != sizeof(can_tx_frame_)) {
    std::cerr << "CAN transmit failure" << std::endl;
  }
}

}  // namespace xiaomi
