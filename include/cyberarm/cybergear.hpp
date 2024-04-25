#pragma once

#include <linux/can.h>
#include <net/if.h>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f
// #define MAX_P 720
// #define MIN_P -720
//主机CANID设置
#define Master_CAN_ID 0x00                      //主机ID
//控制命令宏定义
//参数读取宏定义

#define Gain_Angle 720/32767.0
#define Bias_Angle 0x8000
#define Gain_Speed 30/32767.0
#define Bias_Speed 0x8000
#define Gain_Torque 12/32767.0
#define Bias_Torque 0x8000
#define Temp_Gain   0.1

#define Motor_Error 0x00
#define Motor_OK 0x01

namespace xiaomi {

enum control_mode_t {
  MOTION_MODE = 0,
  POSITION_MODE = 1,
  SPEED_MODE = 2,
  CURRENT_MODE = 3,
};

enum communication_type_t {
  GET_ID = 0x00,            //获取设备的ID和64位MCU唯一标识符
  MOTION_CONTROL = 0x01,	  //用来向主机发送控制指令
  MOTOR_STATUS = 0x02,	    //用来向主机反馈电机运行状态
  MOTOR_ENABLE = 0x03,      //电机使能运行
  MOTOR_STOP = 0x04,        //电机停止运行
  SET_ZERO_POSITION = 0x06, //设置电机机械零位
  SET_CAN_ID = 0x07,        //更改当前电机CAN_ID
  GET_SINGLE_PARAM = 0x11,	//读取单个参数
  SET_SINGLE_PARAM = 0x12,	//设定单个参数
  ERROR_FEEDBACK = 0x15,	  //故障反馈帧
};

enum param_type_t {
  RUN_MODE = 0x7005,
  IQ_REF = 0x7006,
  SPD_REF = 0x700A,
  LIMIT_TORQUE = 0x700B,
  CUR_KP = 0x7010,
  CUR_KI = 0x7011,
  CUR_FILT_GAIN = 0x7014,
  LOC_REF = 0x7016,
  LIMIT_SPD = 0x7017,
  LIMIT_CUR = 0x7018,
  MECH_POS = 0x7019,
  IQF = 0x701A,
  MECH_VEL = 0x701B,
  VBUS = 0x701C,
  ROTATION = 0x701D,
  LOC_KP = 0x701E,
  SPD_KP = 0x701F,
  SPD_KI = 0x7020,
};

class CyberGear {
 public:
  struct State {
    float position;
    float velocity;
    float torque;
    float temperature;
  };

  CyberGear(uint8_t can_id, const std::string& can_if = "can0", control_mode_t mode = MOTION_MODE);
  ~CyberGear();

  template <typename T>
  void SetMotorParameter(uint16_t index, T value) {
    static_assert (sizeof(T) <= 4, "value type size cannot exceed 4 bytes");
    can_tx_frame_.data[0] = index & 0xff;
    can_tx_frame_.data[1] = index >> 8;
    can_tx_frame_.data[2] = 0;
    can_tx_frame_.data[3] = 0;
    memcpy(can_tx_frame_.data + 4, &value, sizeof(T));
    Transmit(SET_SINGLE_PARAM);
  }

  void Start();
  void Stop(bool clear_err = false);
  void SetZeroPosition();
  void SetRunMode(control_mode_t mode);
  void SendMotionCommand(float torque, float position, float speed, float kp, float kd);
  State GetState() const;

 private:
  // CAN data structures
  const uint8_t can_id_;
  int sock_;
  struct sockaddr_can can_addr_;
  struct ifreq can_ifr_;
  struct can_frame can_tx_frame_;
  struct can_frame can_rx_frame_;

  // Rx loop
  std::thread rx_loop_;
  std::atomic<bool> rx_should_exit_ = false;

  // motor state
  mutable std::mutex state_mtx_;
  State state_;
  uint8_t error_code_;

  void Transmit(communication_type_t comm_type, uint16_t header_data = 0x00);
  void RxLoop();
};

}  // namespace xiaomi
