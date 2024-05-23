#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64.hpp>

#include "cyber_arm/cybergear.hpp"
#include "cyber_msgs/msg/cybergear_state_stamped.hpp"
#include "cyber_msgs/msg/motion_command.hpp"

using cyber_msgs::msg::CybergearStateStamped;
using cyber_msgs::msg::MotionCommand;
using std_msgs::msg::Float64;

template <typename T>
static T Clip(T val, T abs_limit) {
  abs_limit = std::abs(abs_limit);
  return std::clamp(val, -abs_limit, abs_limit);
}

class CybergearNode : public rclcpp::Node {
 public:
  CybergearNode() : rclcpp::Node("cybergear_node") {
    this->declare_parameter("can_id", 127);
    this->declare_parameter("mode", static_cast<int>(xiaomi::MOTION_MODE));
    this->declare_parameter("max_velocity", 2.0);
    this->declare_parameter("max_current", 5.0);
    this->declare_parameter("max_torque", 4.0);
    this->declare_parameter("position_kp", 15.0);
    this->declare_parameter("velocity_kp", 1.0);
    this->declare_parameter("velocity_ki", 0.002);
    this->declare_parameter("motion_kp", 0.0);
    this->declare_parameter("motion_kd", 0.0);
    this->declare_parameter("position_abs_limit", M_PI / 2.);

    state_pub_ = this->create_publisher<CybergearStateStamped>("state", 10);
    motion_cmd_sub_ = this->create_subscription<MotionCommand>("cmd_motion", 10,
        [this](MotionCommand::SharedPtr msg) {MotionCommandCallback(msg);});
    position_cmd_sub_ = this->create_subscription<Float64>("cmd_position", 10,
        [this](Float64::SharedPtr msg) {PositionCommandCallback(msg);});

    param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    param_cb_handles_.push_back(param_sub_->add_parameter_callback("can_id",
      [this](const rclcpp::Parameter&) {
        CybergearInit();
        RCLCPP_INFO(get_logger(), "CAN ID changed, re-initializing motor");
      }));
    param_cb_handles_.push_back(param_sub_->add_parameter_callback("mode",
      [this](const rclcpp::Parameter&) {
        cybergear_->Stop();
        cybergear_->SetRunMode(Mode());
        cybergear_->Start();
      }));
    param_cb_handles_.push_back(param_sub_->add_parameter_callback("max_velocity",
      [this](const rclcpp::Parameter& p) {
        cybergear_->SetMotorParameter<float>(xiaomi::LIMIT_SPD, p.as_double());
      }));
    param_cb_handles_.push_back(param_sub_->add_parameter_callback("max_current",
      [this](const rclcpp::Parameter& p) {
        cybergear_->SetMotorParameter<float>(xiaomi::LIMIT_CUR, p.as_double());
      }));
    param_cb_handles_.push_back(param_sub_->add_parameter_callback("position_kp",
      [this](const rclcpp::Parameter& p) {
        cybergear_->SetMotorParameter<float>(xiaomi::LOC_KP, p.as_double());
      }));
    param_cb_handles_.push_back(param_sub_->add_parameter_callback("velocity_kp",
      [this](const rclcpp::Parameter& p) {
        cybergear_->SetMotorParameter<float>(xiaomi::SPD_KP, p.as_double());
      }));
    param_cb_handles_.push_back(param_sub_->add_parameter_callback("velocity_ki",
      [this](const rclcpp::Parameter& p) {
        cybergear_->SetMotorParameter<float>(xiaomi::SPD_KI, p.as_double());
      }));

    CybergearInit();
  }

 private:
  std::shared_ptr<rclcpp::ParameterEventHandler> param_sub_;
  std::unique_ptr<xiaomi::CyberGear> cybergear_;
  rclcpp::Publisher<CybergearStateStamped>::SharedPtr state_pub_;
  rclcpp::Subscription<MotionCommand>::SharedPtr motion_cmd_sub_;
  rclcpp::Subscription<Float64>::SharedPtr position_cmd_sub_;
  std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> param_cb_handles_;

  uint8_t CanID() const {
    return static_cast<uint8_t>(this->get_parameter("can_id").as_int());
  }

  xiaomi::control_mode_t Mode() const {
    return static_cast<xiaomi::control_mode_t>(this->get_parameter("mode").as_int());
  }

  double MaxVelocity() const {
    return this->get_parameter("max_velocity").as_double();
  }

  double MaxCurrent() const {
    return this->get_parameter("max_current").as_double();
  }

  double MaxTorque() const {
    return this->get_parameter("max_torque").as_double();
  }

  double PositionKp() const {
    return this->get_parameter("position_kp").as_double();
  }

  double VelocityKp() const {
    return this->get_parameter("velocity_kp").as_double();
  }

  double VelocityKi() const {
    return this->get_parameter("velocity_ki").as_double();
  }

  double MotionKp() const {
    return this->get_parameter("motion_kp").as_double();
  }

  double MotionKd() const {
    return this->get_parameter("motion_kd").as_double();
  }

  double PositionAbsLimit() const {
    return this->get_parameter("position_abs_limit").as_double();
  }

  void CybergearInit() {
    cybergear_ = std::make_unique<xiaomi::CyberGear>(CanID(), Mode());
    cybergear_->SetZeroPosition();

    if (Mode() == xiaomi::POSITION_MODE) {
      cybergear_->ConfigurePositionMode(MaxVelocity(),
                                        MaxCurrent(),
                                        PositionKp(),
                                        VelocityKp(),
                                        VelocityKi());
    }

    cybergear_->RegisterStateCallback(std::bind(&CybergearNode::PublishState, this));
  }

  void PublishState() {
    CybergearStateStamped msg{};
    msg.header.stamp = this->get_clock()->now();
    msg.state = cybergear_->GetState();
    state_pub_->publish(msg);
  }

  void MotionCommandCallback(MotionCommand::SharedPtr msg) {
    if (Mode() == xiaomi::MOTION_MODE) {
      cybergear_->SendMotionCommand(Clip(msg->torque, MaxTorque()),
                                    Clip(msg->position, PositionAbsLimit()),
                                    Clip(msg->velocity, MaxVelocity()),
                                    MotionKp(), MotionKd());
    } else {
      RCLCPP_WARN(get_logger(), "motion command ignored: cybergear not in motion mode");
    }
  }

  void PositionCommandCallback(Float64::SharedPtr msg) {
    if (Mode() == xiaomi::POSITION_MODE) {
      cybergear_->SendPositionCommand(Clip(msg->data, PositionAbsLimit()));
    } else {
      RCLCPP_WARN(get_logger(), "position command ignored: cybergear not in position mode");
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CybergearNode>());
  rclcpp::shutdown();
}
