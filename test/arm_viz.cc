#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "cyberarm/cybergear.hpp"

using namespace std::chrono_literals;
using sensor_msgs::msg::JointState;

class ArmVizNode : public rclcpp::Node {
 public:
  ArmVizNode()
      : rclcpp::Node("arm_viz_node"),
        m_arm0_(127),
        m_arm1_(126),
        m_arm2_(125),
        m_tip_(124) {
    m_arm0_.SetZeroPosition();
    m_arm1_.SetZeroPosition();
    m_arm2_.SetZeroPosition();
    m_tip_.SetZeroPosition();

    js_pub_ = create_publisher<JointState>("joint_states", 10);
    loop_timer_ = create_wall_timer(5ms, std::bind(&ArmVizNode::Loop, this));
  }

 private:
  rclcpp::Publisher<JointState>::SharedPtr js_pub_;
  rclcpp::TimerBase::SharedPtr loop_timer_;

  xiaomi::CyberGear m_arm0_;
  xiaomi::CyberGear m_arm1_;
  xiaomi::CyberGear m_arm2_;
  xiaomi::CyberGear m_tip_;

  xiaomi::CyberGear::State s_arm0_;
  xiaomi::CyberGear::State s_arm1_;
  xiaomi::CyberGear::State s_arm2_;
  xiaomi::CyberGear::State s_tip_;

  void Loop() {
    m_arm0_.SendMotionCommand(0.0, 0.0, 0.0, 1.0, 0.0);
    m_arm1_.SendMotionCommand(0.0, 0.0, 0.0, 1.0, 0.0);
    m_arm2_.SendMotionCommand(0.0, 0.0, 0.0, 1.0, 0.0);
    m_tip_.SendMotionCommand(0.0, 0.0, 0.0, 1.0, 0.0);

    s_arm0_ = m_arm0_.GetState();
    s_arm1_ = m_arm1_.GetState();
    s_arm2_ = m_arm2_.GetState();
    s_tip_ = m_tip_.GetState();

    JointState msg{};
    msg.header.stamp = this->get_clock()->now();

    msg.position.resize(4);
    msg.position[0] = s_arm0_.position;
    msg.position[1] = s_arm1_.position;
    msg.position[2] = s_arm2_.position;
    msg.position[3] = s_tip_.position;

    msg.name.resize(4);
    msg.name[0] = "base_to_arm1_joint";
    msg.name[1] = "arm1_to_arm2_joint";
    msg.name[2] = "arm2_to_arm3_joint";
    msg.name[3] = "arm3_to_end_effector_joint";

    js_pub_->publish(msg);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmVizNode>());
  rclcpp::shutdown();
  return 0;
}
