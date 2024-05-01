#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <Eigen/Dense>

#include <tf2_eigen/tf2_eigen.hpp>

#include "cyberarm/cybergear.hpp"

#define ARM0_CAN_ID 127
#define ARM1_CAN_ID 126
#define ARM2_CAN_ID 125
#define TIP_CAN_ID  124

#define L0 0.11729
#define L1 0.129772
#define L2 0.129772
#define L3 0.141307

using namespace std::chrono_literals;
using sensor_msgs::msg::JointState;
using geometry_msgs::msg::Vector3;

class ArmVizNode : public rclcpp::Node {
 public:
  ArmVizNode()
      : rclcpp::Node("cyberarm_node"),
        m_arm0_(ARM0_CAN_ID),
        m_arm1_(ARM1_CAN_ID),
        m_arm2_(ARM2_CAN_ID),
        m_tip_(TIP_CAN_ID) {
    m_arm0_.SetZeroPosition();
    m_arm1_.SetZeroPosition();
    m_arm2_.SetZeroPosition();
    m_tip_.SetZeroPosition();

    js_pub_ = create_publisher<JointState>("joint_states", 10);
    target_sub_ = create_subscription<Vector3>("ctrl/target_xyz", 10,
      [this](Vector3::SharedPtr msg) { TargetXYZCallback(msg); });
    viz_timer_ = create_wall_timer(10ms, std::bind(&ArmVizNode::VizLoop, this));
  }

 private:
  rclcpp::Publisher<JointState>::SharedPtr js_pub_;
  rclcpp::Subscription<Vector3>::SharedPtr target_sub_;
  rclcpp::TimerBase::SharedPtr viz_timer_;

  xiaomi::CyberGear m_arm0_;
  xiaomi::CyberGear m_arm1_;
  xiaomi::CyberGear m_arm2_;
  xiaomi::CyberGear m_tip_;

  xiaomi::CyberGear::State s_arm0_;
  xiaomi::CyberGear::State s_arm1_;
  xiaomi::CyberGear::State s_arm2_;
  xiaomi::CyberGear::State s_tip_;

  void TargetXYZCallback(Vector3::SharedPtr msg) {
    Eigen::Vector3d target_xyz;
    tf2::fromMsg(*msg, target_xyz);

    if ((target_xyz - L0 * Eigen::Vector3d::UnitZ()).norm() > (L1 + L2 + L3 - 0.05)) {
      RCLCPP_ERROR(get_logger(), "Target out of reachable workspace");
      return;
    }

  }

  void VizLoop() {
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
