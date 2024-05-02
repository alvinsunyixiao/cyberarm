#include <algorithm>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

#include "cyberarm/cybergear.hpp"
#include "cyberarm/sym/forward3d.h"

#define ARM0_CAN_ID 127
#define ARM1_CAN_ID 126
#define ARM2_CAN_ID 125
#define ARM3_CAN_ID 124

#define L0 0.11729
#define L1 0.129772
#define L2 0.129772
#define L3 0.141307

#define NUM_MOTORS 4
#define NUM_LM_ITERS  20

using namespace std::chrono_literals;
using sensor_msgs::msg::JointState;
using geometry_msgs::msg::PointStamped;
using cyber_msgs::msg::CybergearState;

class CyberarmNode : public rclcpp::Node {
 public:
  CyberarmNode()
      : rclcpp::Node("cyberarm_node") {

    m_arm_[0] = std::make_unique<xiaomi::CyberGear>(ARM0_CAN_ID, xiaomi::POSITION_MODE);
    m_arm_[1] = std::make_unique<xiaomi::CyberGear>(ARM1_CAN_ID, xiaomi::POSITION_MODE);
    m_arm_[2] = std::make_unique<xiaomi::CyberGear>(ARM2_CAN_ID, xiaomi::POSITION_MODE);
    m_arm_[3] = std::make_unique<xiaomi::CyberGear>(ARM3_CAN_ID, xiaomi::POSITION_MODE);

    for (int i = 0; i < NUM_MOTORS; ++i) {
      m_arm_[i]->SetZeroPosition();
      cb_state_pubs_[i] = create_publisher<CybergearState>("state/arm" + std::to_string(i), 10);
    }

    rclcpp::sleep_for(100ms);

    m_arm_[0]->ConfigurePositionMode();
    m_arm_[1]->ConfigurePositionMode(2.0, 23.0, 30.0, 3.0, 0.006);
    m_arm_[2]->ConfigurePositionMode(2.0, 23.0, 30.0, 3.0, 0.004);
    m_arm_[3]->ConfigurePositionMode(2.0, 23.0, 30.0, 3.0, 0.004);

    js_pub_ = create_publisher<JointState>("joint_states", 10);
    ee_pub_ = create_publisher<PointStamped>("state/end_effector", 10);
    target_sub_ = create_subscription<PointStamped>("ctrl/target3d", 10,
      [this](PointStamped::SharedPtr msg) { Target3DCallback(msg); });
    state_timer_ = create_wall_timer(5ms, std::bind(&CyberarmNode::StateLoop, this));
    ctrl_timer_ = create_wall_timer(5ms, std::bind(&CyberarmNode::CtrlLoop, this));
  }

 private:
  rclcpp::Publisher<JointState>::SharedPtr js_pub_;
  rclcpp::Publisher<PointStamped>::SharedPtr ee_pub_;
  rclcpp::Subscription<PointStamped>::SharedPtr target_sub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr ctrl_timer_;
  rclcpp::Publisher<CybergearState>::SharedPtr cb_state_pubs_[NUM_MOTORS];

  std::unique_ptr<xiaomi::CyberGear> m_arm_[NUM_MOTORS];
  CybergearState s_arm_[NUM_MOTORS];

  Eigen::Vector4d target_q_ = Eigen::Vector4d::Zero();

  void Target3DCallback(PointStamped::SharedPtr msg) {
    Eigen::Vector3d target3d;
    tf2::fromMsg(msg->point, target3d);

    if ((target3d - L0 * Eigen::Vector3d::UnitZ()).norm() > (L1 + L2 + L3 - 0.01)) {
      RCLCPP_WARN(get_logger(), "Target out of reachable workspace");
    }

    // solve with LM Chan
    Eigen::Vector4d q(s_arm_[0].position,
                      s_arm_[1].position,
                      s_arm_[2].position,
                      s_arm_[3].position);

    Eigen::Vector3d f;
    double e;
    Eigen::Matrix<double, 3, 4> J;
    Eigen::Matrix4d A;
    constexpr double lambda = 1e-4;

    for (int i = 0; i < NUM_LM_ITERS; ++i) {
      symik::Forward3D(q, lambda, target3d, &f, &e, &J, &A);
      q -= A.inverse() * J.transpose() * (f - target3d);
      q[0] = std::clamp(q[0], -M_PI / 2., M_PI / 2.);
      q[1] = std::clamp(q[1], -M_PI / 4., M_PI / 4.);
      q[2] = std::clamp(q[2], -M_PI / 2., M_PI / 2.);
      q[3] = std::clamp(q[3], -M_PI / 2., M_PI / 2.);
    }

    RCLCPP_INFO(get_logger(), "LM solved with error: %lf, final configuration: [%lf, %lf, %lf, %lf]",
        e, q[0], q[1], q[2], q[3]);

    target_q_ = q;
  }

  void StateLoop() {
    // obtain latest states
    for (int i = 0; i < NUM_MOTORS; ++i) {
      s_arm_[i] = m_arm_[i]->GetState();
    }

    // get timestamp
    const auto now = this->get_clock()->now();

    // publish joint state
    JointState msg{};
    msg.header.stamp = now;

    msg.position.resize(4);
    msg.position[0] = s_arm_[0].position;
    msg.position[1] = s_arm_[1].position;
    msg.position[2] = s_arm_[2].position;
    msg.position[3] = s_arm_[3].position;

    msg.name.resize(4);
    msg.name[0] = "base_to_arm1_joint";
    msg.name[1] = "arm1_to_arm2_joint";
    msg.name[2] = "arm2_to_arm3_joint";
    msg.name[3] = "arm3_to_end_effector_joint";

    // publish end effector location
    constexpr double lambda = 1e-4;
    const Eigen::Vector3d target3d = Eigen::Vector3d::Zero();
    const Eigen::Vector4d q(s_arm_[0].position,
                            s_arm_[1].position,
                            s_arm_[2].position,
                            s_arm_[3].position);
    Eigen::Vector3d f;
    symik::Forward3D<double>(q, lambda, target3d, &f, nullptr, nullptr, nullptr);

    PointStamped ee_msg{};
    ee_msg.point = tf2::toMsg(f);
    ee_msg.header.stamp = now;
    ee_msg.header.frame_id = "base_link";

    js_pub_->publish(msg);
    ee_pub_->publish(ee_msg);
    for (int i = 0; i < NUM_MOTORS; ++i) {
      cb_state_pubs_[i]->publish(s_arm_[i]);
    }
  }

  void CtrlLoop() {
    for (int i = 0; i < NUM_MOTORS; ++i) {
      m_arm_[i]->SendPositionCommand(target_q_[i]);
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CyberarmNode>());
  rclcpp::shutdown();
  return 0;
}
