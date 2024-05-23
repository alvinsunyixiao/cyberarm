#include <algorithm>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

#include "cyber_arm/cybergear.hpp"
#include "cyber_arm/sym/forward3d.h"
#include "cyber_arm/sym/forward4d.h"
#include "cyber_msgs/msg/cybergear_state.hpp"
#include "cyber_msgs/msg/cybergear_state_stamped.hpp"
#include "cyber_msgs/msg/cyberarm_target4_d.hpp"

// #define ARM0_CAN_ID 127
// #define ARM1_CAN_ID 126
// #define ARM2_CAN_ID 125
// #define ARM3_CAN_ID 124

#define L0 0.11729
#define L1 0.129772
#define L2 0.129772
#define L3 0.10095
#define L {L0, L1, L2, L3}

#define NUM_MOTORS 4
#define NUM_LM_ITERS  20

// #define ARM0_LIMIT M_PI
// #define ARM1_LIMIT (M_PI / 2.)
// #define ARM2_LIMIT (M_PI / 2.)
// #define ARM3_LIMIT (M_PI / 2.)

using namespace std::chrono_literals;
using sensor_msgs::msg::JointState;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::PointStamped;
using std_msgs::msg::Float64;
using cyber_msgs::msg::CybergearState;
using cyber_msgs::msg::CybergearStateStamped;
using cyber_msgs::msg::CyberarmTarget4D;

class CyberarmNode : public rclcpp::Node {
 public:
  CyberarmNode() : rclcpp::Node("cyberarm_node") {

    rclcpp::sleep_for(100ms);

    js_pub_ = create_publisher<JointState>("joint_states", 10);
    viz_ee_pub_ = create_publisher<PointStamped>("viz/end_effector", 10);
    viz_target_pub_ = create_publisher<PointStamped>("viz/target3d", 10);
    target3d_sub_ = create_subscription<Point>("ctrl/target3d", 10,
      [this](Point::SharedPtr msg) { Target3DCallback(msg); });
    target4d_sub_ = create_subscription<CyberarmTarget4D>("ctrl/target4d", 10,
      [this](CyberarmTarget4D::SharedPtr msg) { Target4DCallback(msg); });

    for (size_t i = 0; i < NUM_MOTORS; ++i) {
      cb_pos_pubs_[i] = create_publisher<Float64>(
        "cybergear" + std::to_string(i) + "/cmd_position",
        10
      );
      cb_state_subs_[i] = create_subscription<CybergearStateStamped>(
        "cybergear" + std::to_string(i) + "/state",
        10,
        [this, i](CybergearStateStamped::SharedPtr msg) {
          CybergearStateCallback(msg, i);
        });
    }
  }

 private:
  rclcpp::Publisher<JointState>::SharedPtr js_pub_;
  rclcpp::Publisher<PointStamped>::SharedPtr viz_ee_pub_;
  rclcpp::Publisher<PointStamped>::SharedPtr viz_target_pub_;
  rclcpp::Subscription<Point>::SharedPtr target3d_sub_;
  rclcpp::Subscription<CyberarmTarget4D>::SharedPtr target4d_sub_;
  rclcpp::Subscription<CybergearStateStamped>::SharedPtr cb_state_subs_[NUM_MOTORS];
  rclcpp::Publisher<Float64>::SharedPtr cb_pos_pubs_[NUM_MOTORS];

  CybergearState s_arm_[NUM_MOTORS];

  void PublishPoint(rclcpp::Publisher<PointStamped>::SharedPtr pub,
                    const Point& point,
                    const std::string& frame_id = "base_link") {
    PointStamped msg{};
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = frame_id;
    msg.point = point;

    pub->publish(msg);
  }

  void Target3DCallback(Point::SharedPtr msg) {
    Eigen::Vector3d target3d;
    tf2::fromMsg(*msg, target3d);

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
      symik::Forward3D(q, lambda, target3d, L, &f, &e, &J, &A);
      q -= A.inverse() * J.transpose() * (f - target3d);
    }

    RCLCPP_INFO(get_logger(), "LM solved with error: %lf, final configuration: [%lf, %lf, %lf, %lf]",
        e, q[0], q[1], q[2], q[3]);

    for (size_t i = 0; i < NUM_MOTORS; ++i) {
      Float64 msg{};
      msg.data = q[i];
      cb_pos_pubs_[i]->publish(msg);
    }

    PublishPoint(viz_target_pub_, *msg);
  }

  void Target4DCallback(CyberarmTarget4D::SharedPtr msg) {
    Eigen::Vector4d target4d;
    target4d[0] = msg->point.x;
    target4d[1] = msg->point.y;
    target4d[2] = msg->point.z;
    target4d[3] = msg->tilt;

    if ((target4d.head<3>() - L0 * Eigen::Vector3d::UnitZ()).norm() > (L1 + L2 + L3 - 0.01)) {
      RCLCPP_WARN(get_logger(), "Target out of reachable workspace");
    }

    // solve with LM Chan
    Eigen::Vector4d q(s_arm_[0].position,
                      s_arm_[1].position,
                      s_arm_[2].position,
                      s_arm_[3].position);

    Eigen::Vector4d f;
    double e;
    Eigen::Matrix4d J;
    Eigen::Matrix4d A;
    constexpr double lambda = 1e-4;

    for (int i = 0; i < NUM_LM_ITERS; ++i) {
      symik::Forward4D(q, lambda, target4d, L, &f, &e, &J, &A);
      q -= A.inverse() * J.transpose() * (f - target4d);
    }

    RCLCPP_INFO(get_logger(), "LM solved with error: %lf, final configuration: [%lf, %lf, %lf, %lf]",
        e, q[0], q[1], q[2], q[3]);

    for (size_t i = 0; i < NUM_MOTORS; ++i) {
      Float64 msg{};
      msg.data = q[i];
      cb_pos_pubs_[i]->publish(msg);
    }

    PublishPoint(viz_target_pub_, msg->point);
  }

  void CybergearStateCallback(CybergearStateStamped::SharedPtr msg, size_t idx) {
    s_arm_[idx] = msg->state;
    PublishState(msg->header.stamp);
  }

  void PublishState(const rclcpp::Time& timestamp) {
    // publish joint state
    JointState msg{};
    msg.header.stamp = timestamp;

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
    symik::Forward3D<double>(q, lambda, target3d, L, &f, nullptr, nullptr, nullptr);

    js_pub_->publish(msg);
    PublishPoint(viz_ee_pub_, tf2::toMsg(f));
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CyberarmNode>());
  rclcpp::shutdown();
  return 0;
}
