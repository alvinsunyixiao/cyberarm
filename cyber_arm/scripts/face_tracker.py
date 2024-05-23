#!/usr/bin/env python3

import cv2
import dlib
import math
import numpy as np

import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import Point, Pose, PointStamped
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge

from cyber_msgs.msg import CyberarmConfig, CyberarmTarget4D
from realsense2_camera_msgs.msg import RGBD


class FaceDetectorNode(Node):
    def __init__(self):
        super().__init__("face_detector_node")

        self.rgbd_sub = self.create_subscription(RGBD, "/camera/camera/rgbd", self.rgbd_callback, 1)
        self.viz_pub = self.create_publisher(Image, "/viz/detected_faces", 10)
        self.face_lm_pub = self.create_publisher(Marker, "/viz/face_landmarks", 10)
        self.ctrl_pub = self.create_publisher(CyberarmTarget4D, "/ctrl/target4d", 10)
        #self.ctrl_pub = self.create_publisher(PointStamped, "/viz/nose", 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()
        self.detector = dlib.get_frontal_face_detector()
        self.predictor = dlib.shape_predictor("../ml/shape_predictor_5_face_landmarks.dat")
        self.x_nose = None

        self.send_init_config()

    def send_init_config(self):
        self.config_pub = self.create_publisher(CyberarmConfig, "/ctrl/configuration", 10)
        config = CyberarmConfig()
        config.q[0] = math.radians(-90.)
        config.q[1] = math.radians(30.)
        config.q[2] = math.radians(-60.)
        config.q[3] = 0.
        self.config_pub.publish(config)

    def rgbd_callback(self, msg: RGBD):
        img_rgb = self.bridge.imgmsg_to_cv2(msg.rgb, desired_encoding="rgb8")
        img_depth = self.bridge.imgmsg_to_cv2(msg.depth) * 1e-3
        rects = self.detector(img_rgb)

        img_viz = img_rgb.copy()
        for i, r in enumerate(rects):
            shape = self.predictor(img_rgb, r)

            # only visualize the first detection
            if i == 0:
                marker = Marker()
                marker.header.frame_id = "camera_color_optical_frame"
                marker.header.stamp = msg.header.stamp
                marker.type = Marker.POINTS
                marker.action = Marker.ADD
                identity_T = Pose()
                identity_T.orientation.w = 1.
                marker.pose = identity_T
                marker.scale.x = 0.01
                marker.scale.y = 0.01
                marker.scale.z = 0.01
                for j in range(shape.num_parts):
                    u = np.clip(shape.part(j).x, 0, msg.depth_camera_info.width - 1, dtype=int)
                    v = np.clip(shape.part(j).y, 0, msg.depth_camera_info.height - 1, dtype=int)
                    uv1 = np.array([u, v, 1.])
                    K = np.asarray(msg.rgb_camera_info.k).reshape(3, 3)
                    x_norm = np.linalg.inv(K) @ uv1
                    depth = img_depth[v, u]
                    x_scaled = depth * x_norm
                    marker.points.append(Point(
                        x=x_scaled[0],
                        y=x_scaled[1],
                        z=x_scaled[2],
                    ))
                    if j == 4:
                        marker.colors.append(ColorRGBA(b=1.0, a=0.5))
                        if self.x_nose is None:
                            self.x_nose = x_scaled
                        else:
                            self.x_nose = 0.9 * self.x_nose + 0.1 * x_scaled
                    else:
                        marker.colors.append(ColorRGBA(r=1.0, a=0.5))

                marker.points.append(Point(x=self.x_nose[0], y=self.x_nose[1], z=self.x_nose[2]))
                marker.colors.append(ColorRGBA(g=1.0, a=1.0))

                x_nose_stamped = PointStamped()
                x_nose_stamped.header.frame_id = "camera_color_optical_frame"
                x_nose_stamped.header.stamp = msg.header.stamp
                x_nose_stamped.point.x = self.x_nose[0]
                x_nose_stamped.point.y = self.x_nose[1]
                x_nose_stamped.point.z = self.x_nose[2]

                try:
                    x_nose_world = self.tf_buffer.transform(x_nose_stamped, "base_link")
                except tf2_ros.TransformException as ex:
                    self.get_logger().error(f"transform failed: {ex}")
                    return

                x_nose_world_xy = np.array([x_nose_world.point.x, x_nose_world.point.y, 0.])
                x_nose_world_xy /= np.linalg.norm(x_nose_world_xy)
                x_nose = np.array([x_nose_world.point.x, x_nose_world.point.y, x_nose_world.point.z])
                x_cam_world = x_nose - x_nose_world_xy * 0.5

                ctrl_msg = CyberarmTarget4D()
                ctrl_msg.point.x = x_cam_world[0]
                ctrl_msg.point.y = x_cam_world[1]
                ctrl_msg.point.z = x_cam_world[2]
                ctrl_msg.tilt = math.pi / 2.
                self.ctrl_pub.publish(ctrl_msg)

                self.face_lm_pub.publish(marker)

            cv2.rectangle(img_viz, (r.left(), r.top()), (r.right(), r.bottom()), (255, 0, 0))
            for j in range(shape.num_parts):
                cv2.circle(img_viz, (shape.part(j).x, shape.part(j).y), 1, (255, 0, 0), -1)

        self.viz_pub.publish(self.bridge.cv2_to_imgmsg(img_viz, encoding="rgb8"))


if __name__ == "__main__":
    rclpy.init()
    node = FaceDetectorNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()
