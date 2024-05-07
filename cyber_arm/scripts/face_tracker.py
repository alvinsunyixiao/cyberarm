#!/usr/bin/env python3

import cv2
import dlib
import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Pose
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge

from cyber_msgs.msg import CyberarmConfig
from realsense2_camera_msgs.msg import RGBD


class FaceDetectorNode(Node):
    def __init__(self):
        super().__init__("face_detector_node")

        self.rgbd_sub = self.create_subscription(RGBD, "/camera/camera/rgbd", self.rgbd_callback, 10)
        self.viz_pub = self.create_publisher(Image, "/viz/detected_faces", 10)
        self.face_lm_pub = self.create_publisher(Marker, "/viz/face_landmarks", 10)
        self.bridge = CvBridge()
        self.detector = dlib.get_frontal_face_detector()
        self.predictor = dlib.shape_predictor("../ml/shape_predictor_5_face_landmarks.dat")

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
            cv2.rectangle(img_viz, (r.left(), r.top()), (r.right(), r.bottom()), (255, 0, 0))
            shape = self.predictor(img_rgb, r)
            for j in range(shape.num_parts):
                cv2.circle(img_viz, (shape.part(j).x, shape.part(j).y), 1, (255, 0, 0), -1)

            # only visualize the first detection
            if i == 0:
                marker = Marker()
                marker.header.frame_id = "camera_color_optical_frame"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.type = Marker.POINTS
                marker.action = Marker.ADD
                identity_T = Pose()
                identity_T.orientation.w = 1.
                marker.pose = identity_T
                marker.scale.x = 0.01
                marker.scale.y = 0.01
                marker.scale.z = 0.01
                marker.color.r = 1.0
                marker.color.a = 1.0
                for j in range(shape.num_parts):
                    uv1 = np.array([shape.part(j).x, shape.part(j).y, 1.])
                    K = np.asarray(msg.rgb_camera_info.k).reshape(3, 3)
                    x_norm = np.linalg.inv(K) @ uv1
                    depth = img_depth[shape.part(j).y, shape.part(j).x]
                    x_scaled = depth * x_norm
                    marker.points.append(Point(
                        x=x_scaled[0],
                        y=x_scaled[1],
                        z=x_scaled[2],
                    ))
                    marker.colors.append(marker.color)

                self.face_lm_pub.publish(marker)


        self.viz_pub.publish(self.bridge.cv2_to_imgmsg(img_viz, encoding="rgb8"))


if __name__ == "__main__":
    rclpy.init()
    node = FaceDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()
