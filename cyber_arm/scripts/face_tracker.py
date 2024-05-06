#!/usr/bin/env python3

import cv2
import dlib

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class FaceDetectorNode(Node):
    def __init__(self):
        super().__init__("face_detector_node")
        self.img_sub = self.create_subscription(Image, "/camera/camera/color/image_raw", self.img_callback, 10)
        self.viz_pub = self.create_publisher(Image, "/viz/detected_faces", 10)
        self.bridge = CvBridge()
        self.detector = dlib.get_frontal_face_detector()
        self.predictor = dlib.shape_predictor("../ml/shape_predictor_5_face_landmarks.dat")

    def img_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        rects = self.detector(img)

        viz_img = img.copy()
        for r in rects:
            cv2.rectangle(viz_img, (r.left(), r.top()), (r.right(), r.bottom()), (255, 0, 0))
            shape = self.predictor(img, r)
            for i in range(shape.num_parts):
                cv2.circle(viz_img, (shape.part(i).x, shape.part(i).y), 1, (255, 0, 0), -1)

        self.viz_pub.publish(self.bridge.cv2_to_imgmsg(viz_img, encoding="rgb8"))


if __name__ == "__main__":
    rclpy.init()
    node = FaceDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()
