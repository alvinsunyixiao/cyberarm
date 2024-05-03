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
        self.bridge = CvBridge()
        self.detector = dlib.get_frontal_face_detector()
        self.predictor = dlib.shape_predictor("/home/alvin/Downloads/shape_predictor_68_face_landmarks.dat")

    def img_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        rects = self.detector(img)
        for r in rects:
            cv2.rectangle(img, (r.left(), r.top()), (r.right(), r.bottom()), (255, 0, 0))
            shape = self.predictor(img, r)
            for i in range(shape.num_parts):
                cv2.circle(img, (shape.part(i).x, shape.part(i).y), 1, (255, 0, 0), -1)

        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imshow("img", img)
        cv2.waitKey(1)


if __name__ == "__main__":
    rclpy.init()
    node = FaceDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()
