#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import argparse

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy.duration


sys.path.append("/home/ros_ws/src/rise_contact_graspnet2/contact_graspnet")
from data import load_available_input_data

import rclpy
from rclpy.node import Node
import rclpy.logging

from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from rise_contact_msg2.srv import ContactGraspNetPlanner
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Pose


class Main(Node):
    def __init__(self):
        super().__init__("grasp_planner_client")
        self.rgb_subscription = self.create_subscription(
            Image, "/camera/color/image_raw", self.rgb_callback, 10
        )
        self.depth_subscription = self.create_subscription(
            Image, "/camera/depth/image_raw", self.depth_callback, 10
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self.camera_info_callback, 10
        )
        self.grasp_result_publisher = self.create_publisher(Pose, "/cgn_grasp_point", 1)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.rgb_received = False
        self.depth_received = False
        self.camera_info_received = False

        # get sample data
        sample_data_path = "/home/ros_ws/src/rise_contact_graspnet2/test_data/7.npy"
        segmap, rgb, depth, cam_K, pc_full, pc_colors = load_available_input_data(
            sample_data_path, K=None
        )
        # self.get_logger().info("Load exmaple datas: {}".format(sample_data_path))

        # conver to ros input data
        self.cv_bridge = CvBridge()
        # print("rgb size: ", np.array(rgb).shape)
        # print("depth size: ", np.array(depth).shape)
        # rgb_img_msg = cv_bridge.cv2_to_imgmsg(np.array(rgb))
        # depth_img_msg = self.cv_bridge.cv2_to_imgmsg(np.array(depth))
        # print("depth_img_msg: ", depth_img_msg.data, flush=True)
        self.segmask = self.cv_bridge.cv2_to_imgmsg(np.array(segmap))
        # camera_intr = CameraInfo()
        # camera_intr.k = np.array(cam_K).reshape(9)

        # request service to server
        self.get_logger().info("Start grasp_planner_client")

        self.cli = self.create_client(ContactGraspNetPlanner, "grasp_planner")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = ContactGraspNetPlanner.Request()

        # Request
        # resp_result = self.send_request(
        #     rgb_img_msg, depth_img_msg, segmask, camera_intr
        # )
        # self.get_logger().info(
        #     "Get {} grasps from the server.".format(len(resp_result.grasps))
        # )
        self.init = False

    def send_request(self, rgb_img_msg, depth_img_msg, segmask, camera_intr):
        self.get_logger().info("Request Contact-GraspNet grasp planning")

        self.req.color_image = rgb_img_msg
        self.req.detph_image = depth_img_msg
        self.req.camera_info = camera_intr
        self.req.segmask = segmask
        self.resp = self.cli.call_async(self.req)

        rclpy.spin_until_future_complete(self, self.resp)
        return self.resp.result()

    def rgb_callback(self, msg):
        # rclpy.logging.get_logger("client").info("Received rgb image")
        self.rgb_data = msg
        self.rgb_received = True

    def depth_callback(self, msg):
        # rclpy.logging.get_logger("client").info("Received depth image")
        self.depth_data = msg
        self.depth_received = True

    def camera_info_callback(self, msg):
        # rclpy.logging.get_logger("client").info("Received camera info")
        self.camera_info = msg
        self.camera_info_received = True

    def make_tf(self, resp_result):
        transform = TransformStamped()
        # Set the header
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "camera_depth_optical_frame"
        self.min_z_idx = 0
        self.max_score_idx = 0
        self.min_z = 1000
        self.max_score = 0

        for i, grasp in enumerate(resp_result.grasps):
            # # Set the child frame ID
            # transform.child_frame_id = "grasp_result" + str(i)

            # # Set translation
            # transform.transform.translation.x = grasp.pose.position.x
            # transform.transform.translation.y = grasp.pose.position.y
            # transform.transform.translation.z = grasp.pose.position.z

            if grasp.pose.position.z < self.min_z:
                self.min_z_idx = i
                self.min_z = grasp.pose.position.z

            if grasp.score > self.max_score:
                self.max_score_idx = i
                self.max_score = self.max_score_idx

            # print("\n grasp # ", i, flush=True)
            # print("score: ", grasp.score, flush=True)
            # print("z position: ", grasp.pose.position.z, flush=True)

            # # Set rotation
            # transform.transform.rotation = grasp.pose.orientation

            # # Broadcast the transform
            # self.tf_broadcaster.sendTransform(transform)

        # print("min_z: ", self.min_z_idx, flush=True)
        # print("max score: ", self.max_score_idx, flush=True)
        transform.child_frame_id = "grasp_result"

        # Set translation
        transform.transform.translation.x = resp_result.grasps[
            self.min_z_idx
        ].pose.position.x
        transform.transform.translation.y = resp_result.grasps[
            self.min_z_idx
        ].pose.position.y
        transform.transform.translation.z = resp_result.grasps[
            self.min_z_idx
        ].pose.position.z

        # Set rotation
        transform.transform.rotation = resp_result.grasps[
            self.min_z_idx
        ].pose.orientation

        self.tf_broadcaster.sendTransform(transform)
        self.grasp_result_publisher.publish(resp_result.grasps[self.min_z_idx].pose)

        transform.child_frame_id = "max_score"
        transform.transform.translation.x = resp_result.grasps[
            self.max_score_idx
        ].pose.position.x
        transform.transform.translation.y = resp_result.grasps[
            self.max_score_idx
        ].pose.position.y
        transform.transform.translation.z = resp_result.grasps[
            self.max_score_idx
        ].pose.position.z

        # Set rotation
        transform.transform.rotation = resp_result.grasps[
            self.max_score_idx
        ].pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def run(self):
        while rclpy.ok():
            # 카메라 토픽이 들어오지 않으면, spin_once에서 무한 대기 상태가 된다.
            # See: http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
            rclpy.spin_once(self)
            if self.rgb_received and self.depth_received and self.camera_info_received:
                user_input = (
                    input("Type 'y' to send the request or 'quit' to exit: ")
                    .strip()
                    .lower()
                )
                if user_input == "y":
                    cv2_depth = (
                        self.cv_bridge.imgmsg_to_cv2(self.depth_data, "passthrough")
                        / 1000.0
                    )
                    depth_img = self.cv_bridge.cv2_to_imgmsg(cv2_depth)
                    resp_result = self.send_request(
                        self.rgb_data, depth_img, self.segmask, self.camera_info
                    )
                    self.make_tf(resp_result)

                    self.rgb_received = False
                    self.depth_received = False
                    self.camera_info_received = False

                if user_input == "quit":
                    break

            # break


def main(args=None):
    rclpy.init(args=args)
    minimal_client = Main()
    minimal_client.run()

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
