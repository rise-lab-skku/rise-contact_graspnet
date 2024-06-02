#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import argparse

import numpy as np
from scipy.spatial.transform import Rotation as R


sys.path.append("/home/ros_ws/src/rise_contact_graspnet2/contact_graspnet")
from data import load_available_input_data

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from rise_contact_msg2.srv import ContactGraspNetPlanner


class Main(Node):
    def __init__(self):
        super().__init__('grasp_planner_client')

        # get sample data
        sample_data_path = "/home/ros_ws/src/rise_contact_graspnet2/test_data/7.npy"
        segmap, rgb, depth, cam_K, pc_full, pc_colors = load_available_input_data(sample_data_path, K=None)
        self.get_logger().info("Load exmaple datas: {}".format(sample_data_path))

        # conver to ros input data
        cv_bridge = CvBridge()
        rgb_img_msg = cv_bridge.cv2_to_imgmsg(np.array(rgb))
        depth_img_msg = cv_bridge.cv2_to_imgmsg(np.array(depth))
        segmask = cv_bridge.cv2_to_imgmsg(np.array(segmap))
        camera_intr = CameraInfo()
        camera_intr.k = np.array(cam_K).reshape(9)

        # request service to server
        self.get_logger().info('Start grasp_planner_client')

        self.cli = self.create_client(ContactGraspNetPlanner, 'grasp_planner')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ContactGraspNetPlanner.Request()

        # Request
        resp_result = self.send_request(rgb_img_msg, depth_img_msg, segmask, camera_intr)
        self.get_logger().info("Get {} grasps from the server.".format(len(resp_result.grasps)))

    def send_request(self, rgb_img_msg, depth_img_msg, segmask, camera_intr):
        self.get_logger().info("Request Contact-GraspNet grasp planning")

        self.req.color_image = rgb_img_msg
        self.req.detph_image = depth_img_msg
        self.req.camera_info = camera_intr
        self.req.segmask = segmask
        self.resp = self.cli.call_async(self.req)


        rclpy.spin_until_future_complete(self, self.resp)
        return self.resp.result()


def main(args=None):
    rclpy.init(args=args)
    minimal_client = Main()

    # response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    # minimal_client.get_logger().info(
    #     'Result of add_two_ints: for %d + %d = %d' %
    #     (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    # rclpy.spin(minimal_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    print("GOOOO HOME")
