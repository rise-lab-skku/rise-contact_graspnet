#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

import tensorflow.compat.v1 as tf
tf.disable_eager_execution()
physical_devices = tf.config.experimental.list_physical_devices('GPU')
tf.config.experimental.set_memory_growth(physical_devices[0], True)

# BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# sys.path.append(os.path.join(BASE_DIR))
sys.path.append("/home/ros_ws/src/rise_contact_graspnet2/contact_graspnet")
print(sys.path)
import config_utils
from contact_grasp_estimator import GraspEstimator

from cv_bridge import CvBridge
from rise_contact_msg2.msg import ContactGrasp
from rise_contact_msg2.srv import ContactGraspNetPlanner


class CgpMain(Node):
    def __init__(self):
        # init node
        super().__init__('contact_graspnet_planner')
        self.get_logger().info(f"Contact GraspNet Planner is launched with Python {sys.version}")

        # get arguments from the ros parameter server
        # ckpt_dir = self.get_parameter('ckpt_dir').get_parameter_value().string_value
        ckpt_dir_param = "/home/ros_ws/src/rise_contact_graspnet2/checkpoints/scene_test_2048_bs3_hor_sigma_001"
        z_min_param = 0.2
        z_max_param = 1.8
        self.z_range = np.array([z_min_param, z_max_param])
        self.local_regions = False
        self.filter_grasps = False
        self.skip_border_objects = False
        self.forward_passes = 1
        self.segmap_id = 0

        arg_configs = []
        # get global config
        global_config = config_utils.load_config(
            ckpt_dir_param,
            batch_size=self.forward_passes,
            arg_configs=arg_configs)

        # print config
        self.get_logger().info(f"global_config: {global_config}")

        # Build the model
        self.grasp_estimator = GraspEstimator(global_config)
        self.grasp_estimator.build_network()

        # Add ops to save and restore all the variables.
        saver = tf.train.Saver(save_relative_paths=True)

        # Create a session
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        config.allow_soft_placement = True
        self.sess = tf.Session(config=config)

        # Load weights
        self.grasp_estimator.load_weights(self.sess, saver, ckpt_dir_param, mode='test')

        # ros cv bridge
        self.cv_bridge = CvBridge()

        # ros2 service
        self.srv = self.create_service(ContactGraspNetPlanner, 'grasp_planner', self.plan_grasp_handler)
        self.get_logger().info("Start Contact-GraspNet grasp planner.")

    def plan_grasp_handler(self, req, grasp_resp):
        #############
        # Get Input #
        #############
        # exmaple data format get input from npy
        # pc_segments = {}
        # segmap, rgb, depth, cam_K, pc_full, pc_colors = load_available_input_data(p, K=K)
        # segmap : (720, 1280), [0,12]
        # rgb : (720, 1280, 3)
        # cam_K : fx=912.72143555, fy=912.7409668, cx-649.00366211, cy=363.2547192
        # pc_full : None
        # pc_colors : None

        # unpack request massage
        color_im, depth_im, segmask, camera_intr = self.read_images(req)

        if np.mean(segmask) == -1:
            segmask = None
            self.get_logger().warn('No segmentation mask. Generate grasp with full scene.')
            if (self.local_regions or self.filter_grasps):
                self.get_logger().error('For the invalid segmentation mask, local_regions or filter_grasp should be False.')
        else:
            self.get_logger().info('Num instance id: {}, filter_grasp: {}, local_region: {}'.format(
                np.max(segmask), self.filter_grasps, self.local_regions))

        # Convert depth image to point clouds
        self.get_logger().info('Converting depth to point cloud(s)...')
        pc_full, pc_segments, pc_colors = self.grasp_estimator.extract_point_clouds(
            depth=depth_im,
            segmap=segmask,
            K=camera_intr,
            rgb=color_im,
            skip_border_objects=self.skip_border_objects,
            z_range=self.z_range,
            )

        #############
        # Gen Grasp #
        #############
        # if fc_full, key=-1
        # pred_grasps_cam : dict.keys=[1, num_instance], TF(4x4) on camera coordinate
        # scores : dict.keys=[1, num_instance]
        # contact_pts : dict.keys=[1, num_instance], c.p(3) on camera coordinate

        # Generate grasp
        start_time = time.time()
        self.get_logger().info('Start to genterate grasps')
        pred_grasps_cam, scores, contact_pts, _ = self.grasp_estimator.predict_scene_grasps(
            self.sess,
            pc_full,
            pc_segments=pc_segments,
            local_regions=self.local_regions,
            filter_grasps=self.filter_grasps,
            forward_passes=self.forward_passes,
            )

        # Generate grasp responce msg
        # grasp_resp = response()
        for instance_id in pred_grasps_cam.keys():
            grasp_score_cp = zip(pred_grasps_cam[instance_id], scores[instance_id], contact_pts[instance_id])
            for grasp, score, contact_pt in grasp_score_cp:
                grasp_msg = self.get_grasp_msg(instance_id, grasp, score, contact_pt)
                grasp_resp.grasps.append(grasp_msg)
        self.get_logger().info('Generate grasp {} took {}s'.format(len(grasp_resp.grasps), time.time() - start_time))

        return grasp_resp

    def read_images(self, req):
        """Reads images from a ROS service request.

        Parameters
        ---------
        req: :obj:`ROS ServiceRequest`
            ROS ServiceRequest for grasp planner service.
        """
        # Get the raw depth and color images as ROS `Image` objects.
        raw_color = req.color_image
        raw_depth = req.detph_image
        raw_segmask = req.segmask

        # Get the raw camera info as ROS `CameraInfo`.
        raw_camera_info = req.camera_info

        camera_intr = np.array([raw_camera_info.k]).reshape((3, 3))

        try:
            color_im = self.cv_bridge.imgmsg_to_cv2(raw_color)
            depth_im = self.cv_bridge.imgmsg_to_cv2(raw_depth)
            segmask = self.cv_bridge.imgmsg_to_cv2(raw_segmask)
        except NotImplementedError as e:
            self.get_logger().error(e)

        return (color_im, depth_im, segmask, camera_intr)

    def get_grasp_msg(self, id, tf_mat, score, contact_pt):
        grasp_msg = ContactGrasp()

        # set instance id
        grasp_msg.id = float(id)

        # convert tf matrix to pose msg
        rot = R.from_matrix(tf_mat[0:3, 0:3])
        quat = rot.as_quat()
        grasp_msg.pose.position.x = float(tf_mat[0, 3])
        grasp_msg.pose.position.y = float(tf_mat[1, 3])
        grasp_msg.pose.position.z = float(tf_mat[2, 3])
        grasp_msg.pose.orientation.x = float(quat[0])
        grasp_msg.pose.orientation.y = float(quat[1])
        grasp_msg.pose.orientation.z = float(quat[2])
        grasp_msg.pose.orientation.w = float(quat[3])

        # conver contact point to msg
        grasp_msg.contact_point.x = float(contact_pt[0])
        grasp_msg.contact_point.y = float(contact_pt[1])
        grasp_msg.contact_point.z = float(contact_pt[2])

        # get grasp msg
        grasp_msg.score = float(score)

        return grasp_msg



def main(args=None):
    rclpy.init(args=args)
    minimal_service = CgpMain()
    rclpy.spin(minimal_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()