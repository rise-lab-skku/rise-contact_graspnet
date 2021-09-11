#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import argparse

import numpy as np
from scipy.spatial.transform import Rotation as R

import tensorflow.compat.v1 as tf
tf.disable_eager_execution()
physical_devices = tf.config.experimental.list_physical_devices('GPU')
tf.config.experimental.set_memory_growth(physical_devices[0], True)

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(BASE_DIR))
import config_utils
from data import load_available_input_data
from contact_grasp_estimator import GraspEstimator
from visualization_utils import visualize_grasps, show_image

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from contact_graspnet_planner.srv import ContactGraspNetPlanner


if __name__ == '__main__':
    # argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--ckpt_dir', default='checkpoints/scene_test_2048_bs3_hor_sigma_001', help='Log dir [default: checkpoints/scene_test_2048_bs3_hor_sigma_001]')
    parser.add_argument('--np_path', default='test_data/7.npy', help='Input data: npz/npy file with keys either "depth" & camera matrix "K" or just point cloud "pc" in meters. Optionally, a 2D "segmap"')
    FLAGS = parser.parse_args()

    # init ros
    rospy.init_node('grasp_planner_client')

    # get sample data
    sample_data_path = FLAGS.np_path
    segmap, rgb, depth, cam_K, pc_full, pc_colors = load_available_input_data(sample_data_path, K=None)
    rospy.loginfo("Load exmaple datas: {}".format(sample_data_path))

    # Convert depth image to point clouds --> need `GraspEstimator`
    global_config = config_utils.load_config(FLAGS.ckpt_dir, batch_size=1, arg_configs=[])
    grasp_estimator = GraspEstimator(global_config)

    rospy.loginfo('Converting depth to point cloud(s)...')
    segmap = None
    pc_full, pc_segments, pc_colors = grasp_estimator.extract_point_clouds(
        depth=depth,
        segmap=segmap,
        K=cam_K,
        rgb=rgb,
        skip_border_objects=False,
        z_range=[0.2, 1.8],
        )

    # conver to ros input data
    cv_bridge = CvBridge()
    rgb_img_msg = cv_bridge.cv2_to_imgmsg(np.array(rgb))
    depth_img_msg = cv_bridge.cv2_to_imgmsg(np.array(depth))
    segmask = cv_bridge.cv2_to_imgmsg(np.zeros_like(depth))
    camera_intr = CameraInfo()
    camera_intr.K = np.array(cam_K).reshape(9)

    # request service to server
    rospy.loginfo('Start grasp_planner_client')
    service_name = 'grasp_planner'
    rospy.loginfo('Wait for the grasp_plnnaer_server')
    rospy.wait_for_service(service_name)
    try:
        grasp_planner = rospy.ServiceProxy(service_name, ContactGraspNetPlanner)
        resp = grasp_planner(
            rgb_img_msg,
            depth_img_msg,
            camera_intr,
            segmask,
            )
        rospy.loginfo("Request Contact-GraspNet grasp planning")
    except rospy.ServiceException as e:
        print("Service call failed: {}".format(e))

    # visulaize grasp
    pred_grasps_cam = {}
    scores = {}
    contact_pts = {}

    grasp_list = []
    scores_list = []
    contact_pts_list = []
    for grasp in resp.grasps:
        pose_msg = grasp.pose
        score = grasp.score
        contact_pt_msg = grasp.contact_point

        # get transform matrix
        tf_mat = np.zeros((4, 4), dtype=np.float64)
        quat = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
        rot_mat = R.from_quat(quat)
        tf_mat[0:3, 0:3] = rot_mat.as_matrix()
        tf_mat[0, 3] = pose_msg.position.x
        tf_mat[1, 3] = pose_msg.position.y
        tf_mat[2, 3] = pose_msg.position.z

        # get contact point as numpy
        contact_pt = np.array([contact_pt_msg.x, contact_pt_msg.y, contact_pt_msg.z])

        # append to list
        grasp_list.append(tf_mat)
        scores_list.append(score)
        contact_pts_list.append(contact_pt)

    # convert list to numpy array
    grasp_list = np.array(grasp_list)
    scores_list = np.array(scores_list)
    contact_pts_list = np.array(contact_pts_list)

    # put on the dictionary
    pred_grasps_cam[-1] = grasp_list
    scores[-1] = scores_list
    contact_pts[-1] = contact_pts_list

    show_image(rgb, None)
    visualize_grasps(pc_full, pred_grasps_cam, scores, plot_opencv_cam=True, pc_colors=pc_colors)
