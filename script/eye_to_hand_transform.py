#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import PointCloud2
import numpy as np
import cv2
rclpy.init()
node = rclpy.create_node('calib_node')

def callback(msg):
    np_dtype = [('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('b0', '<f4'), ('rgb', '<f4')]
    np_pc = np.frombuffer(msg.data, dtype=np_dtype)
    points = np.expand_dims(np.hstack((np.expand_dims(np_pc['x'],-1), np.expand_dims(np_pc['y'], -1), np.expand_dims(np_pc['z'],-1))), 0)
    points = points.reshape((480,640,3))
    rgb = np.frombuffer(np.ascontiguousarray(np_pc['rgb']).data, dtype=np.uint8)
    rgb = np.expand_dims(rgb,0).reshape(480*640,4)[:,:3]
    rgb = np.expand_dims(rgb,0).reshape(480,640,3)

    def onMouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
          print(f'Target Loc: {points[y,x]}')
    
    cv2.imshow('img', rgb)
    cv2.setMouseCallback('img', onMouse)
    cv2.waitKey(20)

pc_sub = node.create_subscription(PointCloud2, '/camera/depth/color/points', callback, 10)
rclpy.spin(node)