#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import yaml
import os

def load_params_from_yaml(path):
    """从 yaml 文件读取相机内参和畸变系数"""
    if not os.path.exists(path):
        raise FileNotFoundError(f"找不到配置文件: {path}")
    
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    
    # 解析 camera_matrix
    cm_data = data['camera_matrix']
    camera_matrix = np.array(cm_data['data'], dtype=np.float32).reshape(cm_data['rows'], cm_data['cols'])
    
    # 解析 distortion_coefficients
    dist_data = data['distortion_coefficients']
    dist_coeffs = np.array(dist_data['data'], dtype=np.float32).reshape(dist_data['rows'], dist_data['cols'])
    
    return camera_matrix, dist_coeffs

class ArUcoToLidar:
    def __init__(self):
        rospy.init_node('aruco_lidar_display_node', anonymous=True)
        
        # 二维码实际边长 (单位: 米)
        self.marker_size = 0.1999 
        yaml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cali.yaml")
        
        try:
            self.camera_matrix, self.dist_coeffs = load_params_from_yaml(yaml_path)
            rospy.loginfo("Camera parameters loaded successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to load parameters: {e}")
            exit()

        # 初始化 ArUco 检测器
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        
        # 相机相对于雷达的物理偏移量 (相机在雷达前0.075m，左0.065m)
        self.off_x = 0.075 # 前 (Forward)
        self.off_y = 0.065 # 左 (Left)

    def image_callback(self, data):
        try:
            # ROS 图像转 OpenCV 图像
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError:
            return

        corners, ids, _ = self.detector.detectMarkers(frame)

        if ids is not None:
            # 绘制二维码边框
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            # 定义码坐标系 3D 点
            obj_points = np.array([
                [-self.marker_size/2,  self.marker_size/2, 0],
                [ self.marker_size/2,  self.marker_size/2, 0],
                [ self.marker_size/2, -self.marker_size/2, 0],
                [-self.marker_size/2, -self.marker_size/2, 0]
            ], dtype=np.float32)

            for i in range(len(ids)):
                # 解算位姿
                success, rvec, tvec = cv2.solvePnP(obj_points, corners[i], self.camera_matrix, self.dist_coeffs)
                
                if success:
                    # 绘制 3D 坐标轴
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

                    # 1. 获取相机坐标系下的位置 (X:右, Y:下, Z:前)
                    tx, ty, tz = tvec[0][0], tvec[1][0], tvec[2][0]

                    # 2. 转换到雷达坐标系 (前 X, 左 Y, 上 Z)
                    lidar_x = tz + self.off_x
                    lidar_y = -tx + self.off_y
                    lidar_z = -ty

                    # 3. 按照要求布局绘制文本
                    color = (0, 255, 0) # 绿色
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    scale = 1.2         # 增大字体
                    thickness = 3       # 增加粗细

                    # 每一行的高度间隔设为 50 像素
                    cv2.putText(frame, "Offset from Lidar Center:", (30, 60), font, scale, color, thickness)
                    cv2.putText(frame, f"X: {lidar_x:.3f} m", (30, 110), font, scale, color, thickness)
                    cv2.putText(frame, f"Y: {lidar_y:.3f} m", (30, 160), font, scale, color, thickness)
                    cv2.putText(frame, f"Z: {lidar_z:.3f} m", (30, 210), font, scale, color, thickness)

        cv2.imshow("Lidar Center Calibration", frame)
        cv2.waitKey(1)

def main():
    aligner = ArUcoToLidar()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()