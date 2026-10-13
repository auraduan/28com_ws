import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time
import math
import sys
import copy
import os
import glob
import yaml
import shutil 

class ImageSubscriber:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('image_subscriber', anonymous=True)

        # 创建CvBridge实例，用于ROS图像和OpenCV图像之间的转换
        self.bridge = CvBridge()

        # 初始化一个属性，用于保存接收到的OpenCV图像
        self.cv_image = None

        # 订阅图像话题 (根据实际情况修改话题名称，例如 /usb_cam/image_raw 或 /camera/color/image_raw)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)

    def image_callback(self, data):
        try:
            # 将ROS图像消息转换为OpenCV图像
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def get_image(self):
        # 返回最新的OpenCV图像
        return self.cv_image


def CameraCalc(images):
    """
    Args:
        images (nparrary): 不同角度的棋盘图像
    Returns:
        ret (float): 重投影误差
        camera_matrix (nparrary): 相机内参矩阵
        dist_coeffs (nparrary): 相机畸变矩阵
    """

    # 设置棋盘格的大小（内部角点的数目）
    chessboard_size = (5, 5)  # 仿真平台所给出的角点数目为12x8的棋盘格
    # 设置标定板的物理尺寸
    square_size = 0.0272  # 每个方块的实际大小，单位是mm
    # 准备标定板的 3D 坐标，(0,0,0), (1,0,0), (2,0,0)...
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp = objp * square_size
    # 用于存储3D点和2D点的列表
    objpoints = []  # 3D点（真实世界中的空间坐标）
    imgpoints = []  # 2D点（图像中的像素坐标）
    
    gray = None 

    for img_temp in images:
        # 检查图像是否为彩色图像，如果是灰度图像，跳过转换
        if len(img_temp.shape) == 3:  # 如果是彩色图像
            print(f"左图像尺寸={img_temp.shape}， 正在转换")
            gray = cv2.cvtColor(img_temp, cv2.COLOR_BGR2GRAY)
        else:
            gray = img_temp  # 如果已经是灰度图，直接使用
        # 查找棋盘角点
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
        # 如果找到角点，将它们存储起来
        if ret:
            objpoints.append(objp)
            imgpoints.append(corners)
            # 绘制并显示角点
            img_temp = cv2.drawChessboardCorners(img_temp, chessboard_size, corners, ret)
            cv2.imshow('Chessboard Corners', img_temp)
            cv2.waitKey(100)
        else:
            print(f"图像未能检测到角点：图像尺寸={gray.shape}")
    cv2.destroyAllWindows()
    
    # 标定相机，返回相机内参矩阵、畸变系数等
    if gray is not None and len(objpoints) > 0:
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        # ret重投影误差
        print("Reprojection error:", ret)
        # 返回误差、内参矩阵以及去畸变矩阵
        return ret, camera_matrix, dist_coeffs
    else:
        print("未检测到足够的角点进行标定")
        return 100.0, None, None # 返回一个大误差值


def save_to_yaml(camera_matrix, dist_coeffs, filename="cali.yaml"):
    data = {
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": camera_matrix.flatten().tolist()
        },
        "distortion_coefficients": {
            "rows": 1,
            "cols": int(dist_coeffs.size),
            "data": dist_coeffs.flatten().tolist()
        }
    }

    with open(filename, "w") as f:
        yaml.dump(data, f)

    print(f"Calibration saved to {filename}")

def clear_images_folder(directory):
    """清空指定文件夹下的所有文件"""
    print(f"正在清空文件夹: {directory} ...")
    files = glob.glob(os.path.join(directory, '*'))
    for f in files:
        try:
            os.remove(f)
        except OSError as e:
            print(f"Error deleting {f}: {e}")
    print("文件夹已清空。")

def process_calibration(img_list, output_dir):
    """
    统一处理标定结果判断的函数
    """
    print("len of img:", len(img_list), "图像尺寸：", img_list[0].shape)
    
    # 获取返回的误差 ret
    ret, camera_matrix, dist_coeffs = CameraCalc(img_list)

    if camera_matrix is None:
        print("标定失败，未能计算出参数。")
        return

    # 判断误差是否过大
    if ret > 0.28:
        print("="*40)
        print(f"警告：重投影误差为 {ret:.4f}，高于阈值 0.28！")
        print("标定结果不准确。")
        print("正在清空图片文件夹，并删除无效的 yaml 文件...")
        print("请重新调整环境并进行采集标定。")
        print("="*40)
        
        # 1. 清空图片文件夹
        clear_images_folder(output_dir)

        # 2. 删除 cali.yaml 文件 (如果存在)
        yaml_file = "cali.yaml"
        if os.path.exists(yaml_file):
            try:
                os.remove(yaml_file)
                print(f"已删除旧的标定文件: {yaml_file}")
            except OSError as e:
                print(f"删除 {yaml_file} 失败: {e}")
        else:
            print(f"{yaml_file} 不存在，无需删除。")

    else:
        print(f"标定成功！重投影误差: {ret:.4f} (合格)")
        print(f"相机内参矩阵：\n{camera_matrix}")
        print(f"相机畸变矩阵：\n{dist_coeffs}")
        save_to_yaml(camera_matrix, dist_coeffs, "cali.yaml")


def ask_recalibrate():
    """弹出置顶窗口询问是否重新标定"""
    import tkinter as tk
    from tkinter import messagebox

    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    result = messagebox.askyesno("相机标定", "是否重新采集图像进行标定？\n\n选择 [是]：清空图像，重新标定\n选择 [否]：使用已有图像直接标定", parent=root)
    root.destroy()
    return result


def show_info_popup(title, message):
    """显示信息弹窗"""
    import tkinter as tk
    from tkinter import messagebox

    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    messagebox.showinfo(title, message, parent=root)
    root.destroy()


MIN_IMAGES = 20  # 最少需要的图像数量


def main():
    """
    主函数 
    """

    output_dir = 'img'

    # 确保输出文件夹存在
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # ===== 弹窗询问是否重新标定 =====
    recalibrate = ask_recalibrate()

    if recalibrate:
        # 用户选择重新标定 → 清空 img 文件夹
        clear_images_folder(output_dir)
        print("已清空图像文件夹，准备重新采集...")
        start_capture(output_dir)
    else:
        # 用户选择使用已有图像
        existing_images = glob.glob(os.path.join(output_dir, '*.png'))
        num_existing = len(existing_images)

        if num_existing >= MIN_IMAGES:
            print(f"img 文件夹中有 {num_existing} 张图像 (>= {MIN_IMAGES})，直接开始标定...")
            img = []
            for img_path in existing_images:
                img1 = cv2.imread(img_path, 0)  # 直接读取为灰度
                if img1 is not None:
                    img.append(img1)

            if len(img) >= MIN_IMAGES:
                process_calibration(img, output_dir)
            else:
                print(f"有效图像不足 {MIN_IMAGES} 张，需要重新采集。")
                show_info_popup("图像不足", f"有效图像只有 {len(img)} 张，不足 {MIN_IMAGES} 张。\n将清空文件夹并重新采集。")
                clear_images_folder(output_dir)
                start_capture(output_dir)
        else:
            msg = f"img 文件夹中只有 {num_existing} 张图像，不足 {MIN_IMAGES} 张。\n将进入采集模式。"
            print(msg)
            show_info_popup("图像不足", msg)
            start_capture(output_dir)


def start_capture(output_dir):
    """启动 ROS 节点并采集图像"""
    try:
        image_subscriber = ImageSubscriber()
    except rospy.ROSInterruptException:
        print("Exception: ros_image to cv_image")
        return

    rate = rospy.Rate(10)  # 10Hz

    saved_images_count = 0
    max_images = 20  # 要保存的最大图像数

    img = []

    print(f"开始采集图像... (按 's' 保存，上限 {max_images} 张)")
    while not rospy.is_shutdown():
        if saved_images_count < max_images:
            # 获取最新图像
            images = image_subscriber.get_image()

            # 检查是否接收到图像
            if images is not None:
                # 对图像进行处理或其他操作
                cv2.imshow("left", images)

                key = cv2.waitKey(1) & 0xFF  # 获取按键并去掉高位 

                if key == ord('s'):  # 当按下 's' 键时保存图像
                    if saved_images_count < max_images:
                        image_path = os.path.join(output_dir, f'img_{saved_images_count}.png')

                        cv2.imwrite(image_path, images)

                        img.append(images)

                        print(f'Saved {image_path} ({saved_images_count + 1}/{max_images})')
                        saved_images_count += 1

                    else:
                        print("Reached maximum number of saved images.")

        else:
            # 达到最大数量，开始计算外参并进行判断
            process_calibration(img, output_dir)
            break

if __name__ == '__main__':
    main()