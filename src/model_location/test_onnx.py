#!/usr/bin/env python3
import time
import cv2
import torch
import yaml
import rospy
import numpy as np

# 【注意】如果 ONNX Runtime 报错 np.bool 问题，保留此补丁；否则可以尝试注释掉
try:
    np.bool = np.bool_ 
except AttributeError:
    pass

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

# 【修改 1】导入标准的 YOLO 类 (Ultralytics v8/v11 通用)
from ultralytics import YOLO 

def compute_xyz_from_bbox(x1, y1, x2, y2, fx, fy, cx, cy, W, H, mode="avg"):
    w_px = max(1.0, x2 - x1)
    h_px = max(1.0, y2 - y1)
    u = 0.5 * (x1 + x2)
    v = 0.5 * (y1 + y2)

    Zw = fx * W / w_px
    Zh = fy * H / h_px

    if mode == "w":
        Z = Zw
    elif mode == "h":
        Z = Zh
    else:
        Z = 0.5 * (Zw + Zh)

    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    return X, Y, Z, w_px, h_px, Zw, Zh


class YoloeAutoPoseVisNode:
    def __init__(self):
        rospy.init_node("yoloe_auto_pose_vis_node")

        # ===== ROS 参数 =====
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.vis_topic = rospy.get_param("~vis_topic", "/det/vis_image")
        self.point_topic = rospy.get_param("~point_topic", "/det/xyz")
        self.mode = rospy.get_param("~mode", "avg")
        self.center_offset = float(rospy.get_param("~center_offset", 0.2))
        self.center_offset_z = float(rospy.get_param("~center_offset_z", 0.5))

        # ===== 从 yaml 文件读取内参 =====
        cali_path = "/home/nvidia/28_com_ws/Calibrate/cali.yaml"
        try:
            with open(cali_path, 'r') as f:
                calib_data = yaml.safe_load(f)
            
            K = calib_data["camera_matrix"]["data"]
            self.fx = float(K[0])
            self.cx = float(K[2])
            self.fy = float(K[4])
            self.cy = float(K[5])
            
            if "distortion_coefficients" in calib_data:
                self.dist_coeffs = calib_data["distortion_coefficients"]["data"]
            else:
                self.dist_coeffs = []
                
            self.have_info = True
            rospy.loginfo(f"Loaded calibration from {cali_path}")

        except Exception as e:
            rospy.logerr(f"Failed to load calibration from {cali_path}: {e}")
            self.fx = 834.85
            self.fy = 836.33
            self.cx = 652.57
            self.cy = 358.64
            self.have_info = False

        # ===== YOLOE 参数 (ONNX) =====
        # 【修改 2】默认路径改为 .onnx 文件
        self.model_path = rospy.get_param("~model_path", "./model/best.onnx")
        self.conf_thres = float(rospy.get_param("~conf_thres", 0.45)) # ONNX有时需要调低一点置信度测试
        self.infer_every_n = int(rospy.get_param("~infer_every_n", 1))
        self.max_det = int(rospy.get_param("~max_det", 50))

        # ===== 类别列表 =====
        default_classes = [
            "yellow_black_rectangle_hole"
        ]
        self.all_class_names = rospy.get_param("~all_class_names", default_classes)

        # ===== 物体真实尺寸表 =====
        self.size_map = {
            "yellow_black_rectangle_black_hole": (0.66, 1.00),
            "yellow_black_rectangle_hole": (0.66, 1.00),
            "rd40": (0.23, 0.40),
        }

        # ===== 加载 ONNX 模型 =====
        rospy.loginfo(f"Loading ONNX Model: {self.model_path}")
        
        # 【修改 3】使用 YOLO 类加载 ONNX 模型
        # task='detect' 是为了告诉 ultralytics 这是一个检测任务
        try:
            self.model = YOLO(self.model_path, task='detect')
            rospy.loginfo(f"ONNX Model loaded successfully. Conf threshold: {self.conf_thres}")
        except Exception as e:
            rospy.logerr(f"Failed to load ONNX model: {e}")
            exit(1)

        # ===== ROS I/O =====
        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_frame_id = "camera"
        self.frame_count = 0

        # ===== FPS 统计变量 =====
        self.fps_start_time = time.time()
        self.fps_frame_count = 0
        self.current_fps = 0.0

        self.sub_img = rospy.Subscriber(self.image_topic, Image, self.on_image, queue_size=10)
        self.pub_vis = rospy.Publisher(self.vis_topic, Image, queue_size=10)
        self.pub_point = rospy.Publisher(self.point_topic, PointStamped, queue_size=50)

        self.timer = rospy.Timer(rospy.Duration(0.01), self.on_timer)

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if msg.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            self.latest_frame = frame
            self.latest_frame_id = msg.header.frame_id if msg.header.frame_id else "camera"
        except Exception as e:
            rospy.logwarn(f"cv_bridge failed: {e}")

    def clsid_to_name(self, cls_id: int) -> str:
        # 【修改 4】优化类别名称获取
        # 优先使用模型自带的 names，如果没有则使用参数表
        if hasattr(self.model, 'names') and cls_id in self.model.names:
            return self.model.names[cls_id]
        elif 0 <= cls_id < len(self.all_class_names):
            return self.all_class_names[cls_id]
        return str(cls_id)

    def publish_point(self, frame_id: str, X, Y, Z, name: str):
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.point.x = float(X)
        msg.point.y = float(Y)
        msg.point.z = float(Z)
        self.pub_point.publish(msg)

    def on_timer(self, event):
        if self.latest_frame is None:
            return

        self.frame_count += 1
        frame = self.latest_frame
        disp = frame.copy()

        dets = None
        infer_ms = 0.0

        if (self.frame_count % max(1, self.infer_every_n)) == 0:
            t0 = time.time()
            
            # 【修改 5】ONNX 推理参数调整
            # device='0' 使用 GPU，device='cpu' 使用 CPU
            # half=False ONNX 通常使用 FP32，除非你导出了 FP16 的 ONNX
            results = self.model.predict(
                source=frame,
                conf=self.conf_thres,
                imgsz=640,
                verbose=False,
                max_det=self.max_det,
                device='0',  # 如果在 Jetson 上，建议显式指定 '0' (GPU) 或 'cpu'
                half=False 
            )
            infer_ms = (time.time() - t0) * 1000.0

            if results and results[0].boxes is not None and len(results[0].boxes) > 0:
                b = results[0].boxes
                # 【注意】ONNX 结果也是 Tensor，这里处理逻辑和 Engine 是一样的
                xyxy = b.xyxy.detach().cpu().numpy()
                conf = b.conf.detach().cpu().numpy()
                cls  = b.cls.detach().cpu().numpy().astype(int)
                dets = list(zip(xyxy, conf, cls))
            
            # ===== FPS 计算 =====
            self.fps_frame_count += 1
            curr_time = time.time()
            elapsed = curr_time - self.fps_start_time
            if elapsed >= 1.0:
                self.current_fps = self.fps_frame_count / elapsed
                rospy.loginfo(f"FPS: {self.current_fps:.2f} | Infer Time: {infer_ms:.1f}ms")
                self.fps_frame_count = 0
                self.fps_start_time = curr_time

        # 绘制检测结果
        info = f"FPS: {self.current_fps:.1f} Infer: {infer_ms:.1f}ms (ONNX)"
        cv2.putText(disp, info, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        if dets:
            for (x1, y1, x2, y2), c, cls_id in dets:
                name = self.clsid_to_name(int(cls_id))
                
                bh = max(1.0, float(y2) - float(y1))
                bw = max(1.0, float(x2) - float(x1))
                if name == "yellow_black_rectangle_black_hole":
                     if bh > 3.0 * bw or 0.7 * bh < bw:
                         continue

                # 画框
                cv2.rectangle(disp, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

                # 测距
                if name in self.size_map:
                    W, H = self.size_map[name]
                    X, Y, Z, _, _, _, _ = compute_xyz_from_bbox(
                        float(x1), float(y1), float(x2), float(y2),
                        self.fx, self.fy, self.cx, self.cy, W, H, mode=self.mode
                    )

                    u_center = (x1 + x2) / 2.0
                    ratio = min(1.0, abs(u_center - 320) / 320.0)
                    current_offset = ratio * self.center_offset
                    
                    if X < 0: X -= current_offset
                    elif X > 0: X += current_offset
                    Z += self.center_offset_z

                    label_txt = f"{name} {c:.2f} Z={Z:.2f}m"
                    self.publish_point(self.latest_frame_id, X, Y, Z, name)
                else:
                    label_txt = f"{name} {c:.2f}"

                cv2.putText(disp, label_txt, (int(x1), max(20, int(y1)-8)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 1. 发布到 ROS
        if self.pub_vis.get_num_connections() > 0:
            try:
                vis_msg = self.bridge.cv2_to_imgmsg(disp, encoding="bgr8")
                vis_msg.header.frame_id = self.latest_frame_id
                self.pub_vis.publish(vis_msg)
            except Exception as e:
                pass
        
        # 2. 本地显示窗口
        cv2.imshow("ONNX Detection", disp)
        cv2.waitKey(1)

def main():
    node = YoloeAutoPoseVisNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()