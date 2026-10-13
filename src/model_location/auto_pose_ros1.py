#!/usr/bin/env python3
import time
import cv2
import torch
import yaml
import rospy
import sys
import tkinter as tk
from tkinter import messagebox
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from ultralytics import YOLOE

def show_error_popup(message):
    """显示一个始终置顶的错误弹窗"""
    root = tk.Tk()
    root.withdraw()  # 隐藏主窗口
    root.attributes("-topmost", True)  # 窗口置顶
    messagebox.showerror("错误", message, parent=root)
    root.destroy()

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
        self.point_topic = rospy.get_param("~point_topic", "/det/xyz")  # 不想要就不用订阅/echo
        self.mode = rospy.get_param("~mode", "avg")              # avg / w / h
        self.center_offset = float(rospy.get_param("~center_offset", 0.55))
        self.center_offset_z = float(rospy.get_param("~center_offset_z", 0.9))
        self.edge_margin = int(rospy.get_param("~edge_margin", 150))  # 边缘过滤阈值(像素)

        # ===== 从 yaml 文件读取内参 =====
        cali_path = "../Calibrate/cali.yaml"
        try:
            with open(cali_path, 'r') as f:
                calib_data = yaml.safe_load(f)
                
            # camera_matrix -> data -> [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            # 这里的 YAML 格式是 flatten 的 3x3 矩阵
            K = calib_data["camera_matrix"]["data"]
            self.fx = float(K[0])
            self.cx = float(K[2])
            self.fy = float(K[4])
            self.cy = float(K[5])
            
            # distortion_coefficients -> data
            if "distortion_coefficients" in calib_data:
                self.dist_coeffs = calib_data["distortion_coefficients"]["data"]
            else:
                self.dist_coeffs = []
                
            self.have_info = True
            rospy.loginfo(f"Loaded calibration from {cali_path}")
            rospy.loginfo(f"fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

        except Exception as e:
            err_msg = f"缺少标定文件！\n\n路径: {cali_path}\n错误信息: {e}\n\n请先前往 'Calibrate' 文件夹运行标定程序！"
            rospy.logerr(err_msg)
            # 弹出图形化提示窗口
            show_error_popup(err_msg)
            # 退出程序
            sys.exit(1)

        # ===== YOLOE 参数 =====
        self.model_path = rospy.get_param("~model_path", "./model/best.pt")
        self.conf_thres = float(rospy.get_param("~conf_thres", 0.85))
        self.use_half = bool(rospy.get_param("~half", False))          # Orin 上 CUDA 通常 True 更快；如遇报错改 False
        self.infer_every_n = int(rospy.get_param("~infer_every_n", 1))    # 每 N 帧推理一次（降负载）
        self.max_det = int(rospy.get_param("~max_det", 50))

        # ===== 类别列表（用于 cls_id -> name）=====
        default_classes = [
            "yellow_black_rectangle_hole","person","bicycle","car","motorcycle","airplane","bus","train","truck","boat","traffic light","fire hydrant",
            "stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow","elephant","bear","zebra",
            "giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
            "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle","wine glass","cup",
            "fork","knife","spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza",
            "donut","cake","chair","couch","potted plant","bed","dining table","toilet","tv","laptop","mouse","remote",
            "keyboard","cell phone","microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissors",
            "teddy bear","hair drier","toothbrush","circle","hole","balloon","rd40","yellow_black_rectangle_black_hole"  
        ]
        self.all_class_names = rospy.get_param("~all_class_names", default_classes)

        # ===== 物体真实尺寸表（米）=====
        # 你给的：
        # yellow_black_rectangle_black_hole: high 103cm width 66cm
        # rd40: high 40cm width 23cm
        self.size_map = {
            "yellow_black_rectangle_black_hole": (0.68, 1.03),   # (W, H)
            "yellow_black_rectangle_hole": (0.68, 1.03),
            "rd40": (0.23, 0.40),
        }

        # ===== 设备 & 加速 =====
        self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        if self.device == "cuda:0":
            torch.backends.cudnn.enabled = True
            torch.backends.cudnn.benchmark = True
            torch.backends.cudnn.deterministic = False
            torch.backends.cuda.matmul.allow_tf32 = True

        # ===== 加载 YOLOE =====
        rospy.loginfo(f"Loading YOLOE model: {self.model_path}")
        self.model = YOLOE(self.model_path)
        self.model.to(self.device)
        self.model.eval()

        # YOLOE 文本特征（如果失败也不影响普通权重推理）
        try:
            text_pe = self.model.get_text_pe(self.all_class_names)
            self.model.set_classes(self.all_class_names, text_pe)
            rospy.loginfo(f"set_classes OK, classes={len(self.all_class_names)}")
        except Exception as e:
            rospy.logwarn(f"set_classes skipped: {e}")

        try:
            for p in self.model.model.parameters():
                p.requires_grad = False
        except Exception:
            pass

        rospy.loginfo(f"Device={self.device}, half={self.use_half}, conf={self.conf_thres}")

        # ===== ROS I/O =====
        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_frame_id = "camera"
        self.frame_count = 0

        self.sub_img = rospy.Subscriber(self.image_topic, Image, self.on_image, queue_size=10)
        # self.pub_vis = rospy.Publisher(self.vis_topic, Image, queue_size=10)
        self.pub_point = rospy.Publisher(self.point_topic, PointStamped, queue_size=50)

        self.timer = rospy.Timer(rospy.Duration(0.06), self.on_timer)  # ~33Hz

    def on_image(self, msg: Image):
        try:
            # Use passthrough to avoid cv_bridge trying to do fancy conversions
            # that might trigger 'Compressed Depth Image Transport' errors in some environments
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
            # Handle common encodings manually
            if msg.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif msg.encoding == "bgr8":
                pass
            # specific check for other formats if needed, but usually rgb8/bgr8 are the main ones on Jetson
            
        except Exception as e:
            rospy.logwarn(f"cv_bridge failed: {e}")
            return

        if frame is None or frame.size == 0:
            return
        self.latest_frame = frame
        self.latest_frame_id = msg.header.frame_id if msg.header.frame_id else "camera"

    def clsid_to_name(self, cls_id: int) -> str:
        if 0 <= cls_id < len(self.all_class_names):
            return self.all_class_names[cls_id]
        return str(cls_id)

    def publish_point(self, frame_id: str, X, Y, Z, name: str):
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.point.x = float(X)
        msg.point.y = float(Y)
        msg.point.z = float(Z)
        # 这里 PointStamped 没地方写 name；如你需要 name，后续可改成 Marker 或自定义 msg
        self.pub_point.publish(msg)

    def on_timer(self, event):
        if self.latest_frame is None:
            return

        self.frame_count += 1
        frame = self.latest_frame
        disp = frame.copy()

        # 每 N 帧推理一次
        dets = None
        infer_ms = None
        if (self.frame_count % max(1, self.infer_every_n)) == 0:
            t0 = time.time()
            results = self.model.predict(
                source=frame,
                conf=self.conf_thres,
                device=self.device,
                half=(self.use_half and self.device.startswith("cuda")),
                verbose=False,
                max_det=self.max_det
            )
            infer_ms = (time.time() - t0) * 1000.0

            if results and results[0].boxes is not None and len(results[0].boxes) > 0:
                b = results[0].boxes
                xyxy = b.xyxy.detach().cpu().numpy()
                conf = b.conf.detach().cpu().numpy()
                cls  = b.cls.detach().cpu().numpy().astype(int)
                dets = list(zip(xyxy, conf, cls))

        # 画基础信息
        info = f"fx={self.fx:.1f} fy={self.fy:.1f} cx={self.cx:.1f} cy={self.cy:.1f}"
        info += " (CameraInfo)" if self.have_info else " (default)"
        if infer_ms is not None:
            info += f"  infer={infer_ms:.1f}ms"
        cv2.putText(disp, info, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 画检测框 +（若有尺寸）算 XYZ
        if dets:
            for (x1, y1, x2, y2), c, cls_id in dets:
                name = self.clsid_to_name(int(cls_id))
                x1, y1, x2, y2 = float(x1), float(y1), float(x2), float(y2)
                
                 # 计算宽高（像素）
                bw = max(1.0, x2 - x1)
                bh = max(1.0, y2 - y1)

                # ✅ 过滤：yellow_black_rectangle_black_hole 中 高 > 3 * 宽 or 0.7*高 < 宽 的框直接丢弃
                if name in ("yellow_black_rectangle_black_hole"):
                    if bh > 3.0 *bw or 0.7 *bh < bw:
                        continue
                
                # ✅ 中心过滤：只有当检测框中心靠近图像水平中心时才有效
                # 确保目标在画面中心区域，测距更准确
                img_h, img_w = frame.shape[:2]
                center_x = (x1 + x2) / 2.0
                img_center_x = img_w / 2.0
                
                # 检测框中心必须在图像中心附近 (中心 ± edge_margin)
                if abs(center_x - img_center_x) > self.edge_margin:
                    # rospy.loginfo_throttle(1.0, f"Discarded: {name} center_x={center_x:.0f}, img_center={img_center_x:.0f}")
                    continue

                # 画框
                cv2.rectangle(disp, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                
                # rospy.loginfo(f"Detected: {name}")

                # 如果该类别有真实尺寸，就算 XYZ
                if name in self.size_map:
                    W, H = self.size_map[name]
                    X, Y, Z, w_px, h_px, Zw, Zh = compute_xyz_from_bbox(
                        x1, y1, x2, y2,
                        self.fx, self.fy, self.cx, self.cy,
                        W, H,
                        mode=self.mode
                    )

                    # Apply variable offset correction based on distance from center
                    # Ratio: 0 at center, 1 at edge (approx cx)
                    # u_center = (x1 + x2) / 2.0
                    # dist_px = abs(u_center - 320)
                    # ratio = dist_px / 320
                    # if ratio > 1.0: 
                    #     ratio = 1.0

                    # # ratio = 1.0
                    # current_offset = ratio * self.center_offset

                    # # X -= current_offset
                    # if X < 0:
                    #     X -= current_offset
                    #     # 修正Z轴偏移，使目标点往后移动0.1m
                    #     Z += self.center_offset_z

                    # elif X > 0:
                    #     # X += current_offset
                    #     # 修正Z轴偏移，使目标点往后移动0.1m
                    #     Z += self.center_offset_z

                    Z += self.center_offset_z

                    label = f"{name} {c:.2f} XYZ=({X:.2f},{Y:.2f},{Z:.2f})m"
                    self.publish_point(self.latest_frame_id, X, Y, Z, name)
                else:
                    label = f"{name} {c:.2f}"

                # 写字
                ty = max(20, int(y1) - 8)
                cv2.putText(disp, label, (int(x1), ty),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(disp, "No detections", (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        


        # 发布可视化图像话题
        # Display using cv2
        cv2.imshow("YOLO Visualization", disp)
        cv2.waitKey(1)


def main():
    node = YoloeAutoPoseVisNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()



