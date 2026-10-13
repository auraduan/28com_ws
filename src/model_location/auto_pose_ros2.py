#!/usr/bin/env python3
import time
import cv2
import torch

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

from ultralytics import YOLOE


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


class YoloeAutoPoseVisNode(Node):
    def __init__(self):
        super().__init__("yoloe_auto_pose_vis_node")

        # ===== ROS 参数 =====
        self.declare_parameter("image_topic", "/camera/image")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("vis_topic", "/det/vis_image")
        self.declare_parameter("point_topic", "/det/xyz")  # 不想要就不用订阅/echo
        self.declare_parameter("mode", "avg")              # avg / w / h

        # 默认内参（订阅不到 camera_info 就用这组）
        #矫正后的fx fy cx cy分别为-598.0277099609375 -770.2853393554688 -650.1595705969376 -357.0982532765993
        self.declare_parameter("fx", 834.853491)  
        self.declare_parameter("fy", 836.330079)
        self.declare_parameter("cx", 652.571199)
        self.declare_parameter("cy", 358.648036)


        # ===== YOLOE 参数 =====
        self.declare_parameter("model_path", "/home/unitree/YOLOE_WS/src/YOLOE/train662/weights/best.pt")
        self.declare_parameter("conf_thres", 0.5)
        self.declare_parameter("half", False)          # Orin 上 CUDA 通常 True 更快；如遇报错改 False
        self.declare_parameter("infer_every_n", 1)    # 每 N 帧推理一次（降负载）
        self.declare_parameter("max_det", 50)

        # ===== 类别列表（用于 cls_id -> name）=====
        self.declare_parameter("all_class_names", [
            "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat","traffic light","fire hydrant",
            "stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow","elephant","bear","zebra",
            "giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
            "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle","wine glass","cup",
            "fork","knife","spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza",
            "donut","cake","chair","couch","potted plant","bed","dining table","toilet","tv","laptop","mouse","remote",
            "keyboard","cell phone","microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissors",
            "teddy bear","hair drier","toothbrush","circle","hole","balloon","rd40","yellow_black_rectangle_black_hole"  
        ])

        # ===== 读取参数 =====
        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.vis_topic = self.get_parameter("vis_topic").value
        self.point_topic = self.get_parameter("point_topic").value
        self.mode = self.get_parameter("mode").value

        self.fx = float(self.get_parameter("fx").value)
        self.fy = float(self.get_parameter("fy").value)
        self.cx = float(self.get_parameter("cx").value)
        self.cy = float(self.get_parameter("cy").value)
        self.have_info = False

        self.model_path = self.get_parameter("model_path").value
        self.conf_thres = float(self.get_parameter("conf_thres").value)
        self.use_half = bool(self.get_parameter("half").value)
        self.infer_every_n = int(self.get_parameter("infer_every_n").value)
        self.max_det = int(self.get_parameter("max_det").value)

        self.all_class_names = list(self.get_parameter("all_class_names").value)

        # ===== 物体真实尺寸表（米）=====
        # 你给的：
        # yellow_black_rectangle_black_hole: high 103cm width 66cm
        # rd40: high 40cm width 23cm
        self.size_map = {
            "yellow_black_rectangle_black_hole": (0.66, 1.03),   # (W, H)
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
        self.get_logger().info(f"Loading YOLOE model: {self.model_path}")
        self.model = YOLOE(self.model_path)
        self.model.to(self.device)
        self.model.eval()

        # YOLOE 文本特征（如果失败也不影响普通权重推理）
        try:
            text_pe = self.model.get_text_pe(self.all_class_names)
            self.model.set_classes(self.all_class_names, text_pe)
            self.get_logger().info(f"set_classes OK, classes={len(self.all_class_names)}")
        except Exception as e:
            self.get_logger().warn(f"set_classes skipped: {e}")

        try:
            for p in self.model.model.parameters():
                p.requires_grad = False
        except Exception:
            pass

        self.get_logger().info(f"Device={self.device}, half={self.use_half}, conf={self.conf_thres}")
        self.get_logger().info(f"Subscribe: {self.image_topic}, {self.camera_info_topic}")
        self.get_logger().info(f"Publish vis: {self.vis_topic}")

        # ===== ROS I/O =====
        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_frame_id = "camera"
        self.frame_count = 0

        self.sub_img = self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.sub_info = self.create_subscription(CameraInfo, self.camera_info_topic, self.on_caminfo, 10)

        self.pub_vis = self.create_publisher(Image, self.vis_topic, 10)
        self.pub_point = self.create_publisher(PointStamped, self.point_topic, 50)

        self.timer = self.create_timer(0.06, self.on_timer)  # ~33Hz

    def on_caminfo(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.have_info = True

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
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
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.point.x = float(X)
        msg.point.y = float(Y)
        msg.point.z = float(Z)
        # 这里 PointStamped 没地方写 name；如你需要 name，后续可改成 Marker 或自定义 msg
        self.pub_point.publish(msg)

    def on_timer(self):
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

                # 画框
                cv2.rectangle(disp, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

                # 如果该类别有真实尺寸，就算 XYZ
                if name in self.size_map:
                    W, H = self.size_map[name]
                    X, Y, Z, w_px, h_px, Zw, Zh = compute_xyz_from_bbox(
                        x1, y1, x2, y2,
                        self.fx, self.fy, self.cx, self.cy,
                        W, H,
                        mode=self.mode
                    )
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
        try:
            out_msg = self.bridge.cv2_to_imgmsg(disp, encoding="bgr8")
            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.header.frame_id = self.latest_frame_id
            self.pub_vis.publish(out_msg)
        except Exception as e:
            self.get_logger().warn(f"publish vis failed: {e}")


def main():
    rclpy.init()
    node = YoloeAutoPoseVisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

