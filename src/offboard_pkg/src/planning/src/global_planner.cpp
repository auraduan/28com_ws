#include "global_planner.h"
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <cmath>
#include <deque>
#include <mutex>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static int debug_count = 0;
static int idle_count = 0;
static int first_point = true;

namespace Global_Planning {
// 四元数绕Z轴旋转yaw角度
geometry_msgs::Quaternion
RotateQuaternionByYaw(const geometry_msgs::Quaternion &originalQuaternion,
                      double yaw) {
  tf2::Quaternion original;
  tf2::fromMsg(originalQuaternion, original);

  tf2::Quaternion rotation;
  rotation.setRPY(0, 0, yaw);

  tf2::Quaternion rotated = original * rotation;
  rotated.normalize();

  return tf2::toMsg(rotated);
}

// 四元数转偏航角
double QuaternionToYaw(const geometry_msgs::Quaternion &originalQuaternion) {
  tf2::Quaternion quat;
  tf2::fromMsg(originalQuaternion, quat);

  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return yaw;
}

// 限制偏航角到[-π, π]
double LimitYaw(double yaw) {
  while (yaw > M_PI)
    yaw -= 2 * M_PI;
  while (yaw < -M_PI)
    yaw += 2 * M_PI;
  return yaw;
}

// 限制值到[-max, max]
double LimitValue(double value, double max) {
  if (value > max)
    return max;
  if (value < -max)
    return -max;
  return value;
}

// 初始化函数
void Global_Planner::init(ros::NodeHandle &nh) {
  // 参数初始化
  nh.param("ctrl_interval", ctrl_interval, 0.05);
  nh.param("yaw_p", yaw_p, 15.0);
  nh.param("yaw_i", yaw_i, 0.0);
  nh.param("yaw_max", yaw_max, 1.0);
  nh.param("thrust_set_min", thrust_set_min, 0.4);
  nh.param("precision", precision, 0.1);
  nh.param("yaw_precision", yaw_precision, 0.05);
  nh.param("arrival_delay", arrival_delay, 1.0);
  nh.param("lookahead_angle", lookahead_angle, 10.0);
  nh.param("alignment_check_interval", alignment_check_interval, 1.0);
  nh.param("camera_offset_x", camera_offset_x, 0.05);
  nh.param("camera_offset_y", camera_offset_y, 0.05);

  nh.param("feedback_check_interval_min", feedback_check_interval_min, 0.25);
  nh.param("feedback_check_interval_max", feedback_check_interval_max, 1.0);
  nh.param("alignment_yaw_threshold", alignment_yaw_threshold, 0.087);
  nh.param("extra_offset_factor_left", extra_offset_factor_left, 0.3);
  nh.param("extra_offset_factor_right", extra_offset_factor_right, 0.3);
  nh.param("lookahead_distance", lookahead_distance, 0.5);
  nh.param("yaw_overshoot_left", yaw_overshoot_left, 0.03);
  nh.param("yaw_overshoot_right", yaw_overshoot_right, 0.0);
  nh.param("yaw_overshoot_threshold", yaw_overshoot_threshold, 0.2);

  // 控制状态初始化
  current_state = STATE_IDLE;
  last_arrival_time = ros::Time::now();
  last_control_time = ros::Time::now();
  last_control_time = ros::Time::now();
  last_alignment_check_time = ros::Time::now();
  sum_yaw_err = 0.0;

  // 姿态目标消息初始化
  attitude_target_msg.header.stamp = ros::Time::now();
  attitude_target_msg.type_mask =
      mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
      mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
      mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
  attitude_target_msg.orientation.w = 1.0;
  attitude_target_msg.orientation.x = 0.0;
  attitude_target_msg.orientation.y = 0.0;
  attitude_target_msg.orientation.z = 0.0;
  attitude_target_msg.thrust = 0.0;

  // 订阅当前位置和速度
  local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "/mavros/local_position/pose", 20, &Global_Planner::local_pos_cb, this);
  body_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(
      "/mavros/local_position/velocity_body", 20, &Global_Planner::body_vel_cb,
      this);
  mavros_state_sub = nh.subscribe<mavros_msgs::State>(
      "/mavros/state", 10, &Global_Planner::mavros_state_cb, this);

  // 订阅位置命令队列
  det_sub = nh.subscribe("/det/xyz", 10, &Global_Planner::det_cb, this);

  // 加载目标点
  std::string goal_file_path;
  nh.param<std::string>("goal_file_path", goal_file_path, "");
  ROS_INFO("goal_file_path param is: '%s'", goal_file_path.c_str());
  if (!goal_file_path.empty()) {
    load_goals_from_xml(goal_file_path);
  } else {
    // ROS_INFO("goal_file_path param is: '%s'", goal_file_path.c_str());
    ROS_WARN("No goal file path provided!");
  }

  // 服务客户端
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  arming_client =
      nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

  // 发布器
  attitude_target_pub = nh.advertise<mavros_msgs::AttitudeTarget>(
      "/mavros/setpoint_raw/attitude", 20);
  message_pub =
      nh.advertise<offboard_pkg::Message>("/Car/message/global_planner", 10);
  current_target_pub =
      nh.advertise<geometry_msgs::PoseStamped>("/current_target", 10);

  // 控制定时器
  control_timer = nh.createTimer(ros::Duration(ctrl_interval),
                                 &Global_Planner::control_cb, this);

  // 发布初始消息
  pub_message(message_pub, offboard_pkg::Message::NORMAL, NODE_NAME,
              "Global Planner initialized.");

  ROS_INFO("Global Planner initialized. Waiting for activation...");
}

// 当前位置回调
void Global_Planner::local_pos_cb(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  m_pose = *msg;
  m_2dpos[0] = m_pose.pose.position.x;
  m_2dpos[1] = m_pose.pose.position.y;
  m_yaw = QuaternionToYaw(m_pose.pose.orientation);

  if (!init_orientation) {
    init_orientation = true;
    attitude_target_msg.orientation = m_pose.pose.orientation;
    ROS_INFO("Initial orientation set.");
  }
}

// 速度回调
void Global_Planner::body_vel_cb(
    const geometry_msgs::TwistStampedConstPtr &msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  m_twist = *msg;
  m_speed = m_twist.twist.linear.x;
}

// 状态回调
void Global_Planner::mavros_state_cb(const mavros_msgs::StateConstPtr &msg) {
  current_state_mavros = *msg;

  if (!car_ready && current_state_mavros.armed &&
      current_state_mavros.mode == "OFFBOARD") {
    car_ready = true;
    ROS_INFO("Car is ready (OFFBOARD mode and armed)");
  }

  if (current_state_mavros.mode != "OFFBOARD") {
    car_ready = false;
  }
}

// 获取下一个目标点
bool Global_Planner::get_next_target(Eigen::Vector2d &target) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (pos_queue.empty()) {
    // ROS_WARN("[get_next_target] Queue is empty!");
    return false;
  }

  // 获取队列中的下一个点 (引用以修改队列内容)
  Waypoint &wp = pos_queue.front();

  // Store the type for state handling
  current_wp_type = wp.type;

  if (wp.type == 2) {
    // Left turn 90 deg
    target_yaw_rad = LimitYaw(m_yaw + M_PI / 2.0);
    ROS_INFO("Target: Turn Left 90. Target Yaw: %.3f", target_yaw_rad);
    // target position doesn't matter for pure rotation, but let's keep it safe
    target = m_2dpos;
  } else if (wp.type == 3) {
    // Right turn 90 deg
    target_yaw_rad = LimitYaw(m_yaw - M_PI / 2.0);
    ROS_INFO("Target: Turn Right 90. Target Yaw: %.3f", target_yaw_rad);
    target = m_2dpos;
  } else if (wp.type == 1) {
    if (!has_det_target) {
      ROS_WARN_THROTTLE(
          1.0, "Current waypoint is Type 1 (Visual), waiting for detection...");
      return false;
    }

    // 内部计算点
    ROS_INFO("Reached dynamic waypoint. Calculating target internally...");
    target = calculate_internal_target(wp.pos);

    // 更新队列：标记为普通点，固定坐标
    wp.pos = target;
    wp.type = 0;
    // Update current type as well effectively, though next loop will handle it
    // as 0 if we don't pop. Actually we don't pop here, we just modify the
    // front. The state machine will process this target.

    // 视觉检测标志
    has_det_target = false;

  } else {
    // 普通点
    target = wp.pos;
  }

  return true;
}

// 弹出当前目标点
void Global_Planner::pop_current_target() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!pos_queue.empty()) {
    // Waypoint completed = pos_queue.front(); // Unused variable
    pos_queue.pop_front();
  }
}

// 窥探下一个普通航点（不弹出队列）
// 用于前瞻转向逻辑，跳过纯旋转指令(type 2/3)
// 返回下一个航点的坐标和类型
bool Global_Planner::peek_next_normal_waypoint(Eigen::Vector2d &next_target,
                                               int &next_type) {
  std::lock_guard<std::mutex> lock(mutex_);

  // 至少需要队列中有两个点（当前点 + 下一个点）
  if (pos_queue.size() < 2) {
    return false;
  }

  // 从第二个点开始查找下一个普通航点
  for (size_t i = 1; i < pos_queue.size(); ++i) {
    const Waypoint &wp = pos_queue[i];
    // type 0: 普通点
    // type 1: 视觉点(坐标是hint，方向大致正确，可用于前瞻转向)
    // type 4: 前瞻转向点
    // type 2/3: 纯旋转，跳过
    if (wp.type == 0 || wp.type == 1 || wp.type == 4) {
      next_target = wp.pos;
      next_type = wp.type;
      return true;
    }
  }

  return false;
}

// 主控制回调
void Global_Planner::control_cb(const ros::TimerEvent &e) {
  static ros::Time last_request = ros::Time::now();

  // 等待初始化完成
  if (!init_orientation) {
    return;
  }

  // OFFBOARD模式和解锁
  if (!car_ready) {
    attitude_target_pub.publish(attitude_target_msg);

    if (ros::Time::now() - last_request > ros::Duration(2.0)) {
      ROS_INFO("Waiting for system to be ARMED and in OFFBOARD mode...");
      last_request = ros::Time::now();
    }
    return;
  }

  // 状态机处理
  switch (current_state) {
  case STATE_IDLE:
    handle_idle_state();
    break;
  case STATE_ROTATING:
    handle_rotating_state();
    break;

  case STATE_MOVING:
    handle_moving_state();
    break;

  case STATE_ARRIVED:
    handle_arrived_state();
    break;
  }

  // 发布当前目标点用于可视化
  if (current_state != STATE_IDLE) {
    geometry_msgs::PoseStamped target_msg;
    target_msg.header.stamp = ros::Time::now();
    target_msg.header.frame_id = "world";
    target_msg.pose.position.x = current_target[0];
    target_msg.pose.position.y = current_target[1];
    target_msg.pose.position.z = 0.0;
    current_target_pub.publish(target_msg);
  }

  // Debug Output
  static ros::Time last_debug_time = ros::Time::now();
  if ((ros::Time::now() - last_debug_time).toSec() > 0.25) {
    std::string state_s;
    switch (current_state) {
    case STATE_IDLE:
      state_s = "IDLE";
      break;
    case STATE_ROTATING:
      state_s = "ROTATING";
      break;
    case STATE_MOVING:
      state_s = "MOVING";
      break;
    case STATE_ARRIVED:
      state_s = "ARRIVED";
      break;
    default:
      state_s = "UNKNOWN";
      break;
    }
    ROS_INFO("State: %s | Target: (%.2f, %.2f) | CurPos: (%.2f, %.2f) | "
             "YawErr: %.3f",
             state_s.c_str(), current_target[0], current_target[1], m_2dpos[0],
             m_2dpos[1], (current_state == STATE_ROTATING) ? sum_yaw_err : 0.0);
    last_debug_time = ros::Time::now();
  }

  // 发布控制指令
  attitude_target_pub.publish(attitude_target_msg);
}

// 空闲状态处理
void Global_Planner::handle_idle_state() {
  Eigen::Vector2d target;
  if (get_next_target(target)) {
    current_target = target;
    is_first_rotation = true;
    overshoot_locked = false;
    current_state = STATE_ROTATING;
  } else {
    // 保持停止状态
    attitude_target_msg.thrust = 0.0;
    attitude_target_msg.orientation = m_pose.pose.orientation; // 保持当前朝向
  }
}

// 旋转状态处理
void Global_Planner::handle_rotating_state() {
  double yaw_err;

  if (current_wp_type == 2 || current_wp_type == 3) {
    // Pure Rotation Logic
    yaw_err = LimitYaw(target_yaw_rad - m_yaw);

    if (fabs(yaw_err) < yaw_precision) {
      current_state = STATE_ARRIVED;
      last_arrival_time = ros::Time::now();
      sum_yaw_err = 0.0;
      ROS_INFO("Pure rotation completed. Moving to ARRIVED.");
      return;
    }

  } else {
    // Standard Move-to-Target Logic
    Eigen::Vector2d err = current_target - m_2dpos;
    double dis = err.norm();
    if (dis < precision) {
      current_state = STATE_ARRIVED;
      last_arrival_time = ros::Time::now();
      sum_yaw_err = 0.0;
      return;
    }
    double yaw_target = atan2(err[1], err[0]);
    // Apply overshoot only on first rotation for a new waypoint.
    // Lock the direction on first frame to prevent sign-flipping oscillation.
    if (is_first_rotation) {
      if (!overshoot_locked) {
        double raw_yaw_err = LimitYaw(yaw_target - m_yaw);
        overshoot_sign = (raw_yaw_err >= 0) ? 1.0 : -1.0;
        overshoot_locked = true;
        // Only apply overshoot if initial yaw error exceeds threshold
        if (fabs(raw_yaw_err) < yaw_overshoot_threshold) {
          is_first_rotation = false; // Skip overshoot for small rotations
          ROS_INFO("Overshoot skipped: |raw_err|=%.3f < threshold=%.3f",
                   fabs(raw_yaw_err), yaw_overshoot_threshold);
        } else {
          ROS_INFO("Overshoot locked: sign=%.0f (raw_err=%.3f, threshold=%.3f)",
                   overshoot_sign, raw_yaw_err, yaw_overshoot_threshold);
        }
      }
      // Apply overshoot (only if is_first_rotation is still true)
      if (is_first_rotation) {
        double overshoot_val =
            (overshoot_sign > 0) ? yaw_overshoot_left : yaw_overshoot_right;
        yaw_target = LimitYaw(yaw_target + overshoot_sign * overshoot_val);
      }
    }
    yaw_err = LimitYaw(yaw_target - m_yaw);
  }

  // Common PID Control Logic

  // 计算控制间隔
  ros::Time now = ros::Time::now();
  double dt = (now - last_control_time).toSec();
  last_control_time = now;
  if (dt > 0.2)
    dt = 0.2;

  // 积分项
  sum_yaw_err += yaw_err * dt;

  // PID控制
  double yaw_rate = yaw_err * yaw_p + sum_yaw_err * yaw_i;
  yaw_rate = LimitValue(yaw_rate, yaw_max);

  // 设置姿态和油门
  attitude_target_msg.orientation =
      RotateQuaternionByYaw(m_pose.pose.orientation, yaw_rate);
  attitude_target_msg.thrust = 0; // 低速旋转

  // 如果是一般移动任务，检查是否对准进入MOVING
  if (current_wp_type != 2 && current_wp_type != 3) {
    if (fabs(yaw_err) < yaw_precision) {
      current_state = STATE_MOVING;
      sum_yaw_err = 0.0;
      is_first_rotation = false; // 后续修正旋转不再加 overshoot
      initial_move_yaw = m_yaw;  // Record initial yaw when starting to move
    }
  }
}

// 移动状态处理
void Global_Planner::handle_moving_state() {
  Eigen::Vector2d err = current_target - m_2dpos;
  double dis = err.norm();

  // 计算控制间隔
  ros::Time now = ros::Time::now();
  double dt = (now - last_control_time).toSec();
  last_control_time = now;
  if (dt > 0.2)
    dt = 0.2;

  // ============================================================
  // Type 4: 前瞻目标点 - 边走边转向下一个目标
  // ============================================================
  if (current_wp_type == 4) {
    Eigen::Vector2d next_wp_target;
    int next_wp_type = 0;
    bool has_next_wp = peek_next_normal_waypoint(next_wp_target, next_wp_type);

    // 检查是否应该激活/保持前瞻模式
    // 一旦 dis < lookahead_distance，激活前瞻模式
    // 激活后持续转向，直到对准下一个目标点
    if (!lookahead_active && dis < lookahead_distance && has_next_wp) {
      lookahead_active = true;
      sum_yaw_err = 0.0; // 重置积分项
      ROS_INFO("[Type4] Lookahead mode ACTIVATED! Distance=%.2f, NextType=%d",
               dis, next_wp_type);
    }

    ROS_INFO_THROTTLE(1.0,
                      "[Type4] Distance=%.2f, lookahead_dist=%.2f, active=%d, "
                      "has_next=%d, next_type=%d",
                      dis, lookahead_distance, lookahead_active, has_next_wp,
                      next_wp_type);

    if (lookahead_active && has_next_wp) {
      // ========================================
      // 前瞻转向模式已激活
      // ========================================

      // 【新功能】如果下一个点是视觉点(Type 1)，检测是否已有视觉目标
      // 一旦检测到，立即停止并跳过当前航点，进入视觉处理
      if (next_wp_type == 1 && has_det_target) {
        // 立即停止
        attitude_target_msg.thrust = 0.0;
        attitude_target_msg.orientation = m_pose.pose.orientation;

        // 跳过当前Type 4航点，进入下一个视觉航点
        ROS_INFO("[Type4] Visual target DETECTED during lookahead! Skipping to "
                 "Type 1...");

        // 弹出当前Type 4航点
        pop_current_target();

        // 重置状态
        lookahead_active = false;
        sum_yaw_err = 0.0;

        // 切换到IDLE，让下一个控制周期处理Type 1视觉航点
        current_state = STATE_IDLE;
        return;
      }

      // 正常前瞻转向逻辑
      Eigen::Vector2d next_err = next_wp_target - m_2dpos;
      double steering_yaw_target = atan2(next_err[1], next_err[0]);
      double yaw_err = LimitYaw(steering_yaw_target - m_yaw);

      ROS_INFO_THROTTLE(
          0.5,
          "[Type4] STEERING! NextWP=(%.2f,%.2f), YawErr=%.3f rad (%.1f deg)",
          next_wp_target[0], next_wp_target[1], yaw_err,
          yaw_err * 180.0 / M_PI);

      // 检查是否已对准下一个目标点 -> 完成条件
      if (fabs(yaw_err) < yaw_precision) {
        current_state = STATE_ARRIVED;
        last_arrival_time = now;
        sum_yaw_err = 0.0;
        lookahead_active = false; // 重置标志
        ROS_INFO("[Type4] ALIGNED! YawErr=%.3f < precision=%.3f. -> ARRIVED.",
                 fabs(yaw_err), yaw_precision);
        return;
      }

      // 边走边转向 - 使用全强度P增益，快速转向
      sum_yaw_err += yaw_err * dt;
      double yaw_rate = yaw_err * yaw_p + sum_yaw_err * yaw_i;
      yaw_rate = LimitValue(yaw_rate, yaw_max);

      attitude_target_msg.orientation =
          RotateQuaternionByYaw(m_pose.pose.orientation, yaw_rate);
      // 减速到0.2油门，边走边转
      attitude_target_msg.thrust = 0.2;

    } else {
      // === 未进入前瞻范围，普通直行 ===
      attitude_target_msg.orientation = m_pose.pose.orientation;
      attitude_target_msg.thrust = thrust_set_min;

      // 普通目标点到达检查（备用，防止没有下一个点的情况）
      if (dis < precision) {
        current_state = STATE_ARRIVED;
        last_arrival_time = now;
        lookahead_active = false; // 重置标志
        ROS_INFO("[Type4] Reached target (no next WP). ARRIVED.");
        return;
      }
    }
    return;
  }

  // ============================================================
  // Type 0 (默认): 普通目标点 - 直线走过去
  // ============================================================

  // 检查是否到达
  if (dis < precision) {
    current_state = STATE_ARRIVED;
    last_arrival_time = now;
    return;
  }

  // 保持当前朝向直行
  attitude_target_msg.orientation = m_pose.pose.orientation;
  attitude_target_msg.thrust = thrust_set_min;

  // === 周期性姿态对齐检查 ===
  double k =
      (feedback_check_interval_min - feedback_check_interval_max) / (1.0 - 0.2);
  double b = feedback_check_interval_max - k * 0.2;
  double dynamic_interval = k * thrust_set_min + b;
  if (dynamic_interval < feedback_check_interval_min)
    dynamic_interval = feedback_check_interval_min;
  if (dynamic_interval > feedback_check_interval_max)
    dynamic_interval = feedback_check_interval_max;

  if ((now - last_alignment_check_time).toSec() > dynamic_interval) {
    last_alignment_check_time = now;
    double yaw_target = atan2(err[1], err[0]);
    double current_yaw_err = LimitYaw(yaw_target - m_yaw);

    if (fabs(current_yaw_err) > alignment_yaw_threshold) {
      current_state = STATE_ROTATING;
      sum_yaw_err = 0.0;
      ROS_INFO(
          "Periodic alignment check: Yaw error %.3f > threshold, correcting...",
          current_yaw_err);
      return;
    }
  }
}

// 到达状态处理
void Global_Planner::handle_arrived_state() {
  // 停止所有运动
  attitude_target_msg.thrust = 0.0;
  attitude_target_msg.orientation = m_pose.pose.orientation;

  // 检查停留时间
  if ((ros::Time::now() - last_arrival_time).toSec() > arrival_delay) {
    // 弹出当前目标
    pop_current_target();

    // 检查是否有下一个目标
    Eigen::Vector2d next_target;
    if (get_next_target(next_target)) {
      current_target = next_target;
      // current_state = STATE_WAITING;

      // 重置所有控制积分项和状态标志
      sum_yaw_err = 0.0;
      lookahead_active = false;

      if (current_wp_type == 2 || current_wp_type == 3) {
        // Pure rotation task, always go to ROTATING
        current_state = STATE_ROTATING;
        ROS_INFO(
            "Next target is pure rotation (Type %d), switching to ROTATING",
            current_wp_type);
      } else {
        // Normal move task, check alignment
        Eigen::Vector2d err = current_target - m_2dpos;
        double yaw_target = atan2(err[1], err[0]);
        double yaw_err = LimitYaw(yaw_target - m_yaw);

        if (fabs(yaw_err) > yaw_precision) {
          current_state = STATE_ROTATING;
        } else {
          // Already aligned, straight to moving
          current_state = STATE_MOVING;
        }
      }
    } else {
      current_state = STATE_IDLE;
      // ROS_INFO("All targets completed, returning to idle state");
    }
  }
}

// 发布消息
void Global_Planner::pub_message(ros::Publisher &pub, uint8_t msg_type,
                                 std::string source_node,
                                 std::string msg_content) {
  offboard_pkg::Message message;
  message.header.stamp = ros::Time::now();
  message.message_type = msg_type; // 注意：可能是大写Type而不是小写type
  message.source_node = source_node;
  message.content = msg_content;
  pub.publish(message);
}

void Global_Planner::load_goals_from_xml(std::string file_path) {
  std::lock_guard<std::mutex> lock(mutex_);
  ROS_INFO("Loading goals from XML: %s", file_path.c_str());

  try {
    boost::property_tree::ptree pt;
    boost::property_tree::read_xml(file_path, pt);

    BOOST_FOREACH (boost::property_tree::ptree::value_type &v,
                   pt.get_child("root")) {
      if (v.first == "waypoint") {
        double x = v.second.get<double>("<xmlattr>.x");
        double y = v.second.get<double>("<xmlattr>.y");
        int type = v.second.get<int>("<xmlattr>.type",
                                     0); // Default to 0 if not specified

        Waypoint wp;
        wp.pos = Eigen::Vector2d(x, y);
        wp.type = type;

        pos_queue.push_back(wp);
        ROS_INFO("Loaded waypoint: (%.3f, %.3f) Type: %d", x, y, type);
      }
    }
    ROS_INFO("Total waypoints loaded: %zu", pos_queue.size());
  } catch (std::exception const &e) {
    ROS_ERROR("Error loading XML: %s", e.what());
  }
}

Eigen::Vector2d
Global_Planner::calculate_internal_target(const Eigen::Vector2d &hint) {
  // If we have a detection target, use it
  if (has_det_target) {
    // current_det_body is [x_body, y_body, z_body]
    // We need to transform this vector from Body Frame to Local (World) Frame
    // (ENU) Position_Global = R_body_to_global * Position_Body +
    // Vehicle_Position_Global

    // 2D Rotation (Yaw only)
    // x_g = x_b * cos(yaw) - y_b * sin(yaw) + pos_x
    // y_g = x_b * sin(yaw) + y_b * cos(yaw) + pos_y

    double xb = current_det_body[0];
    double yb = current_det_body[1];

    // Offset logic: asymmetric left/right compensation
    // Car tends to drift right, so right factor can be set larger to
    // compensate.
    // 1. Reference is Yaw direction (Body X axis).
    // 2. Offsets applied laterally (Body Y axis).
    // 3. Magnitude depends on |yb|, scaled by direction-specific factor, max
    // 0.2.
    if (yb > 0) {
      // Target is LEFT of body center (yb > 0)
      double extra_offset = fabs(yb) * extra_offset_factor_left;
      if (extra_offset > 0.2)
        extra_offset = 0.2;
      yb += extra_offset;
      ROS_INFO("Target LEFT. factor=%.2f, offset=%.3f, new yb=%.3f",
               extra_offset_factor_left, extra_offset, yb);
    } else {
      // Target is RIGHT of body center (yb <= 0)
      double extra_offset = fabs(yb) * extra_offset_factor_right;
      if (extra_offset > 0.2)
        extra_offset = 0.2;
      yb -= extra_offset;
      ROS_INFO("Target RIGHT. factor=%.2f, offset=%.3f, new yb=%.3f",
               extra_offset_factor_right, extra_offset, yb);
    }

    // Use recorded yaw and pos to sync with detection time
    double xg = xb * cos(det_rec_yaw) - yb * sin(det_rec_yaw) + det_rec_pos[0];
    double yg = xb * sin(det_rec_yaw) + yb * cos(det_rec_yaw) + det_rec_pos[1];

    ROS_INFO("Calculated target from det (Sync): Body(%.3f, %.3f) -> "
             "Global(%.3f, %.3f) using Yaw %.3f",
             xb, yb, xg, yg, det_rec_yaw);
    return Eigen::Vector2d(xg, yg);
  }

  ROS_WARN("No det target received, using hint");
  return hint;
}

// 视觉检测回调
void Global_Planner::det_cb(const geometry_msgs::PointStampedConstPtr &msg) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (pos_queue.empty()) {
    return;
  }

  int front_type = pos_queue.front().type;

  // 允许接收视觉检测数据的条件：
  // 1. 当前航点是Type 1 (视觉点)
  // 2. 当前航点是Type 4 (前瞻点) 且 lookahead_active 已激活
  //    (此时需要提前检测下一个视觉航点)
  bool should_process =
      (front_type == 1) || (front_type == 4 && lookahead_active);

  if (!should_process) {
    return;
  }

  // msg->point is in Camera Frame (Optical): X Right, Y Down, Z Forward
  // Body Frame: X Forward, Y Left, Z Up
  // User spec: Camera is 5cm Forward (+X body), 5cm Left (+Y body)

  // Transform Camera -> Body
  // Body X = Cam Z + Offset X
  // Body Y = -Cam X + Offset Y
  // Body Z = -Cam Y

  current_det_body[0] = msg->point.z + camera_offset_x;
  current_det_body[1] = -msg->point.x + camera_offset_y;
  current_det_body[2] = -msg->point.y;

  has_det_target = true;
  det_rec_yaw = m_yaw;
  det_rec_pos = m_2dpos;

  ROS_INFO_THROTTLE(0.5,
                    "[det_cb] Detection received! Body(%.3f, %.3f), "
                    "front_type=%d, lookahead=%d",
                    current_det_body[0], current_det_body[1], front_type,
                    lookahead_active);
}
} // namespace Global_Planning