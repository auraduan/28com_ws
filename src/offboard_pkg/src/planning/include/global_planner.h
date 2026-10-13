#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <Eigen/Dense>
#include <deque>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mutex>
#include <offboard_pkg/Message.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

namespace Global_Planning {
// 工具函数声明
geometry_msgs::Quaternion
RotateQuaternionByYaw(const geometry_msgs::Quaternion &originalQuaternion,
                      double yaw);
double QuaternionToYaw(const geometry_msgs::Quaternion &originalQuaternion);
double LimitYaw(double yaw);
double LimitValue(double value, double max);

struct Waypoint {
  Eigen::Vector2d pos;
  int type; // 0: Normal, 1: Internal Calculation
};

class Global_Planner {
public:
  Global_Planner() = default;
  ~Global_Planner() = default;

  void init(ros::NodeHandle &nh);

private:
  // 回调函数
  void local_pos_cb(const geometry_msgs::PoseStampedConstPtr &msg);
  void body_vel_cb(const geometry_msgs::TwistStampedConstPtr &msg);
  void mavros_state_cb(const mavros_msgs::StateConstPtr &msg);

  void control_cb(const ros::TimerEvent &e);

  // 控制状态机处理
  void handle_idle_state();
  void handle_waiting_state();
  void handle_rotating_state();
  void handle_moving_state();
  void handle_arrived_state();

  // 工具函数
  void load_goals_from_xml(std::string file_path);
  bool get_next_target(Eigen::Vector2d &target);
  Eigen::Vector2d calculate_internal_target(const Eigen::Vector2d &hint);
  void pop_current_target();
  bool peek_next_normal_waypoint(Eigen::Vector2d &next_target, int &next_type);
  void pub_message(ros::Publisher &pub, uint8_t msg_type,
                   std::string source_node, std::string msg_content);

  // 参数
  double ctrl_interval;
  double yaw_p, yaw_i, yaw_max;
  double thrust_set_min;
  double precision, yaw_precision;
  double arrival_delay;
  double lookahead_angle;
  double alignment_check_interval;
  double camera_offset_x, camera_offset_y;
  double feedback_check_interval_min, feedback_check_interval_max;
  double alignment_yaw_threshold;
  double extra_offset_factor_left, extra_offset_factor_right;
  double lookahead_distance;
  double yaw_overshoot_left, yaw_overshoot_right;
  double yaw_overshoot_threshold;

  // 状态变量
  enum State {
    STATE_IDLE,
    STATE_WAITING,
    STATE_ROTATING,
    STATE_MOVING,
    STATE_ARRIVED
  } current_state;

  bool init_orientation = false;
  bool car_ready = false;
  ros::Time last_arrival_time;
  ros::Time last_control_time;
  double sum_yaw_err = 0.0;
  ros::Time last_alignment_check_time;
  double initial_move_yaw = 0.0;
  bool lookahead_active = false; // Type 4 前瞻转向模式是否激活
  bool is_first_rotation = true; // 首次旋转标志，用于 overshoot 只在首次生效
  double overshoot_sign = 1.0;   // overshoot 方向 (+1=左, -1=右)，首帧锁定
  bool overshoot_locked = false; // overshoot 方向是否已锁定

  // 当前位置信息
  geometry_msgs::PoseStamped m_pose;
  geometry_msgs::TwistStamped m_twist;
  Eigen::Vector2d m_2dpos;
  double m_yaw = 0.0;
  double m_speed = 0.0;

  // 目标队列
  std::deque<Waypoint> pos_queue;
  Eigen::Vector2d current_target;
  int current_wp_type = 0;
  double target_yaw_rad = 0.0;

  // MAVROS状态
  mavros_msgs::State current_state_mavros;

  // 控制消息
  mavros_msgs::AttitudeTarget attitude_target_msg;

  // ROS相关
  ros::Subscriber local_pos_sub, body_vel_sub, mavros_state_sub, det_sub;

  // 视觉检测相关
  void det_cb(const geometry_msgs::PointStampedConstPtr &msg);
  Eigen::Vector3d current_det_body;
  bool has_det_target = false;
  double det_rec_yaw = 0.0;
  Eigen::Vector2d det_rec_pos;

  ros::Publisher attitude_target_pub, message_pub, current_target_pub;
  ros::ServiceClient set_mode_client, arming_client;
  ros::Timer control_timer;

  // 线程安全
  std::mutex mutex_;

  // 节点名称
  const std::string NODE_NAME = "Global_Planner";
};
} // namespace Global_Planning

#endif