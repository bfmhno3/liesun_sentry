/*
 * @file wheel_odometry.cpp
 * @brief 实现轮式里程计
 */

/*
 * MIT License
 *
 * Copyright (c) 2025 BFmHNO3
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "wheel_odometry/wheel_odometry.hpp"

#include <cstdint>
#include <memory>
#include <chrono>
#include <cmath>

#include "message_filters/subscriber.h"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/detail/joint_state__struct.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.h"

using liesun::sentry::lidar_mapping::WheelOdometry;
using namespace std::chrono_literals;

/**
 * @brief 构造函数
 * 
 * @param node_name: WheelOdometry 节点在 ROS2 系统中的名称
 * @param wheel_radius: 底盘轮子半径
 * @param chasis_radius: 底盘半径
 */
WheelOdometry::WheelOdometry(std::string node_name, const double wheel_radius, const double chasis_radius)
    : Node(node_name)
    , wheel_radius_(wheel_radius)
    , chasis_radius_(chasis_radius)
    , if_use_imu_(false)
    , if_publish_tf_(false)
    , imu_yaw_angle_(0.0f) {
        RCLCPP_INFO(this->get_logger(), "Init odometry");

        init_parameters();
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)); // 只保留最新 10 条消息
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        
        if (if_use_imu_) {
            uint8_t queue_size = 10;
            joint_state_imu_sync_ = std::make_shared<SynchronizerJointStateImu>(queue_size);
            msg_ftr_joint_state_sub_ =
                std::make_shared<message_filters::Subscriber<sensor_msgs::msg::JointState>>(this, "joint_states");
            msg_ftr_imu_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, "imu");

            joint_state_imu_sync_->connectInput(*msg_ftr_joint_state_sub_, *msg_ftr_imu_sub_);
            joint_state_imu_sync_->setInterMessageLowerBound(0, rclcpp::Duration(75ms));
            joint_state_imu_sync_->setInterMessageLowerBound(1, rclcpp::Duration(15ms));

            joint_state_imu_sync_->registerCallback(std::bind(
                &WheelOdometry::joint_state_and_imu_callback, this, std::placeholders::_1, std::placeholders::_2
            ));
        } else {
            joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "joint_state", qos,
                std::bind(&WheelOdometry::joint_state_callback, this, std::placeholders::_1)
            );
        }
    }

/**
 * @brief 析构函数
 * 
 */
WheelOdometry::~WheelOdometry() {
    RCLCPP_INFO(this->get_logger(), "Destroying WheelOdometry node.");
}

/**
 * @brief 从 ROS2 参数系统获取配置信息，自动配置 WheelOdometry 的行为
 * 
 */
void WheelOdometry::init_parameters() {
    this->declare_parameter<std::string>("odometry.frame_id");
    this->declare_parameter<std::string>("odometry.child_frame_id");
    this->declare_parameter<bool>("odometry.if_use_imu");
    this->declare_parameter<bool>("odometry.if_publish_tf");

    this->get_parameter_or<std::string>("odometry.frame_id", frame_id_of_odometry_, std::string("odom"));
    this->get_parameter_or<std::string>(
        "odometry.child_frame_id", child_frame_id_of_odometry_, std::string("base_footprint"));
    this->get_parameter_or<bool>("odometry.if_use_imu", if_use_imu_, false);
    this->get_parameter_or<bool>("odometry.if_publish_tf", if_publish_tf_, false);
}

/**
 * @brief 回调函数，接收到 sensor_msgs::msg::JointState 消息后，ROS2 系统自动调用
 * 
 * @param[in] joint_state_msg
 */
void WheelOdometry::joint_state_callback(sensor_msgs::msg::JointState::ConstSharedPtr joint_state_msg) {
    const rclcpp::Time current_time = joint_state_msg->header.stamp;
    static rclcpp::Time last_time = current_time;
    const rclcpp::Duration duration = current_time - last_time;

    update_joint_state(joint_state_msg);
    calculate_odometry(duration);
    publish(current_time);

    last_time = current_time;
}

/**
 * @brief 回调函数，ROS2 系统检测到同步的消息自动调用
 * 
 * @param[in] joint_state_msg 
 * @param[in] imu_msg 
 */
void WheelOdometry::joint_state_and_imu_callback(const sensor_msgs::msg::JointState::ConstSharedPtr joint_state_msg,
                                                 const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg) {
    RCLCPP_DEBUG(
        this->get_logger(), 
        "[joint_state_msg] nanosec: %d [imu_msg] nanosec: %d",
        joint_state_msg->header.stamp.nanosec,
        imu_msg->header.stamp.nanosec
    );

    const rclcpp::Time current_time = joint_state_msg->header.stamp;
    static rclcpp::Time last_time = current_time;
    const rclcpp::Duration duration = current_time - last_time;

    update_joint_state(joint_state_msg);
    update_imu(imu_msg);
    calculate_odometry(duration);
    publish(current_time);

    last_time = current_time;
}

/**
 * @brief 更新偏航角 yaw
 * 
 * @param[in] imu_msg: IMU 消息
 */
void WheelOdometry::update_imu(sensor_msgs::msg::Imu::ConstSharedPtr imu_msg) {
    double qx = imu_msg->orientation.x;
    double qy = imu_msg->orientation.y;
    double qz = imu_msg->orientation.z;
    double qw = imu_msg->orientation.w;
    imu_yaw_angle_ = ::atan2f(qx * qy + qw * qz, 0.5f - qy * qy - qz * qz);
}

/**
 * @brief 更新底盘轮子状态信息，计算轮子在时间间隔中旋转的角度
 * 
 * @param[in] joint_state_msg: 底盘轮子消息
 */
void WheelOdometry::update_joint_state(sensor_msgs::msg::JointState::ConstSharedPtr joint_state_msg) {
    static std::array<double, 4> last_wheel_joint_positions = {0.0f, 0.0f, 0.0f, 0.0f};

    diff_wheel_joint_positions_[0] = joint_state_msg->position[0] - last_wheel_joint_positions[0];
    diff_wheel_joint_positions_[1] = joint_state_msg->position[1] - last_wheel_joint_positions[1];
    diff_wheel_joint_positions_[2] = joint_state_msg->position[2] - last_wheel_joint_positions[2];
    diff_wheel_joint_positions_[3] = joint_state_msg->position[3] - last_wheel_joint_positions[3];

    last_wheel_joint_positions[0] = joint_state_msg->position[0];
    last_wheel_joint_positions[1] = joint_state_msg->position[1];
    last_wheel_joint_positions[2] = joint_state_msg->position[2];
    last_wheel_joint_positions[3] = joint_state_msg->position[3];
}

/**
 * @brief 根据车轮关节角度及 IMU数据（可选）计算轮式里程计
 *
 * 此函数基于车轮角度差及可选的 IMU 数据计算机器人位姿和速度的变化。
 * 支持两种计算方式：
 *   1.使用IMU数据更新偏航角（当if_use_imu_为true时）；
 *   2.直接根据车轮转动计算偏航角
 *
 * @param[in] duration : 表示计算里程计的时间间隔，用于速度计算。
 * @return 若时间间隔为零（delta_t == 0）则返回false，否则返回true，表示成功计算出里程计信息。
 *
 * @note 当 if_use_imu_ 为 false 时，只使用底盘数据计算里程计数据。
 */
bool WheelOdometry::calculate_odometry(const rclcpp::Duration& duration) {

    /*
     * 轮子的定义：wheel_<象限>
     * 哨兵底盘坐标系定义：使用 ROS2 规定的右手系：x 轴为前进的方向，y 轴为与 x 轴垂直的左方，z 轴垂直于 xOy 平面向上。
     * 象限的定义：xy 表示由 x 和 y 轴的正半轴所夹象限（第一象限），nxy 表示由 x 轴负半轴和 y 轴正半轴所夹象限（第二象限），以此类推。
     * 正常来说，第一象限的轮子的
     */
    double wheel_xy = diff_wheel_joint_positions_[0];
    double wheel_nxy = diff_wheel_joint_positions_[1];
    double wheel_nxny = diff_wheel_joint_positions_[2];
    double wheel_xny = diff_wheel_joint_positions_[3];

    // delta_x 是物理意义上的位移变化量，其后缀表示在坐标轴上的投影
    double delta_x_x = 0.0f;
    double delta_x_y = 0.0f;
    double yaw = 0.0;
    double delta_yaw = 0.0f;
    static double last_yaw = 0.0;

    double vx = 0.0f;
    double vy = 0.0f;
    double omega = 0.0f; // 角速度

    double delta_t = duration.seconds(); // 时间间隔

    double cos_yaw, sin_yaw, cos_positive, cos_negative, sin_positive, sin_negative;

    // 确保用于计算的值都是有效的
    if (0.0f == delta_t) {
        return false;
    }
   
    set_nan_to_zero(wheel_xy);
    set_nan_to_zero(wheel_nxy);
    set_nan_to_zero(wheel_nxny);
    set_nan_to_zero(wheel_xny);

    if (if_use_imu_) {
        yaw = imu_yaw_angle_;
        delta_yaw = yaw - last_yaw;
        last_yaw = yaw;
    } else {
        delta_yaw = (wheel_xy + wheel_nxy + wheel_nxny + wheel_xny) * wheel_radius_ / chasis_radius_ / 4;
        yaw = robot_pose_[2] + delta_yaw;
    }

    cos_yaw = std::cos(robot_pose_[2] + delta_yaw / 2);
    sin_yaw = std::sin(robot_pose_[2] + delta_yaw / 2);
    cos_positive = 0.707 * (cos_yaw - sin_yaw);
    cos_negative = 0.707 * (cos_yaw + sin_yaw);
    sin_positive = cos_negative;
    sin_negative = cos_positive;

    // 计算公式基于底盘运动学模型，查看文档
    delta_x_x = wheel_xy * cos_positive - wheel_nxy * cos_negative - wheel_nxny * cos_positive + wheel_xny * cos_negative;
    delta_x_y = wheel_xy * sin_positive + wheel_nxy * sin_negative - wheel_nxny * sin_positive - wheel_xny * sin_negative;
    robot_pose_[0] += delta_x_x;
    robot_pose_[1] += delta_x_y;
    robot_pose_[2] = yaw;

    RCLCPP_DEBUG(this->get_logger(), "x: %f, y: %f, yaw: %f", robot_pose_[0], robot_pose_[1], robot_pose_[2]);

    vx = delta_x_x / delta_t;
    vy = delta_x_y / delta_t;
    omega = delta_yaw / delta_t;

    robot_vel_[0] = vx;
    robot_vel_[1] = vy;
    robot_vel_[2] = omega;

    return true;
}

/**
 * @brief 发布里程计消息和 tf2 转换
 * 
 * @param[in] now: 当前时间戳， 用于设置消息的时间戳
 */
void WheelOdometry::publish(const rclcpp::Time& now) const {
    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

    odom_msg->header.frame_id = frame_id_of_odometry_; // "odom"
    odom_msg->child_frame_id = child_frame_id_of_odometry_; // "base_footprint"
    odom_msg->header.stamp = now;

    odom_msg->pose.pose.position.x = robot_pose_[0];
    odom_msg->pose.pose.position.y = robot_pose_[1];
    odom_msg->pose.pose.position.z = robot_pose_[2];

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, robot_pose_[2]); // 机器人受到地面约束，只有偏航角 yaw

    odom_msg->pose.pose.orientation.x = q.x();
    odom_msg->pose.pose.orientation.y = q.y();
    odom_msg->pose.pose.orientation.z = q.z();
    odom_msg->pose.pose.orientation.w = q.w();

    odom_msg->twist.twist.linear.x = robot_vel_[0];
    odom_msg->twist.twist.linear.y = robot_vel_[1];
    odom_msg->twist.twist.angular.z = robot_vel_[2];

    // 发布 tf2 变换
    geometry_msgs::msg::TransformStamped odom_tf;

    odom_tf.transform.translation.x = odom_msg->pose.pose.position.x;
    odom_tf.transform.translation.y = odom_msg->pose.pose.position.y;
    odom_tf.transform.translation.z = odom_msg->pose.pose.position.z;
    
    odom_tf.transform.rotation = odom_msg->pose.pose.orientation;

    odom_tf.header.stamp = now;
    odom_tf.header.frame_id = frame_id_of_odometry_; // "odom"
    odom_tf.child_frame_id = child_frame_id_of_odometry_; // "base_footprint"

    odom_pub_->publish(std::move(odom_msg));

    if (if_publish_tf_) {
        tf_broadcaster_->sendTransform(odom_tf);
    }
}

/**
 * @brief 将 NaN 修改为 0.0
 * 
 * @param[out] value: 需要设置的值
 */
void WheelOdometry::set_nan_to_zero(double& value) {
    if (std::isnan(value)) {
        value = 0.0f;
        RCLCPP_WARN(this->get_logger(), "value is NaN, set it to zero.");
    }
}
