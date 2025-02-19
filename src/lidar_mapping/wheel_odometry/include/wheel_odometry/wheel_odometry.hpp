/*
 * @file wheel_odometry.hpp
 * @brief 轮式里程计类的声明
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

#ifndef WHEEL_ODOMETRY_HPP_
#define WHEEL_ODOMETRY_HPP_

#include <string>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace liesun {
namespace sentry {
namespace lidar_mapping {

/**
 * @brief 车轮里程计节点。
 *
 * 该类用于融合轮子状态信息和 IMU 数据，通过 ROS2 节点订阅、数据同步以及计算流程，
 * 实现机器人的位置、姿态以及速度的估计。其主要功能包括：
 * - 初始化相关参数配置（轮子半径、底盘半径、tf2 坐标系标识等）。
 * - 单独或同步接收轮子编码器数据（JointState）与 IMU 数据，
 *   并通过回调函数分别更新数据状态。
 * - 基于传感器数据计算机器人的里程计（Odometry），获取位置信息 (x, y, 偏航角)
 *   与速度信息（线速度与角速度）。
 * - 发布 nav_msgs::msg::Odometry 消息到指定话题。
 * - 根据需要广播 tf2 坐标变换。
 *
 * 同步策略：
 * 采用 message_filters::sync_policies::ApproximateTime 策略对 JointState 与 IMU 数据进行同步，
 * 确保数据在不同传感器采样频率情况下的合理匹配。
 *
 * @note 该节点是针对轮式机器人设计的里程计计算模块，适用于同时具备轮子编码器与 IMU 的硬件平台。
 */
class WheelOdometry : public rclcpp::Node {
private:
    using SyncPolicyJointStateImu =
        message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::Imu>;
    using SynchronizerJointStateImu = message_filters::Synchronizer<SyncPolicyJointStateImu>;
public:
    explicit WheelOdometry(std::string node_name, const double wheel_radius, const double chasis_radius);
    ~WheelOdometry();

private:
    void init_parameters();
    void joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr joint_state_msg);
    void joint_state_and_imu_callback(const sensor_msgs::msg::JointState::ConstSharedPtr joint_state_msg,
                                      const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);
    void update_imu(sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);
    void update_joint_state(sensor_msgs::msg::JointState::ConstSharedPtr joint_state);
    bool calculate_odometry(const rclcpp::Duration& duration);
    void publish(const rclcpp::Time& now) const;

    void set_nan_to_zero(double& value);

private:
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // 广播变换

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_; // 发布 /odom 话题
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_; // 获取底盘轮子数据

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::JointState>> msg_ftr_joint_state_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> msg_ftr_imu_sub_;
    std::shared_ptr<SynchronizerJointStateImu> joint_state_imu_sync_;

    double wheel_radius_; // 轮子半径
    double chasis_radius_; // 底盘半径

    // 用于广播 tf2
    std::string frame_id_of_odometry_;
    std::string child_frame_id_of_odometry_;

    bool if_use_imu_;
    bool if_publish_tf_;

    std::array<double, 4> diff_wheel_joint_positions_; // 底盘四个轮子的位置（编码器）的该变量
    double imu_yaw_angle_; // 偏航角

    std::array<double, 3> robot_pose_; // 机器人位置，(x, y) 以及偏航角
    std::array<double, 3> robot_vel_; // 机器人速度，vx、vy、角速度
};

} /* lidar_mapping */
} /* sentry */
} /* liesun */

#endif /* WHEEL_ODOMETRY_HPP_ */
