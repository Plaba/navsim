#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <cmath>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace nav_testbench
{

class FakeRobotOdometry : public rclcpp::Node
{
public:
    FakeRobotOdometry();
private:

    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);
    geometry_msgs::msg::Quaternion getOdomQuaternion() const;
    void updatePosition(const geometry_msgs::msg::Twist::SharedPtr command);
    void publishTransform();
    void publishOdometry();
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr command);
    void timerCallback();

    double odom_x;
    double odom_y;
    double odom_th;
    double odom_vx{};
    double odom_vy{};
    double odom_vth{};

    const rclcpp::Duration transform_timeout_ = 0.5s;
    const std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    const std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    const rclcpp::TimerBase::SharedPtr timer_{nullptr};
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_{nullptr};
    const rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::Time last_cmd_vel_received;    
};

}  // namespace nav_testbench