#ifndef NAV_TESTBENCH_FAKE_LIDAR_HPP
#define NAV_TESTBENCH_FAKE_LIDAR_HPP

#include <nav_testbench/utils/simulation_environment.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_testbench/fake_sensor_node.hpp>

using namespace std::chrono_literals;

namespace nav_testbench
{

class FakeLidar : public FakeSensorNode
{
public:
    FakeLidar();
private:
    void timerCallback();

    const rclcpp::Duration transform_timeout_ = rclcpp::Duration::from_seconds(0.5);
    const std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    const std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    const rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    const rclcpp::TimerBase::SharedPtr timer_;

    float getRayIntersection(const geometry_msgs::msg::TransformStamped &transformStamped, float angle,
                             float min_range, float max_range);

    sensor_msgs::msg::LaserScan getDefaultLaserScan();

    bool getCurrentTransform(geometry_msgs::msg::TransformStamped &transform_out) const;
};

} // namespace nav_testbench

#endif // NAV_TESTBENCH_FAKE_LIDAR_HPP