#ifndef NAV_TESTBENCH_FAKE_LANE_CAMERA_HPP
#define NAV_TESTBENCH_FAKE_LANE_CAMERA_HPP

#include <nav_testbench/fake_sensor_node.hpp>
#include <nav_testbench/utils/simulation_environment.hpp>
#include <nav_testbench/utils/tf2_polygon_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace nav_testbench
{
class FakeLaneCamera : public FakeSensorNode {
public:
    FakeLaneCamera();
private:

    int x_sample_size;
    int y_sample_size;

    geometry_msgs::msg::PolygonStamped image_corners_;

    const std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    const std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    const rclcpp::TimerBase::SharedPtr timer_;
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
    const rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr image_corners_publisher_;

    void timerCallback();

    void publishImageCorners();

    void publishLaneCloud();

    const Eigen::Vector2d getWeightedCenter(double x_idx, double y_idx,
                                            const geometry_msgs::msg::PolygonStamped &transformed_corners);

    const pcl::PointCloud<pcl::PointXYZ> getLanePointCloud(
            const geometry_msgs::msg::PolygonStamped &transformed_corners);

    bool getTransform(geometry_msgs::msg::TransformStamped &transform_out, std::string source_frame,
                 std::string target_frame) const;
};
} // namespace nav_testbench

#endif // NAV_TESTBENCH_FAKE_LANE_CAMERA_HPP