#include <nav_testbench/fake_lane_camera.hpp>

#include <nav_testbench/utils/simulation_environment.hpp>
#include <nav_testbench/utils/tf2_polygon_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <cmath>
#include <Eigen/Core>
#include <boost/filesystem.hpp>

using namespace std::chrono_literals;

namespace nav_testbench
{
    FakeLaneCamera::FakeLaneCamera() : Node("fake_lane_camera")
    , tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock()))
    , tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    , timer_(create_wall_timer(50ms, std::bind(&FakeLaneCamera::timerCallback, this)))
    , cloud_publisher_(create_publisher<sensor_msgs::msg::PointCloud2>("lane_cloud", 10))
    , image_corners_publisher_(create_publisher<geometry_msgs::msg::PolygonStamped>("camera_view", 10))
    {

        declare_parameter("map.path");
        declare_parameter("map.width");
        declare_parameter("map.height");

        declare_parameter("frame");
        robot_frame_ = get_parameter("frame").as_string();
        source_frame_ = "map";

        const std::string map_path = get_parameter("map.path").as_string();
        const double map_width = get_parameter("map.width").as_double();
        const double map_height = get_parameter("map.height").as_double();

        if (boost::filesystem::exists(map_path))
        {
            map_ = std::make_unique<SimulationEnvironment>(map_path, map_width, map_height);
        }
        else
        {
            RCLCPP_FATAL(this->get_logger(), "Map file does not exist at given path: ", map_path.c_str());
            throw std::runtime_error("Map file does not exist at given path");
        }

        geometry_msgs::msg::Point32 points[4];

        declare_parameter("image_corners.bottom_left.x");
        declare_parameter("image_corners.bottom_left.y");
        declare_parameter("image_corners.bottom_right.x");
        declare_parameter("image_corners.bottom_right.y");
        declare_parameter("image_corners.top_left.x");
        declare_parameter("image_corners.top_left.y");
        declare_parameter("image_corners.top_right.x");
        declare_parameter("image_corners.top_right.y");

        points[0].x = get_parameter("image_corners.bottom_left.x").as_double();
        points[0].y = get_parameter("image_corners.bottom_left.y").as_double();

        points[1].x = get_parameter("image_corners.bottom_right.x").as_double();
        points[1].y = get_parameter("image_corners.bottom_right.y").as_double();

        points[3].x = get_parameter("image_corners.top_left.x").as_double();
        points[3].y = get_parameter("image_corners.top_left.y").as_double();

        points[2].x = get_parameter("image_corners.top_right.x").as_double();
        points[2].y = get_parameter("image_corners.top_right.y").as_double();

        image_corners_.polygon.points = {points[0], points[1], points[2], points[3]};

        image_corners_.header.frame_id = robot_frame_;

        declare_parameter("x_sample_size");
        declare_parameter("y_sample_size");

        x_sample_size = get_parameter("x_sample_size").as_int();
        y_sample_size = get_parameter("y_sample_size").as_int();
    }

    void FakeLaneCamera::timerCallback()
    {
        publishImageCorners();
        publishLaneCloud();
    }

    void FakeLaneCamera::publishImageCorners() {
        image_corners_publisher_->publish(image_corners_);
    }

    void FakeLaneCamera::publishLaneCloud() {
        // Get the transform from the map to the robot frame
        geometry_msgs::msg::TransformStamped transform;
        const bool transform_ok = getTransform(transform, source_frame_, robot_frame_);
        if(!transform_ok){
            return;
        }

        // transform the image corner polygon to the robot frame
        geometry_msgs::msg::PolygonStamped transformed_corners;
        tf2::doTransform(image_corners_, transformed_corners, transform);

        pcl::PointCloud<pcl::PointXYZ> cloud_builder = getLanePointCloud(transformed_corners);
        sensor_msgs::msg::PointCloud2 cloud;
        pcl::toROSMsg(cloud_builder, cloud);

        cloud.header.frame_id = source_frame_;
        cloud.header.stamp = this->get_clock()->now();

        geometry_msgs::msg::TransformStamped inverse_transform;
        const bool inv_transform_ok = getTransform(inverse_transform, robot_frame_, source_frame_);
        if(!inv_transform_ok){
            return;
        }

        sensor_msgs::msg::PointCloud2 cloud_out;
        tf2::doTransform(cloud, cloud_out, inverse_transform);

        cloud_publisher_->publish(cloud_out);
    }

    bool FakeLaneCamera::getTransform(geometry_msgs::msg::TransformStamped &transform_out, std::string source_frame,
                                 std::string target_frame) const {
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform_out = tf_buffer_->lookupTransform(
                    source_frame, target_frame,tf2::TimePointZero);
            return true;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s",
                    source_frame.c_str(),
                    target_frame.c_str(),
                    ex.what());
            return false;
        }

    }

    const pcl::PointCloud<pcl::PointXYZ>
    FakeLaneCamera::getLanePointCloud(const geometry_msgs::msg::PolygonStamped &transformed_corners) {
        pcl::PointCloud<pcl::PointXYZ> cloud_builder;

        for (int x_idx = 0; x_idx <= x_sample_size; x_idx++)
        {
            for (int y_idx = 0; y_idx <= y_sample_size; y_idx++)
            {
                const Eigen::Vector2d point = getWeightedCenter(x_idx, y_idx, transformed_corners);

                const double x = point.x();
                const double y = point.y();

                if (!map_->isPointLine(x, y))
                {
                    continue;
                }

                pcl::PointXYZ pt;
                pt = pcl::PointXYZ();

                pt.x = x;
                pt.y = y;
                pt.z = 0;

                cloud_builder.push_back(pt);
            }
        }
        return cloud_builder;
    }

    const Eigen::Vector2d FakeLaneCamera::getWeightedCenter(const double x_idx, const double y_idx, const geometry_msgs::msg::PolygonStamped &transformed_corners)
    {
        double bottom_x = std::lerp(
            (double)transformed_corners.polygon.points[0].x,
            (double)transformed_corners.polygon.points[1].x,
            ((double)x_idx) / x_sample_size);

        double bottom_y = std::lerp(
            (double)transformed_corners.polygon.points[0].y,
            (double)transformed_corners.polygon.points[1].y,
            ((double)x_idx) / x_sample_size);

        double top_x = std::lerp(
            (double)transformed_corners.polygon.points[3].x,
            (double)transformed_corners.polygon.points[2].x,
            ((double)x_idx) / x_sample_size);
        double top_y = std::lerp(
            (double)transformed_corners.polygon.points[3].y,
            (double)transformed_corners.polygon.points[2].y,
            ((double)x_idx) / x_sample_size);

        double x = std::lerp(
            (double)bottom_x,
            (double)top_x,
            ((double)y_idx) / y_sample_size);
        double y = std::lerp(
            (double)bottom_y,
            (double)top_y,
            ((double)y_idx) / y_sample_size);

        return Eigen::Vector2d(x, y);
    }
} // namespace nav_testbench

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<nav_testbench::FakeLaneCamera>());
    rclcpp::shutdown();
    return 0;
}
