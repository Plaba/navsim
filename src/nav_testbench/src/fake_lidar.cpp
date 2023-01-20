#include <nav_testbench/fake_lidar.hpp>

#include <nav_testbench/utils/simulation_environment.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <chrono>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/filesystem.hpp>
 

using namespace std::chrono_literals;

namespace nav_testbench
{

FakeLidar::FakeLidar() : Node("fake_lidar")
, tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock()))
, tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
, publisher_(create_publisher<sensor_msgs::msg::LaserScan>("scan", 10))
, timer_(create_wall_timer(50ms, [this] { timerCallback(); }))
{
    declare_parameter("frame");
    declare_parameter("map.path");
    declare_parameter("map.width");
    declare_parameter("map.height");

    robot_frame_ = get_parameter("frame").as_string();
    source_frame_ = "map";

    const std::string map_path = get_parameter("map.path").as_string();
    const double map_width = get_parameter("map.width").as_double();
    const double map_height = get_parameter("map.height").as_double();

    if (boost::filesystem::exists(map_path)) {
        map_ = std::make_unique<SimulationEnvironment>(map_path, map_width, map_height);
    } else {
        RCLCPP_FATAL(this->get_logger(), "Map file does not exist at given path: ",
                     get_parameter("map.path").as_string().c_str());
        throw std::runtime_error("Map file does not exist at given path");
    }

}

void FakeLidar::timerCallback(){

    geometry_msgs::msg::TransformStamped robot_origin;

    bool good_transform = getCurrentTransform(robot_origin);
    if(!good_transform){
        return;
    }

    sensor_msgs::msg::LaserScan scan = getDefaultLaserScan();
    scan.header.stamp = now() + transform_timeout_;

    float angle = scan.angle_min;
    for(uint i = 0; i < scan.ranges.size(); i++){
        scan.ranges[i] = getRayIntersection(robot_origin, angle,
                                            scan.range_min, scan.range_max);
        angle += scan.angle_increment;
    }

    publisher_->publish(scan);
}

bool FakeLidar::getCurrentTransform(geometry_msgs::msg::TransformStamped &transform_out) const {
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
        transform_out = tf_buffer_->lookupTransform(
                source_frame_,
                robot_frame_,
                tf2::TimePointZero
        );
        return true;
    } catch (tf2::TransformException & ex) {
        RCLCPP_INFO(
                get_logger(),
                "Could not transform %s to %s: %s",
                source_frame_.c_str(),
                robot_frame_.c_str(),
                ex.what()
        );
        return false;
    }
}

sensor_msgs::msg::LaserScan FakeLidar::getDefaultLaserScan() const {
    //https://github.com/AbhiRP/Fake-LaserScan/blob/master/src/laserscan_publisher.cpp

    sensor_msgs::msg::LaserScan scan;
    const unsigned int num_readings = 1081;
    const float laser_frequency = 50.0;

    scan.header.frame_id = robot_frame_;
    scan.angle_min = -135 * (M_PI/180);
    scan.angle_max = 135 * (M_PI/180);
    scan.angle_increment = 0.25 * (M_PI/180);
    scan.time_increment = (1 / laser_frequency * num_readings);
    scan.range_min = 0.02;
    scan.range_max = 20.0;
    scan.scan_time = 1 / laser_frequency;

    scan.ranges.resize(num_readings);

    return scan;
}

float FakeLidar::getRayIntersection(const geometry_msgs::msg::TransformStamped &transformStamped, float angle,
                                    float min_range, float max_range) {
    geometry_msgs::msg::PointStamped pointStamped;
    geometry_msgs::msg::PointStamped pointStamped_transformed;
    pointStamped.header.frame_id = source_frame_;
    pointStamped.header.stamp = transformStamped.header.stamp;
    pointStamped.point.z = 0;

    float intersection_length=0;

    const float resolution = map_->getResolution();
    for(float ray_length=min_range; ray_length < max_range; ray_length += resolution){
        pointStamped.point.x = ray_length * cos(angle);
        pointStamped.point.y = ray_length * sin(angle);

        tf2::doTransform(pointStamped, pointStamped_transformed, transformStamped);
        if(map_->isPointObstacle(pointStamped_transformed.point.x, pointStamped_transformed.point.y)){
            intersection_length = ray_length;
            break;
        }
    }
    return intersection_length;
}
}  // namespace nav_testbench


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<nav_testbench::FakeLidar>());
    rclcpp::shutdown();
    return 0;
}