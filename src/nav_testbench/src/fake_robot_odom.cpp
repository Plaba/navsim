#include <nav_testbench/fake_robot_odom.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <chrono>
#include <memory>
#include <cmath>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace nav_testbench
{

FakeRobotOdometry::FakeRobotOdometry() : Node("fake_robot_odom")
, tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock()))
, tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
, timer_(create_wall_timer(50ms, [this] { timerCallback(); }))
, publisher_(create_publisher<nav_msgs::msg::Odometry>("odom", 10))
, cmd_vel_sub_(create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",rclcpp::SensorDataQoS(),
        std::bind(&FakeRobotOdometry::cmdVelCallback,this,_1)))
{
    declare_parameter<double>("start_x");
    declare_parameter<double>("start_y");
    declare_parameter<double>("start_th");

    odom_x = get_parameter("start_x").as_double();
    odom_y = get_parameter("start_y").as_double();
    odom_th = get_parameter("start_th").as_double();

    last_cmd_vel_received = rclcpp::Time((int64_t)0, now().get_clock_type());

    callback_handle_ = add_on_set_parameters_callback(
        std::bind(&FakeRobotOdometry::parametersCallback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult FakeRobotOdometry::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for(auto parameter : parameters){
        if(parameter.get_name() == "start_x"){
            odom_x = parameter.as_double();
        } else if(parameter.get_name() == "start_y"){
            odom_y = parameter.as_double();
        } else if(parameter.get_name() == "start_th") {
            odom_th = parameter.as_double();
        }
    }

    odom_vth = odom_vx = odom_vy = 0;

    return result;
}

geometry_msgs::msg::Quaternion FakeRobotOdometry::getOdomQuaternion() const{
    tf2::Quaternion q;
    geometry_msgs::msg::Quaternion odom_quat;

    q.setRPY(0, 0, odom_th);
    odom_quat.x = q.getX();
    odom_quat.y = q.getY();
    odom_quat.z = q.getZ();
    odom_quat.w = q.getW();
    return odom_quat;
}

void FakeRobotOdometry::updatePosition(const geometry_msgs::msg::Twist::SharedPtr command){

    double command_vth = command->angular.z;
    double command_vx = command->linear.x;

    if(last_cmd_vel_received.nanoseconds() == 0){
        last_cmd_vel_received = now();
        return;
    }

    double dt = (now() - last_cmd_vel_received).seconds();
    last_cmd_vel_received = now();

    double dth = odom_vth * dt;
    double dx = hypot(odom_vx, odom_vy) * dt;
    
    if (fabs(dth) < 1e-6){
        odom_x += odom_vx * dt;
        odom_y += odom_vy * dt;
    } else {
        double odom_th_old = odom_th;
        odom_th += dth;
        
        double r = dx / dth;

        odom_x += r * (sin(odom_th) - sin(odom_th_old));
        odom_y += - r * (cos(odom_th) - cos(odom_th_old));
    }

    odom_vth = command_vth;
    odom_vx = command_vx*cos(odom_th);
    odom_vy = command_vx*sin(odom_th);
}

void FakeRobotOdometry::publishTransform(){
    geometry_msgs::msg::TransformStamped odom_trans;

    auto odom_quat = getOdomQuaternion();
    odom_trans.header.stamp = now() + transform_timeout_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = odom_x;
    odom_trans.transform.translation.y = odom_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    tf_broadcaster_->sendTransform(odom_trans);
}

void FakeRobotOdometry::publishOdometry(){
    nav_msgs::msg::Odometry odom;

    auto odom_quat = getOdomQuaternion();
    
    //set the position
    odom.pose.pose.position.x = odom_x;
    odom.pose.pose.position.y = odom_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.pose.covariance[0] = 8e-3;
    odom.pose.covariance[7] = 8e-3;
    odom.pose.covariance[14] = 1e-3;
    odom.pose.covariance[21] = 1e-3;
    odom.pose.covariance[28] = 1e-3;
    odom.pose.covariance[35] = 8e-3;
    
    /*
    [8e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 8e-3, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1e-8, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1e-8, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1e-8, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 8e-3];*/
    //set the velocity
    odom.header.stamp = now() + transform_timeout_;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = odom_vx;
    odom.twist.twist.linear.y = odom_vy;
    odom.twist.twist.angular.z = odom_vth;

    //publish the message
    publisher_->publish(odom);
}

void FakeRobotOdometry::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr command){
    updatePosition(command);
}

void FakeRobotOdometry::timerCallback(){
    publishTransform();
    publishOdometry();
}
} // namespace nav_testbench

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<nav_testbench::FakeRobotOdometry>());
    rclcpp::shutdown();
    return 0;
}
