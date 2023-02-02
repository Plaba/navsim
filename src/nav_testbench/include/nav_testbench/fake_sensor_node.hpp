
#ifndef NAV_TESTBENCH_FAKE_SENSOR_NODE_HPP

#define NAV_TESTBENCH_FAKE_SENSOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <nav_testbench/utils/simulation_environment.hpp>

#include <boost/filesystem.hpp>

namespace nav_testbench
{

class FakeSensorNode : public rclcpp::Node
{
public:
    FakeSensorNode(const std::string &node_name);
protected:
    virtual rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);
    
    std::string robot_frame_;
    std::string source_frame_;

    std::string map_path_;
    double map_width_;
    double map_height_;

    std::unique_ptr<SimulationEnvironment> map_;

    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

} // namespace nav_testbench

#endif //NAV_TESTBENCH_FAKE_SENSOR_NODE_HPP

