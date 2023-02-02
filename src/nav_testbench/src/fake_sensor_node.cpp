#include <nav_testbench/fake_sensor_node.hpp>
#include <nav_testbench/utils/simulation_environment.hpp>

#include <boost/filesystem.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace nav_testbench
{

FakeSensorNode::FakeSensorNode(const std::string &node_name)
: Node(node_name)
{
    declare_parameter<std::string>("frame");
    declare_parameter<std::string>("map.path");
    declare_parameter<double>("map.width");
    declare_parameter<double>("map.height");

    robot_frame_ = get_parameter("frame").as_string();
    source_frame_ = "map";

    map_path_ = get_parameter("map.path").as_string();
    map_width_ = get_parameter("map.width").as_double();
    map_height_ = get_parameter("map.height").as_double();

    if (boost::filesystem::exists(map_path_)) {
        map_ = std::make_unique<SimulationEnvironment>(map_path_, map_width_, map_height_);
    } else {
        RCLCPP_FATAL(this->get_logger(), "Map file does not exist at given path: %s",
                        map_path_.c_str());
        throw std::runtime_error("Map file does not exist at given path");
    }

    callback_handle_ = add_on_set_parameters_callback(
        std::bind(&FakeSensorNode::parametersCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
        "source: %s; robot: %s", 
        source_frame_.c_str(), robot_frame_.c_str());
}

rcl_interfaces::msg::SetParametersResult FakeSensorNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters){
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    bool update_map = false;

    source_frame_ = "map";

    for (auto parameter : parameters) {
        if (parameter.get_name() == "map.path") {
            map_path_ = parameter.as_string();
            update_map = true;
        } else if (parameter.get_name() == "map.width") {
            map_width_ = parameter.as_double();
            update_map = true;
        } else if (parameter.get_name() == "map.height") {
            map_height_ = parameter.as_double();
            update_map = true;
        } else if(parameter.get_name() == "frame"){
            robot_frame_ = parameter.as_string();
            RCLCPP_INFO(get_logger(),
                "source: %s; robot: %s", 
                source_frame_.c_str(), robot_frame_.c_str());
        } else {
            RCLCPP_WARN(get_logger(),
                "Parameter not set in not: %s", 
                parameter.get_name().c_str());
        }
    }

    if (update_map) {
        if (boost::filesystem::exists(map_path_)) {
            map_ = std::make_unique<SimulationEnvironment>(map_path_, map_width_, map_height_);
        } else if (map_path_ != ""){
            RCLCPP_FATAL(this->get_logger(), "Map file does not exist at given path: %s",
                            map_path_.c_str()
            );
            result.successful = false;
            result.reason = "Map file does not exist at given path";
        } else {
            RCLCPP_WARN(this->get_logger(), "Map file path is empty");
        }
    }

    return result;
}
    
}  // namespace nav_testbench