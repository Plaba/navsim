#include <nav_testbench/utils/simulation_environment.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>

namespace nav_testbench
{

SimulationEnvironment::SimulationEnvironment(const std::string& file_path, double width, double height)
: map_image_(cv::imread(file_path, cv::IMREAD_COLOR))
, map_width_(width)
, map_height_(height)
, image_width_(map_image_.cols)
, image_height_(map_image_.rows)
, resolution_(std::min(map_width_ / image_width_, map_height_ / image_height_))
{
}


double SimulationEnvironment::getResolution() const
{
    return resolution_;
}

bool SimulationEnvironment::isPointLine(double x, double y) {
    y = -y; // Image is flipped
    if(x < 0 || x > map_width_ || y < 0 || y > map_height_){
        return false;
    }

    const int x_idx = (x / map_width_) * image_width_;
    const int y_idx = (y / map_height_) * image_height_;

    const uchar b = map_image_.data[map_image_.channels() * (map_image_.cols * y_idx + x_idx) + 0];

    return b > 250;
}

bool SimulationEnvironment::isPointObstacle(double x, double y){
    y = -y;

    if(x < 0 || x > map_width_ || y < 0 || y > map_height_){
        return false;
    }

    const int x_idx = (x / map_width_) * image_width_;
    const int y_idx = (y / map_height_) * image_height_;

    const uchar r = map_image_.data[map_image_.channels() * (map_image_.cols * y_idx + x_idx) + 2];

    return r > 250;
}

SimulationEnvironment::~SimulationEnvironment() = default;

} // namespace nav_testbench
