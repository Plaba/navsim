#ifndef SIMULATION_ENVIRONMENT_HPP
#define SIMULATION_ENVIRONMENT_HPP

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>

namespace nav_testbench
{

class SimulationEnvironment
{
public:
    SimulationEnvironment(const std::string& file_path, double width, double height);
    bool isPointObstacle(double x, double y);
    bool isPointLine(double x, double y);

    double getResolution() const;

    ~SimulationEnvironment();
private:
   const cv::Mat map_image_;

   const double map_width_, map_height_;
   const int image_width_, image_height_;
   const double resolution_;
};
    
} // namespace nav_testbench

#endif