#ifndef CONSTELLATION_H
#define CONSTELLATION_H

#include <vector>
#include <cmath>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include "satellite_param.hpp"
#include <thread>
#include <cmath>
#include <sstream>

class Constellation {
public:
    Constellation(const std::string& filename, rclcpp::Node* nh);
    ~Constellation();

    void startAllSatellites();
    void stopAllSatellites();

private:
    std::thread processing_thread;

    std::mutex mtx;
    
    bool running;

    void processingLoop();
};

#endif // CONSTELLATION_H
