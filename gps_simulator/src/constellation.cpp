#include "constellation.hpp"

Constellation::Constellation(const std::string& filename, rclcpp::Node* nh) {
    YAML::Node config = YAML::LoadFile(filename);
    int numSatellites = config["sqrt_sa"].size(); // Assuming this represents the number of satellites

    for (int i = 0; i < numSatellites; i++) {
        SatelliteParam sat_param(filename, i);
        satellites.emplace_back(nh, nh->now().seconds(), sat_param);
    }
}

Constellation::~Constellation() {
    stopAllSatellites();
    if (processing_thread.joinable()) {
        processing_thread.join();
    }
}

void Constellation::startAllSatellites() {
    running = true;
    processing_thread = std::thread(&Constellation::processingLoop, this);
}

void Constellation::stopAllSatellites() {
    running = false;
}
