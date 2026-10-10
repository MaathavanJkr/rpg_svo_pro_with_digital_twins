#ifndef EMITTER_HPP
#define EMITTER_HPP

#include <Eigen/Dense>
#include <ceres/ceres.h>

#include <random>
#include <iostream>

#include "constellation.hpp"
#include <rclcpp/rclcpp.hpp>
#include "satellite_simulator/msg/emitter_msg.hpp"
#include <Eigen/Dense>
#include "satellite_param.hpp"
#include <thread>
#include <cmath>
#include <sstream>

class Emitter  {
public:

    Emitter(rclcpp::Node* nh, double t0, const SatelliteParam& param);

    ~Emitter();

    void emit_pseudorange(Eigen::Vector3d position_receiver_ECEF, Eigen::Quaterniond orientation_receiver_enu);
 
    void publish() ;

    void process() ;
    rclcpp::Publisher<satellite_simulator::msg::EmitterMsg>::SharedPtr position_publisher;

    SatelliteParam param;

    bool above_horizon = false;
    double pseudorange;
    double timestamp;
    Eigen::Vector3d Position_ECEF;

private:

    rclcpp::Node* nh_;
    rclcpp::Publisher<satellite_simulator::msg::EmitterMsg>::SharedPtr pseudorange_publisher;

    double calculate_elevation_angle();

    double calculate_pseudorange(Eigen::Vector3d position_receiver_ECEF);
  

    double t0_;
    
    double n;

    double tol;

    Eigen::Vector3d Position_geo;
    Eigen::Vector3d Linearization_pt_;

    Eigen::Matrix3d R_ecef_enu;
    

    Eigen::Vector3d Initial_ECEF;

    double elevation_angle;
    double azimuth_angle;

    void updateTimestamp() ;

    void calculate_position() ;

};

#endif // EMITTER_HPP
