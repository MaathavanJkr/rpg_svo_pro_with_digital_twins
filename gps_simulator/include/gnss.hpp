#ifndef GPS_SIM_HPP
#define GPS_SIM_HPP

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include "constellation.hpp"
#include <Eigen/Dense>
#include <vector>
#include <array>
#include <iostream>
#include <ceres/ceres.h>
#include <fstream>
#include <string>
#include <limits>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <yaml-cpp/yaml.h>
#include <thread>
#include "frame_handler.hpp"

class Emitter; 

class GNSS {
public:
    GNSS(const std::string& constallation_config_fn, const std::string& reciever_config_fn, const rclcpp::Node::SharedPtr& nh) ;

    ~GNSS();

    void startAllEmitters(std::string ground_truth_topic);

    Eigen::Vector3d receiver_enu_gt;
    Eigen::Vector3d receiver_ecef_gt;

    int number_visible_satellites;
    std::vector<Emitter> emitters; // TODO : map the PRN to the emitters/satellite for more clarity
    
    Eigen::Vector3d linearization_Point;

private:
    rclcpp::Node::SharedPtr nh_;
    
    Eigen::Vector3d Initial_ECEF;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr receiver_enu_gt_sub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr ecef_receiver_gt_pub;

    std::thread processing_thread;
    std::mutex emitters_mtx; 

    bool running;

    void EphemerisLoop();

    void groundtruthCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);   

};


#endif // GPS_SIM_HPP