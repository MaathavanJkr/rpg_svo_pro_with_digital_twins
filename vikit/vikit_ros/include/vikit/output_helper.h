/*
 * output_helper.h
 *
 *  Created on: Jan 20, 2013
 *      Author: cforster
 */

#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <kindr/minimal/quat-transformation.h>

namespace vk {

using Transformation = kindr::minimal::QuatTransformation;

namespace output_helper {

using MarkerPublisher = rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr;

using namespace std;
using namespace Eigen;

void publishTfTransform(
    const Transformation& T,
    const rclcpp::Time& stamp,
    const string& frame_id,
    const string& child_frame_id,
    tf2_ros::TransformBroadcaster& br);

void publishPointMarker(
    const MarkerPublisher& pub,
    const Vector3d& pos,
    const string& ns,
    const rclcpp::Time& timestamp,
    int id,
    int action,
    double marker_scale,
    const Vector3d& color,
    const std::string frame_id = "world",
    rclcpp::Duration lifetime = rclcpp::Duration(0, 0));

void publishLineMarker(
    const MarkerPublisher& pub,
    const Vector3d& start,
    const Vector3d& end,
    const string& ns,
    const rclcpp::Time& timestamp,
    int id,
    int action,
    double marker_scale,
    const Vector3d& color,
    const std::string frame_id = "world",
    rclcpp::Duration lifetime = rclcpp::Duration(0, 0));

void publishArrowMarker(
    const MarkerPublisher& pub,
    const Vector3d& pos,
    const Vector3d& dir,
    double scale,
    const string& ns,
    const rclcpp::Time& timestamp,
    int id,
    int action,
    double marker_scale,
    const Vector3d& color,
    const std::string frame_id = "world");

void publishHexacopterMarker(
    const MarkerPublisher& pub,
    const string& frame_id,
    const string& ns,
    const rclcpp::Time& timestamp,
    int id,
    int action,
    double marker_scale,
    const Vector3d& color);

void publishQuadrocopterMarkers(const MarkerPublisher& pub,
                        const string& frame_id,
                        const string& ns,
                        const rclcpp::Time& timestamp,
                        int id,
                        int action,
                        double marker_scale,
                        const Vector3d& color);

void publishCameraMarker(
    const MarkerPublisher& pub,
    const string& frame_id,
    const string& ns,
    const rclcpp::Time& timestamp,
    int id,
    int action,
    double marker_scale,
    const Vector3d& color);

void publishFrameMarker(
    const MarkerPublisher& pub,
    const Matrix3d& rot,
    const Vector3d& pos,
    const string& ns,
    const rclcpp::Time& timestamp,
    int id,
    int action,
    double marker_scale,
    const std::string frame_id = "world",
    rclcpp::Duration lifetime = rclcpp::Duration(0, 0));

void publishGtsamPoseCovariance(
    const MarkerPublisher& pub,
    const Eigen::Vector3d& mean,
    const Eigen::Matrix3d& R_W_B, // Body in World-Frame
    const Eigen::Matrix<double, 6, 6>& covariance,
    const string& ns,
    int id,
    int action,
    double sigma_scale,
    const Vector4d& color);


} // namespace output_helper
} // namespace vk

