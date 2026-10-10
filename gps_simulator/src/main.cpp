#include "gnss.hpp"
#include "receiver.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("GNSSSimulator");

    std::string constellation_file = nh->declare_parameter<std::string>("constellation_file", "");
    std::string config_path = nh->declare_parameter<std::string>("config_path", "");

    YAML::Node config = YAML::LoadFile(config_path);
    double max_time_before_reset = config["max_time_before_resetting_error"].as<double>();
    double max_distance_before_reset = config["max_distance_before_resetting_error"].as<double>();
    std::string ground_truth_topic = config["ground_truth_topic"].as<std::string>();

    rclcpp::Rate loop_rate(5);
    rclcpp::Time last_called = nh->now();

    GNSS gnss(constellation_file,config_path, nh);
    gnss.startAllEmitters(ground_truth_topic);

    Receiver receiver(gnss, config_path, nh);
    Eigen::Vector3d old_position = gnss.receiver_enu_gt;
    bool reset_pseusorange_errors = true;

    while (rclcpp::ok()) {
        rclcpp::Time current_time = nh->now();
        rclcpp::Duration elapsed = current_time - last_called;

        receiver.setReceiverECEF(gnss.receiver_ecef_gt);
        receiver.setReceiverENU(gnss.receiver_enu_gt);

        if ((old_position - gnss.receiver_enu_gt).norm() > max_distance_before_reset) {
            reset_pseusorange_errors = true; 
            std::cout << "Resetting random values due to distance" << std::endl;
            old_position = gnss.receiver_enu_gt;
        }

        if ((elapsed.seconds() >= max_time_before_reset) || reset_pseusorange_errors) {
            reset_pseusorange_errors = false;
            receiver.generateRandomDoubles();
            last_called = current_time;
        }

        receiver.trilateratesatellites(gnss);
        receiver.publish();
        rclcpp::spin_some(nh);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

