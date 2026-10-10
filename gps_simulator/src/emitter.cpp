
#include "emitter.hpp"
#include "frame_handler.hpp"


Emitter::Emitter(rclcpp::Node* nh, double t0, const SatelliteParam& param)  //, gen(rd()), d(1, 10) // Call to base class constructor
    : param(param), timestamp(t0), nh_(nh), t0_(t0)
{
    // ROS 2 topic name tokens must not start with a digit: ~/satellite/pos/sat_<n>
    std::string topic_sat = "~/satellite/pos/sat_" + std::to_string(param.satellite_number_);
    position_publisher = nh->create_publisher<satellite_simulator::msg::EmitterMsg>(topic_sat, 1000);
    n = std::sqrt(param.muE / std::pow(param.sa, 3));
    tol = 1e-6;

    std::string topic = "~/gnss/pseudorange/sat_" + std::to_string(param.satellite_number_);
    pseudorange_publisher = nh->create_publisher<satellite_simulator::msg::EmitterMsg>(topic, 1000);

}

Emitter::~Emitter() {}

void Emitter::updateTimestamp() {
    
    bool use_sim_time = false;
    if (nh_->get_parameter("use_sim_time", use_sim_time) && use_sim_time) {
        timestamp = nh_->now().seconds();
    } else {
        timestamp += 0.1; 
    }
}


void Emitter::calculate_position() {

    double t = timestamp - t0_; // time since epoch
    double Mi = param.mu_anomalie + n*t; // mean anomalie
    double E1 = Mi; // E1, E2 = approx eccentric anomalie
    double E2 = E1 - (E1 - param.eccen * std::sin(E1) - Mi) / (1 - param.eccen * std::cos(E1));
    double error = 100;

    while (error > tol) { // Newton Raphson
        E1 = E2;
        E2 = E1 - (E1 - param.eccen * std::sin(E1) - Mi) / (1 - param.eccen * std::cos(E1));
        error = std::abs(E2 - E1);
    }

    double r = param.sa * (1 - param.eccen * std::cos(E2)); // radius
    double b = 2*std::atan(std::sqrt((1-param.eccen)/(1+param.eccen))*std::tan(E2/2)); // true anomalie
    double v = b + param.arg; // argument of latitude
    double X = r*std::cos(v); // coordinate in plane 
    double Y = r*std::sin(v);

    Position_ECEF.x() = X*std::cos(param.RAANangle) - Y*std::cos(param.orbinc)*std::sin(param.RAANangle);
    Position_ECEF.y() = X*std::sin(param.RAANangle) + Y*std::cos(param.orbinc)*std::cos(param.RAANangle);
    Position_ECEF.z() = Y*std::sin(param.orbinc);


    double lat, lon, alt;
    ECEFToLLA(Position_ECEF.x(), Position_ECEF.y(), Position_ECEF.z(), lat, lon, alt);
    Position_geo << lat, lon, alt;

}

void Emitter::emit_pseudorange(Eigen::Vector3d position_receiver_ECEF, Eigen::Quaterniond Rot_enu_receiver) {

    Eigen::Vector3d ecef_los_vec = Position_ECEF - position_receiver_ECEF; //maybe the other way around

    Eigen::Matrix3d R_enu_ecef;

    double lat_receiver, lon_receiver, alt_receiver;
    ECEFToLLA(position_receiver_ECEF.x(), position_receiver_ECEF.y(), position_receiver_ECEF.z(), lat_receiver, lon_receiver, alt_receiver); // to have LLA coord of the lin point for ENU

    R_enu_ecef << -std::sin(lon_receiver), std::cos(lon_receiver), 0,
        -std::sin(lat_receiver)*std::cos(lon_receiver), -std::sin(lat_receiver)*std::sin(lon_receiver), std::cos(lat_receiver),
        std::cos(lat_receiver)*std::cos(lon_receiver), std::cos(lat_receiver)*std::sin(lon_receiver), std::sin(lat_receiver); 
        // https://gssc.esa.int/navipedia/index.php/Transformations_between_ECEF_and_ENU_coordinates 

    Eigen::Vector3d ENU_los_vec = R_enu_ecef * (ecef_los_vec); // los vector in body frame


    Eigen::Matrix3d R_body_ENU = Rot_enu_receiver.toRotationMatrix().transpose();

    Eigen::Vector3d body_los_vec = R_body_ENU*ENU_los_vec; // check if here is correct and we have satellite in receiver frame.
    
    
    elevation_angle = std::atan2(body_los_vec(2), std::sqrt(std::pow(body_los_vec(0), 2) + std::pow(body_los_vec(1), 2))); // this is the elevation angle with respect to the linearization point, not the camera at this instant

    if(elevation_angle>0.1745) // above horizon of lin point by at least 10 degrees
    {
        azimuth_angle = std::atan2(body_los_vec(1), body_los_vec(0));
        above_horizon = true;
        pseudorange = ecef_los_vec.norm();
        publish();
    } else {
        above_horizon = false;
        pseudorange = 0;
    }


}

double Emitter::calculate_pseudorange(Eigen::Vector3d position_receiver_ECEF) {

    Eigen::Vector3d line_of_sight = Position_ECEF - position_receiver_ECEF;
    pseudorange = line_of_sight.norm();
    return pseudorange;
}

void Emitter::process() {
    updateTimestamp() ;
    calculate_position();
    publish();
}

void Emitter::publish() {
    satellite_simulator::msg::EmitterMsg msg;
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
    msg.header.frame_id = "ECEF";
    msg.range.data = pseudorange;
    msg.position_ecef.x = Position_ECEF.x();
    msg.position_ecef.y = Position_ECEF.y();
    msg.position_ecef.z = Position_ECEF.z();
    msg.position_geo.x = Position_geo.x();
    msg.position_geo.y = Position_geo.y();
    msg.position_geo.z = Position_geo.z();

    msg.elevation_angle.data = elevation_angle;
    msg.azimuth_angle.data = azimuth_angle;
    msg.idx_satellite.data = param.satellite_number_;
    pseudorange_publisher->publish(msg);

    satellite_simulator::msg::EmitterMsg position_msg;

    position_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));

    position_msg.header.frame_id = "ECEF";

    position_msg.position_geo.x = Position_geo.x();
    position_msg.position_geo.y = Position_geo.y();
    position_msg.position_geo.z = Position_geo.z();

    position_msg.position_ecef.x = Position_ECEF.x();
    position_msg.position_ecef.y = Position_ECEF.y();
    position_msg.position_ecef.z = Position_ECEF.z();



    position_publisher->publish(position_msg);

}




