#ifndef FRAME_HANDLER_HPP
#define FRAME_HANDLER_HPP



#include "constellation.hpp"
#include <Eigen/Dense>
#include <vector>
#include <array>
#include <iostream>
#include <ceres/ceres.h>
#include <fstream>
#include <string>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <limits>
#include <GeographicLib/LocalCartesian.hpp>
#include <yaml-cpp/yaml.h>

// LLA = geocentric coordinate : latitude, longitude, altitude
void LLAtoENU(double lat, double lon, double alt, double refLat, double refLon, double refAlt, double &east, double &north, double &up);

void ENUtoECEF(double east, double north, double up, double refLat, double refLon, double refAlt, double &ecefX, double &ecefY, double &ecefZ);

void ENUToLLA(double refLon, double refLat, double refAlt, 
                     double enuE, double enuN, double enuU,
                     double& lon, double& lat, double& alt) ;

void LLAToECEF(double lat, double lon, double alt, double& x, double& y, double& z);

void ECEFToLLA(double x, double y, double z, double& lat, double& lon, double& alt);

#endif // FRAME_HANDLER_HPP