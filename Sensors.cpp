#include "Sensors.h"
#include "Utils.h"
#include "VehicleParameters.h"
#include <cmath>
#include <iostream>
#include <random>

Sensors::Sensors()
{
}

Sensors::~Sensors()
{
}

VectorXd Sensors::IMU(VectorXd X, VectorXd U)
{
    // Normally distributed, gaussian noise
    std::random_device rd{};
    std::mt19937 generator{rd()};
    std::normal_distribution<double> dist_acc_x(0.0, 0.0);
    std::normal_distribution<double> dist_acc_y(0.0, 0.0);
    std::normal_distribution<double> dist_acc_z(0.0, 0.0);
    std::normal_distribution<double> dist_gyr_p(0.0, 0.0);
    std::normal_distribution<double> dist_gyr_q(0.0, 0.0);
    std::normal_distribution<double> dist_gyr_r(0.0, 0.0);
    std::normal_distribution<double> dist_gyr_phi(0.0, 0.0);
    std::normal_distribution<double> dist_gyr_tht(0.0, 0.0);
    std::normal_distribution<double> dist_gyr_psi(0.0, 0.0);
    
    VectorXd IMU_measurements(9);
    IMU_measurements.setZero();    
   
    // Accelerometer measurements
    IMU_measurements(0) =  (X(5)*X(1) - X(4)*X(2) + U(0)/MASS) + dist_acc_x(generator);
    IMU_measurements(1) = (-X(5)*X(0) + X(3)*X(2) + U(1)/MASS) + dist_acc_y(generator);
    IMU_measurements(2) =  (X(4)*X(0) - X(3)*X(1) + U(2)/MASS) + dist_acc_z(generator);
    
    // Gyroscope measurements
    IMU_measurements(3) = X(3) + dist_gyr_p(generator);
    IMU_measurements(4) = X(4) + dist_gyr_q(generator);
    IMU_measurements(5) = X(5) + dist_gyr_r(generator);

    // Magnetometer measurements (returned as roll, pitch and yaw)
    IMU_measurements(6) = X(6) + dist_gyr_phi(generator);
    IMU_measurements(7) = X(7) + dist_gyr_tht(generator);
    IMU_measurements(8) = X(8) + dist_gyr_psi(generator);
    
    return IMU_measurements;
}

VectorXd Sensors::GPS(VectorXd X)
{
    // Normally distributed, gaussian noise
    std::random_device rd{};
    std::mt19937 generator{rd()};
    std::normal_distribution<double> dist_gps_pn(0.0, 0.0);
    std::normal_distribution<double> dist_gps_pe(0.0, 0.0);
    std::normal_distribution<double> dist_gps_pd(0.0, 0.0);
    std::normal_distribution<double> dist_gps_vn(0.0, 0.0);
    std::normal_distribution<double> dist_gps_ve(0.0, 0.0);
    std::normal_distribution<double> dist_gps_vd(0.0, 0.0);
    
    Utils Utils;
    
    VectorXd GPS_measurements(6);
    GPS_measurements.setZero();
    
    // Position (NED)
    GPS_measurements(0) = X(9)  + dist_gps_pn(generator);  // North
    GPS_measurements(1) = X(10) + dist_gps_pe(generator); // East
    GPS_measurements(2) = X(11) + dist_gps_pd(generator); // Down
    
    // Velocity (NED)
    MatrixXd C_bn = Utils.DirectionCosineMatrix(X(6), X(7), X(8)); // inertial to body transformation
    VectorXd SpeedVec(3);
    SpeedVec << X(0), X(1), X(2); // body velocity vector
    VectorXd PosRates = C_bn.transpose() * SpeedVec; // NED velocity vector
    
    GPS_measurements(3) = PosRates(0) + dist_gps_vn(generator); // Vn
    GPS_measurements(4) = PosRates(1) + dist_gps_ve(generator); // Ve
    GPS_measurements(5) = PosRates(2) + dist_gps_vd(generator); // Vd
    
    return GPS_measurements;
}

double Sensors::Barometer(VectorXd X)
{
    // Normally distributed, gaussian noise
    std::random_device rd{};
    std::mt19937 generator{rd()};
    std::normal_distribution<double> dist_baro_alt(0.0, 0.0);
    
    double Barometer_measurement = -X(11) + dist_baro_alt(generator); // barometric altitude
    
    return Barometer_measurement;
}
