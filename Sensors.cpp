#include "Sensors.h"
#include "Utils.h"
#include "VehicleParameters.h"
#include "MathConstants.h"
#include <cmath>
#include <iostream>
#include <iomanip>
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
    std::normal_distribution<double> dist_acc_x(0.0, acc_noise_std_x); // mean, std
    std::normal_distribution<double> dist_acc_y(0.0, acc_noise_std_y);
    std::normal_distribution<double> dist_acc_z(0.0, acc_noise_std_z);
    std::normal_distribution<double> dist_gyr_p(0.0, gyr_noise_std_x);
    std::normal_distribution<double> dist_gyr_q(0.0, gyr_noise_std_y);
    std::normal_distribution<double> dist_gyr_r(0.0, gyr_noise_std_z);
    std::normal_distribution<double> dist_gyr_phi(0.0, mag_noise_std);
    std::normal_distribution<double> dist_gyr_tht(0.0, mag_noise_std);
    std::normal_distribution<double> dist_gyr_psi(0.0, mag_noise_std);

    Utils Utils;

    VectorXd IMU_measurements(9);
    IMU_measurements.setZero();

    // Accelerometer measurements
    IMU_measurements(0) =  (X(5)*X(1) - X(4)*X(2) + U(0)/MASS) + dist_acc_x(generator) + acc_noise_bia_x;
    IMU_measurements(1) = (-X(5)*X(0) + X(3)*X(2) + U(1)/MASS) + dist_acc_y(generator) + acc_noise_bia_y;
    IMU_measurements(2) =  (X(4)*X(0) - X(3)*X(1) + U(2)/MASS) + dist_acc_z(generator) + acc_noise_bia_z;

    // Gyroscope measurements
    IMU_measurements(3) = X(3) + dist_gyr_p(generator) + gyr_noise_bia_x;
    IMU_measurements(4) = X(4) + dist_gyr_q(generator) + gyr_noise_bia_y;
    IMU_measurements(5) = X(5) + dist_gyr_r(generator) + gyr_noise_bia_z;

    //----------------------------------------------------------------------

    // Magnetometer measurements (returned as roll, pitch and yaw)
    IMU_measurements(6) = X(6) + dist_gyr_phi(generator);
    IMU_measurements(7) = X(7) + dist_gyr_tht(generator);
    IMU_measurements(8) = X(8) + dist_gyr_psi(generator);
    Utils.PiMinusPi(IMU_measurements(8));

    //----------------------------------------------------------------------

//     // add noise to true attitude
//     double phi_temp = X(6) + dist_gyr_phi(generator);
//     double tht_temp = X(7) + dist_gyr_tht(generator);
//     double psi_temp = X(8) + dist_gyr_psi(generator);
//     Utils.PiMinusPi(psi_temp);
//
//     // unit vector of north in magetic field frame
//     Vector3d U_mag0;
//     U_mag0 << 1.0, 0.0, 0.0;
//
//     // navigation to magnetic
//     Matrix3d C_mn = Utils.DirectionCosineMatrix(0.0, MAG_ELE, MAG_AZI);
//
//     // navigation to body
//     Matrix3d C_bn = Utils.DirectionCosineMatrix(phi_temp, tht_temp, psi_temp);
//     VectorXd quat = Utils.EulerToQuaternion(phi_temp, tht_temp, psi_temp);
//
//     // unit vector describing magnetic north vector in body axis
//     Vector3d U_mag = C_bn * C_mn.transpose() * U_mag0;
//
//     // Magnetometer measurements (returned as roll, pitch and yaw)
//     IMU_measurements(6) = U_mag(0);
//     IMU_measurements(7) = U_mag(1);
//     IMU_measurements(8) = U_mag(2);
//
//     // -- TEST --
//
// //     tht_temp -= 0.0*DEG2RAD;
// //     psi_temp += 0.0*DEG2RAD;
// //
// //     // navigation to body
// //     Matrix3d C_bn2 = Utils.DirectionCosineMatrix(phi_temp, tht_temp, psi_temp);
// //
// //     // predicted mag vector
// //     Vector3d U_mag_pred = C_bn2 * C_mn.transpose() * U_mag0;
// //
// //     // rotation correction
// // //     Matrix3d C_bn_mag_correction = Utils.VectorRotation(U_mag_pred, U_mag);
// //     VectorXd quat_mag_correction = Utils.QuaternionTwoVectors(U_mag, U_mag_pred);
// //     VectorXd quat_mag = Utils.QuaternionProduct(quat_mag_correction, quat);
// //     Utils.QuaternionNormalise(quat_mag_correction);
// //     Vector3d euler_mag = Utils.QuatToEuler(quat_mag_correction);
//
// //     std::cout << std::setprecision(3)
// //               << std::fixed
// //               << quat_mag_correction(0) << ", "
// //               << quat_mag_correction(1) << ", "
// //               << quat_mag_correction(2) << ", "
// //               << quat_mag_correction(3) << std::endl;
//
// //     // magnetic attitude measurement
// //     Matrix3d C_bn_mag = C_bn_mag_correction * C_bn;
// //     Vector3d euler_mag = Utils.DirectionCosineMatrixToEuler(C_bn_mag);
// //
// //     std::cout << std::setprecision(3)
// //               << std::fixed
// //               << IMU_measurements(6) * RAD2DEG << ", " << IMU_measurements(7) * RAD2DEG << ", "<< IMU_measurements(8) * RAD2DEG << ", "
// //               << euler_mag(0) * RAD2DEG << ", " << euler_mag(1) * RAD2DEG << ", " << euler_mag(2) * RAD2DEG << ", " << std::endl;

    //----------------------------------------------------------------------

    return IMU_measurements;
}

VectorXd Sensors::GPS(VectorXd X)
{
    // Normally distributed, gaussian noise
    std::random_device rd{};
    std::mt19937 generator{rd()};
    std::normal_distribution<double> dist_gps_pn(0.0, pos_noise_std);
    std::normal_distribution<double> dist_gps_pe(0.0, pos_noise_std);
    std::normal_distribution<double> dist_gps_pd(0.0, pos_noise_std);
    std::normal_distribution<double> dist_gps_vn(0.0, vel_noise_std);
    std::normal_distribution<double> dist_gps_ve(0.0, vel_noise_std);
    std::normal_distribution<double> dist_gps_vd(0.0, vel_noise_std);

    Utils Utils;

    VectorXd GPS_measurements(6);
    GPS_measurements.setZero();

    // Position (NED)
    GPS_measurements(0) = X(9)  + dist_gps_pn(generator);  // North
    GPS_measurements(1) = X(10) + dist_gps_pe(generator); // East
    GPS_measurements(2) = X(11) + dist_gps_pd(generator); // Down

    // Velocity (NED)
    Matrix3d C_bn = Utils.DirectionCosineMatrix(X(6), X(7), X(8)); // inertial to body transformation
    Vector3d SpeedVec;
    SpeedVec << X(0), X(1), X(2); // body velocity vector
    Vector3d PosRates = C_bn.transpose() * SpeedVec; // NED velocity vector

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
    std::normal_distribution<double> dist_baro_alt(0.0, alt_noise_std);

    double Barometer_measurement = -X(11) + dist_baro_alt(generator); // barometric altitude

    return Barometer_measurement;
}
