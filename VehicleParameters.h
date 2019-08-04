#include <cmath>
#include "MathConstants.h"

#ifndef vehicle_parameters__H
#define vehicle_parameters__H

// vehicle properties
const double MASS = 4.0;
const double HEIGHT = 0.15;
const double WIDTH = 0.15;
const double DEPTH = 0.3;
const double L1 = 0.05;
const double L2 = 0.21;
const double L3 = 0.05;
const double L4 = 0.21;
const double L5 = 0.53;
const double Ixx = (1.0/12.0) * MASS * (HEIGHT * HEIGHT + DEPTH * DEPTH);
const double Iyy = (1.0/12.0) * MASS * (WIDTH * WIDTH + DEPTH * DEPTH);
const double Izz = (1.0/12.0) * MASS * (WIDTH * WIDTH + HEIGHT * HEIGHT);
const double Ixz = 0.00;
const double drag_factor_x = 4.0;
const double drag_factor_y = 5.0;
const double drag_factor_z = 1.0;

// magnetic field parameters
const double MAG_ELE = 0.0;
const double MAG_AZI = 0.0;

//---------------------------------------------------

// sensor noise
const double acc_noise_bia_x = -0.17;
const double acc_noise_bia_y = 0.02;
const double acc_noise_bia_z = -0.12;
const double acc_noise_std_x = 0.01;
const double acc_noise_std_y = 0.01;
const double acc_noise_std_z = 0.01;

const double gyr_noise_bia_x = -0.1;
const double gyr_noise_bia_y = -0.03;
const double gyr_noise_bia_z = 0.21;
const double gyr_noise_std_x = 0.035;
const double gyr_noise_std_y = 0.035;
const double gyr_noise_std_z = 0.035;

const double mag_noise_std = 0.035;
const double pos_noise_std = 0.5;
const double vel_noise_std = 0.25;
const double alt_noise_std = 0.5;

// ekf parameters
const double q_pos  = 0.001;
const double q_vel  = 0.002;
const double q_att  = 0.000001;
const double q_alt  = 0.001;
const double q_dalt = 0.002;
const double q_a_bia  = 0.000025;
const double q_g_bia  = 0.00001;

const double r_pos  = 1.0;
const double r_vel  = 0.5;
const double r_att  = 0.001;
const double r_alt  = 1.0;
const double r_dalt = 5.0;

const double p_pos  = 1.0;
const double p_vel  = 0.25;
const double p_att  = 0.001;
const double p_alt  = 1.0;
const double p_dalt = 0.25;
const double p_a_bia  = 0.00000001;
const double p_g_bia  = 0.00000001;

// // sensor noise
// const double acc_noise_bia_x = 0.0;
// const double acc_noise_bia_y = 0.0;
// const double acc_noise_bia_z = 0.0;
// const double acc_noise_std_x = 0.0;
// const double acc_noise_std_y = 0.0;
// const double acc_noise_std_z = 0.0;
//
// const double gyr_noise_bia_x = 0.0;
// const double gyr_noise_bia_y = 0.0;
// const double gyr_noise_bia_z = 0.0;
// const double gyr_noise_std_x = 0.0;
// const double gyr_noise_std_y = 0.0;
// const double gyr_noise_std_z = 0.0;
//
// const double mag_noise_std = 0.0;
// const double pos_noise_std = 0.0;
// const double vel_noise_std = 0.0;
// const double alt_noise_std = 0.0;
//
// // ekf parameters
// const double q_pos  = 0.001;
// const double q_vel  = 0.002;
// const double q_att  = 0.000001;
// const double q_alt  = 0.001;
// const double q_dalt = 0.002;
// const double q_a_bia  = 0.000025;
// const double q_g_bia  = 0.00001;
//
// const double r_pos  = 1.0;
// const double r_vel  = 0.5;
// const double r_att  = 0.001;
// const double r_alt  = 1.0;
// const double r_dalt = 5.0;
//
// const double p_pos  = 1.0;
// const double p_vel  = 0.25;
// const double p_att  = 0.001;
// const double p_alt  = 1.0;
// const double p_dalt = 0.25;
// const double p_a_bia  = 0.00000001;
// const double p_g_bia  = 0.00000001;

//---------------------------------------------------

// controller properties
const double K_alt = 0.5;
const double K_dalt = 3.0;
const double K_i_dalt = 1.0;
const double vertical_speed_limit_upper = 2.5;
const double vertical_speed_limit_lower = -2.5;
const double AltitudeControlDeadband = 0.1;
const double AltitudeHoldVerticalSpeedThreshold = 0.5;
const double thrust_factor_limit = 2.0;
const double thrust_command_limit = 10.0; // N

const double wpcl = 8.5;
const double zpcl = 0.95;
const double K_i_p = 0.15;
const double roll_command_limit = 30.0 * DEG2RAD; // rad

const double wqcl = 6.0;
const double zqcl = 0.95;
const double K_i_q = 0.15;
const double pitch_command_limit = 30.0 * DEG2RAD; // rad
const double pitch_rate_command_limit = 15.0 * DEG2RAD; // rad

const double YawRateControlDeadband = 10.0 * DEG2RAD; // yaw rate input threshold to engage heading hold
const double YawHoldYawRateThreshold =  10.0 * DEG2RAD; // yaw rate command threshold for heading hold

const double wrcl = 4.0;
const double zrcl = 0.95;
const double K_i_r = 0.15;

const double wrrcl = 7.0;
const double K_i_rr = 0.0;
const double yaw_rate_command_limit = 60.0 * DEG2RAD; // rad

#endif
