#include <cmath>
#include "MathConstants.h"

#ifndef vehicle_parameters__H
#define vehicle_parameters__H

// vehicle properties
const double MASS = 4.5;

const double L1 = 0.10;
const double L2 = 0.46;
const double L3 = 0.10;
const double L4 = 0.46;
const double L5 = 0.60;
const double L6 = 0.05;
const double L7 = 0.05;
const double L8 = 0.02;

// body dimensions (NED)
const double sf1b1 = 0.10;
const double sf1b2 = -0.46;
const double sf1b3 = 0.05;
const double sf2b1 = 0.10;
const double sf2b2 = 0.46;
const double sf2b3 = 0.05;
const double sf3b1 = -0.60;
const double sf3b2 = 0.0;
const double sf3b3 = 0.02;

const double Ixx = 0.13;
const double Iyy = 0.13;
const double Izz = 0.26;
const double Ixz = 0.00;
const double drag_factor_x = 3.0;
const double drag_factor_y = 10.0;
const double drag_factor_z = 1.0;

// magnetic field parameters
const double MAG_ELE = 0.0;
const double MAG_AZI = 0.0;

//---------------------------------------------------

// // sensor noise
// const double acc_noise_bia_x = -0.17;
// const double acc_noise_bia_y = 0.02;
// const double acc_noise_bia_z = -0.12;
// const double acc_noise_std_x = 0.01;
// const double acc_noise_std_y = 0.01;
// const double acc_noise_std_z = 0.01;
//
// const double gyr_noise_bia_x = -0.1;
// const double gyr_noise_bia_y = -0.03;
// const double gyr_noise_bia_z = 0.21;
// const double gyr_noise_std_x = 0.035;
// const double gyr_noise_std_y = 0.035;
// const double gyr_noise_std_z = 0.035;
//
// const double mag_noise_std = 0.035;
// const double pos_noise_std = 0.5;
// const double vel_noise_std = 0.25;
// const double alt_noise_std = 0.5;

//------------------------------------------------------

// sensor noise
const double acc_noise_bia_x = 0.0;
const double acc_noise_bia_y = 0.0;
const double acc_noise_bia_z = 0.0;
const double acc_noise_std_x = 0.0;
const double acc_noise_std_y = 0.0;
const double acc_noise_std_z = 0.0;

const double gyr_noise_bia_x = 0.0;
const double gyr_noise_bia_y = 0.0;
const double gyr_noise_bia_z = 0.0;
const double gyr_noise_std_x = 0.0;
const double gyr_noise_std_y = 0.0;
const double gyr_noise_std_z = 0.0;

const double mag_noise_std = 0.0;
const double pos_noise_std = 0.0;
const double vel_noise_std = 0.0;
const double alt_noise_std = 0.0;

//------------------------------------------------------

// EKF parameters

// process noise
const double q_pos  = 0.005;
const double q_vel  = 0.001;
const double q_att  = 0.000001;
const double q_alt  = 0.001;
const double q_dalt = 0.002;
const double q_a_bia  = 0.000025;
const double q_g_bia  = 0.00001;

// measurement noise
const double r_pos  = 9.0;
const double r_vel  = 0.1;
const double r_att  = 0.05;
const double r_alt  = 3.0;
const double r_dalt = 5.0;

// initial state uncertainties
const double p_pos  = 1.0;
const double p_vel  = 0.25;
const double p_att  = 0.001;
const double p_alt  = 1.0;
const double p_dalt = 0.25;
const double p_a_bia  = 0.00000001;
const double p_g_bia  = 0.00000001;

//---------------------------------------------------

// controller properties
const double K_alt = 0.75;
const double K_i_alt = 0.0;
const double K_dalt = 3.0;
const double K_i_dalt = 1.0;
const double vertical_speed_limit_upper = 2.5;
const double vertical_speed_limit_lower = -2.5;
const double altitude_hold_deadband = 0.1;
const double altitude_hold_threshold = 0.5;
const double altitude_integrator_limit = 1.0;
const double thrust_factor_limit = 2.0;
const double thrust_command_limit = 20.0; // N

const double wpcl = 8.5;
const double zpcl = 0.95;
const double K_i_p = 0.15;
const double roll_command_limit = 30.0 * DEG2RAD; // rad

const double wqcl = 7.0;
const double zqcl = 0.95;
const double K_i_q = 1.2;
const double pitch_command_limit = 30.0 * DEG2RAD; // rad
const double pitch_rate_command_limit = 15.0 * DEG2RAD; // rad
const double pitch_integrator_limit = 10.0 * DEG2RAD; // rad

const double yaw_rate_deadband = 10.0 * DEG2RAD; // yaw rate input threshold to engage heading hold
const double yaw_hold_threshold =  5.0 * DEG2RAD; // yaw rate command threshold for heading hold

const double wrcl = 4.0;
const double zrcl = 0.95;
const double K_i_r = 0.15;

const double wrrcl = 7.0;
const double K_i_rr = 0.0;
const double yaw_rate_command_limit = 120.0 * DEG2RAD; // rad

const double g_pos = 0.5;
const double g_pos_i = 0.1;
const double g_vel = 0.8;
const double velocity_hold_threshold = 1.5;
const double position_hold_threshold = 0.7;
const double planar_vel_limit = 4.0;
const double position_integrator_limit = 2.0;

#endif
