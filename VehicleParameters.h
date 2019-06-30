#include <cmath>

#ifndef vehicle_parameters__H
#define vehicle_parameters__H

// math constants
const double PI = acos(-1.0);
const double DEG2RAD = acos(-1.0)/180.0;
const double RAD2DEG = 1.0/DEG2RAD;

// environment
const double GRAVITY = 9.81;
const double R_EA = 6378137.0; // earth semi-major axis
const double FLATTENING = 1.0/298.257223563; // flattening factor
const double R_EB = R_EA * (1.0 - FLATTENING); // earth semi-minor axis
const double ECC = sqrt(R_EA*R_EA - R_EB*R_EB)/R_EA; // eccentricity

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
const double Ixz = 0.001;

// ekf parameters
const double q_pos = 0.0;
const double q_vel = 0.002;
const double q_att = 0.00001;
const double q_bia = 0.000001;
const double q_alt = 0.0;
const double q_dalt = 0.002;

const double r_pos = 1.0;
const double r_vel = 1.0;
const double r_att = 0.0001;
const double r_alt = 1.0;
const double r_dalt = 5.0;

const double p_pos = 1.0;
const double p_vel = 1.0;
const double p_att = 0.0001;
const double p_bia = 0.00001;

#endif
