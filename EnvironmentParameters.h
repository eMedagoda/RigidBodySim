#ifndef ENVIRONMENT_PARAMETERS__H
#define ENVIRONMENT_PARAMETERS__H

#include <cmath>

// environment
const double GRAVITY = 9.81;
const double R_EA = 6378137.0; // earth semi-major axis
const double FLATTENING = 1.0/298.257223563; // flattening factor
const double R_EB = R_EA * (1.0 - FLATTENING); // earth semi-minor axis
const double ECC = sqrt(R_EA*R_EA - R_EB*R_EB)/R_EA; // eccentricity

#endif
