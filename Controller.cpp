#include "Controller.h"
#include "Utils.h"
#include "VehicleParameters.h"
#include "EnvironmentParameters.h"
#include "MathConstants.h"
#include <cmath>
#include <iostream>

Controller::Controller(double DT)
{
    m_DT = DT;
}

Controller::~Controller()
{
}

VectorXd Controller::RunController(VectorXd X_COM, VectorXd X_PVA, bool tilt_mode)
{
    VectorXd dU = FlyAttitudeAltitude(X_COM, X_PVA, tilt_mode);

    return dU;
}

VectorXd Controller::FlyAttitudeAltitude(VectorXd X_COM, VectorXd X_PVA, bool tilt_mode)
{
    Utils Utils;

    VectorXd U(6);
    U.setZero();

    // initialise desired commands
    double desired_vertical_force = 0.0;
    double desired_roll_moment = 0.0;
    double desired_pitch_moment = 0.0;
    double desired_yaw_moment = 0.0;

    // true states
    double roll = X_PVA(6);
    double roll_rate = X_PVA(9);
    double pitch = X_PVA(7);
    double pitch_rate = X_PVA(10);
    double yaw = X_PVA(8); // current vehicle yaw
    double yaw_rate = X_PVA(11); // current vehicle yaw rate
    double altitude = -X_PVA(2); // current vehicle altitude (positive up)
    double vertical_speed = -X_PVA(5); // current vehicle vertical speed (positive up)

    // guidance commands
    double throttle_command = X_COM(0); // throttle command (lift, positive up)
    double roll_command = X_COM(1);     // roll command
    double pitch_command = X_COM(2);    // pitch/tilt command
    double yaw_rate_command = X_COM(4); // yaw rate command (body axis)

    double m_thrust_factor = throttle_command;

    // --------------------------- Altitude Control ---------------------------

    static alt_hold_state_t alt_hold_state = ALT_HOLD_DISABLE;
    static double altitude_command = altitude;
    static double vertical_speed_command = vertical_speed;

    if ((fabs(1.0 - m_thrust_factor) < AltitudeControlDeadband)) // if throttle stick is centred
    {
        if (alt_hold_state == ALT_HOLD_DISABLE) // if not in an altitude holding state
        {
            vertical_speed_command = 0.0; // set commanded speed to zero
            altitude_command = altitude; // update current altitude command

            alt_hold_state = ALT_HOLD_VELOCITY; // transition to velocity hold state
        }

        if (alt_hold_state == ALT_HOLD_VELOCITY) // if in velocity hold state
        {
            altitude_command = altitude; // update current altitude command

            if (fabs(vertical_speed) < AltitudeHoldVerticalSpeedThreshold) // if vertical velocity drops below threshold
            {
                alt_hold_state = ALT_HOLD_ALTITUDE; // transition to altitude hold state
            }
            else
            {
                // run vertical speed controller, output Fz
                desired_vertical_force = VerticalSpeedControl(vertical_speed_command, vertical_speed);
            }
        }

        if (alt_hold_state == ALT_HOLD_ALTITUDE) // if in altitude hold state
        {
            // run altitude hold controller (only when throttle has been re-centred)
            desired_vertical_force = AltitudeHold(altitude_command, altitude, vertical_speed);
        }
    }
    else // if throttle commands have been issued (vertical speed command)
    {
        alt_hold_state = ALT_HOLD_DISABLE; // transition to manoeuvre state

        // vertical speed command (linear interp between 0 - 2 thrust factor, between upper and lower vertical speed limits
        vertical_speed_command = ((vertical_speed_limit_upper - vertical_speed_limit_lower)/thrust_factor_limit) * m_thrust_factor + vertical_speed_limit_lower;

        // apply vertical speed limits
        VerticalSpeedLimiter(vertical_speed_command);

        // run vertical speed controller, output Fz
        desired_vertical_force = VerticalSpeedControl(vertical_speed_command, vertical_speed);

        // update current altitude command
        altitude_command = altitude;
    }

    // --------------------------- Attitude Control ---------------------------

    desired_roll_moment = RollControl(roll_command, roll, roll_rate);
    desired_pitch_moment = PitchControl(pitch_command, pitch, pitch_rate);

    // --------------------------- Heading Control ----------------------------

    static yaw_hold_state_t yaw_hold_state = YAW_HOLD_DISABLE;
    static double yaw_command = yaw;

    if ((fabs(yaw_rate_command) < YawRateControlDeadband)) // if yaw stick is centred
    {
        if (yaw_hold_state == YAW_HOLD_DISABLE) // if not in a yaw holding state
        {
            yaw_rate_command = 0.0; // set commanded yaw rate to zero
            yaw_command = yaw; // update current yaw command

            yaw_hold_state = YAW_HOLD_YAW_RATE; // transition to velocity hold state
        }

        if (yaw_hold_state == YAW_HOLD_YAW_RATE) // if in velocity hold state
        {
            yaw_command = yaw; // update current altitude command

            if (fabs(yaw_rate) < YawHoldYawRateThreshold) // if yaw rate drops below threshold
            {
                yaw_hold_state = YAW_HOLD_YAW; // transition to yaw hold state
            }
            else
            {
                // run yaw rate controller, output Mz
                desired_yaw_moment = YawRateControl(yaw_rate_command, yaw_rate);
            }
        }

        if (yaw_hold_state == YAW_HOLD_YAW) // if in yaw hold state
        {
            // run yaw hold controller (only when yaw stick has been re-centred)
            desired_yaw_moment = YawControl(yaw_command, yaw, yaw_rate);
        }
    }
    else // if yaw rate commands have been issued (yaw rate command)
    {
        yaw_hold_state = YAW_HOLD_DISABLE; // transition to manoeuvre state

        // run yaw rate controller, output Mz
        desired_yaw_moment = YawRateControl(yaw_rate_command, yaw_rate);

        // update current yaw command
        yaw_command = yaw;
    }

    // ------------------------------------------------------------------------

    // command forces in navigation frame (NED)
    Vector3d F_desired_local;
    F_desired_local(0) = X_COM(3);
    F_desired_local(1) = 0.0;
    F_desired_local(2) = -desired_vertical_force;

    // navigation to body
    MatrixXd C_bn = Utils.DirectionCosineMatrix(roll, pitch, yaw);

    // command forces in body frame
    Vector3d F_desired_body = C_bn * F_desired_local;

    if (tilt_mode)
    {
        // if flying in tilt mode
        U(0) = F_desired_body(0); // body x force applied to move forward in tilt mode
        U(1) = 0.0;
        U(2) = F_desired_body(2);
        U(3) = desired_roll_moment;
        U(4) = desired_pitch_moment;
        U(5) = desired_yaw_moment;
    }
    else
    {
        // if flying in normal mode
        U(0) = 0.0;
        U(1) = 0.0;
        U(2) = F_desired_body(2);
        U(3) = desired_roll_moment;
        U(4) = desired_pitch_moment;
        U(5) = desired_yaw_moment;
    }

    return U;
 }

 double Controller::AltitudeHold(double altitude_command, double altitude, double vertical_speed)
{
    double vertical_speed_command;

    //altitude rate command
    vertical_speed_command = K_alt * (altitude_command - altitude); // altitude loop, output is a vertical speed command (altitude positive up)

    // apply vertical speed limits
    VerticalSpeedLimiter(vertical_speed_command);

    // run vertical speed controller
    double Fz = VerticalSpeedControl(vertical_speed_command, vertical_speed);

    return Fz;
}

double Controller::VerticalSpeedControl(double vertical_speed_command, double vertical_speed)
{
    double a_comm;
    double vertical_speed_error;

    // initialise integrator states
    static double ee = 0.0;
    static double eed = 0.0;

    // compute vertical speed error
    vertical_speed_error = vertical_speed_command - vertical_speed;

    // compute acceleration command, output is an acceleration command (positive up)
    a_comm = K_dalt * vertical_speed_error;

    // vertical speed integrator loop
    if (fabs(vertical_speed_error) < AltitudeHoldVerticalSpeedThreshold) // if vertical speed error is within 0.5 m/s, enable integrator
    {
        ee = IntegrateScalar(vertical_speed_error, eed, ee);
        eed = vertical_speed_error;
    }
    else // else, clear integrator states
    {
        ee = 0.0;
        eed = 0.0;
    }

    // add integral action, when hovering a_comm = AGRAV
    a_comm += K_i_dalt * ee + GRAVITY;

    // thrust command (vertical, should equal vehicle weight when hovering - this in navigation frame)
    double Fz = MASS * a_comm;

    return Fz;
}

double Controller::RollControl(double roll_command, double roll_angle, double roll_rate)
{
    // initialise integrator states
    static double aad = 0.0;
    static double aa = 0.0;

    // calculate controller gains
    double K_phi = wpcl * wpcl * Ixx; // proportional gain on roll error
    double K_p = 2.0 * zpcl * wpcl * Ixx; // derivative gain on roll rate error

    // roll angle error
    double ephi = roll_command - roll_angle;

    // integrator
    aa = IntegrateScalar(ephi, aad, aa);
    aad = ephi;

    // desired roll moment (positive roll right, negative roll left)
    double Mx = (K_phi * ephi) + (K_p * (0.0 - roll_rate)) + (K_i_p * aa);

    return Mx;
}

double Controller::PitchControl(double pitch_command, double pitch_angle, double pitch_rate)
{
    // initialise integrator states
    static double bb = 0.0;
    static double bbd = 0.0;

    // calculate controller gains
    double K_tht = wqcl * wqcl * Iyy; // proportional gain on pitch error
    double K_q = 2.0 * zqcl * wqcl * Iyy; // derivative gain on pitch rate error

    // pitch angle error
    double etht = pitch_command - pitch_angle;

    // pitch angle integrator loop
    if (fabs(etht) < (10.0 * DEG2RAD)) // if pitch angle error is within 10.0 deg, enable integrator
    {
        bb = IntegrateScalar(etht, bbd, bb);
        bbd = etht;
    }
    else // else, clear integrator states
    {
        bb = 0.0;
        bbd = 0.0;
    }

    // desired pitch moment (positive to pitch up, negative to pitch down)
    double My = (K_tht * etht) + (K_q * (0.0 - pitch_rate)) + (K_i_q * bb);

    return My;
}

double Controller::YawControl(double yaw_command, double yaw_angle, double yaw_rate)
{
    // initialise integrator states
    static double cc = 0.0;
    static double ccd = 0.0;

    // calculate controller gains
    double K_psi = wrcl * wrcl * Izz; // proportional gain on yaw angle error
    double K_r = 2.0 * zrcl * wrcl * Izz; // derivative gain on yaw rate error

    PiMinusPi(yaw_command);
    PiMinusPi(yaw_angle);

    // yaw angle error
    double epsi = yaw_command - yaw_angle;

    PiMinusPi(epsi);

    // integrator
    cc = IntegrateScalar(epsi, ccd, cc);
    ccd = epsi;

    // desired yaw moment (positive yaw right)
    double Mz = (K_psi * epsi) + (K_r * (0.0 - yaw_rate)) + (K_i_r * cc);

    return Mz;
}

double Controller::YawRateControl(double yaw_rate_command, double yaw_rate)
{
    // initialise integrator states
    static double zz = 0.0;
    static double zzd = 0.0;

    // calculate controller gains
    double K_r = wrrcl * Izz;

    // yaw rate error
    double e_rr = yaw_rate_command - yaw_rate;

    // integrator
    zz = IntegrateScalar(e_rr, zzd, zz);
    zzd = e_rr;

    // desired yaw moment (positive yaw right)
    double Mz = K_r * e_rr + K_i_rr * zz;

    return Mz;
}

void Controller::VerticalSpeedLimiter(double &vertical_speed_command)
{
    // vertical speed limiter
    if (vertical_speed_command > vertical_speed_limit_upper)
    {
        vertical_speed_command = vertical_speed_limit_upper;
    }

    if (vertical_speed_command < vertical_speed_limit_lower)
    {
        vertical_speed_command = vertical_speed_limit_lower;
    }
}

void Controller::PiMinusPi(double& input)
{
    if (input <= -PI)
    {
        input += 2.0 * PI;
    }
    else if (input >= PI)
    {
        input -= 2.0 * PI;
    }

}

double Controller::IntegrateScalar(double dydx_new, double dydx, double y)
{
    return y + (0.5 * (dydx_new + dydx) * m_DT);
}
