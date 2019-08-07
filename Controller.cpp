#include "Controller.h"
#include "Utils.h"
#include "VehicleParameters.h"
#include "EnvironmentParameters.h"
#include "MathConstants.h"
#include <cmath>
#include <iostream>
#include <iomanip>

Controller::Controller(double DT)
{
    m_DT = DT;

    m_H = VectorXd(5);
    m_H(0) = 19.0;
    m_H(1) = 19.0;
    m_H(2) = 6.5;
    m_H(3) = 0.0;
    m_H(4) = 0.0;
}

Controller::~Controller()
{
}

VectorXd Controller::RunController(VectorXd X_COM, VectorXd X_PVA, bool tilt_mode)
{
    VectorXd dU(6);
    dU.setZero();

//     dU = FlyAttitudeAltitude(X_COM, X_PVA, tilt_mode);

    dU = FlyPosition(X_COM, X_PVA, tilt_mode);

    // calculate motor and servo commands
    m_H = Actuators(dU);

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

 VectorXd Controller::FlyPosition(VectorXd X_COM, VectorXd X_PVA, bool tilt_mode)
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

    double vel_x = X_PVA(3);
    double vel_y = X_PVA(4);
    double pos_x = X_PVA(0);
    double pos_y = X_PVA(1);
    double horiz_vel = sqrt((vel_x * vel_x) + (vel_y * vel_y));

    // guidance commands
    double throttle_command = X_COM(0); // throttle command (lift, positive up)
    double roll_command = X_COM(1);     // roll command
    double pitch_command = X_COM(2);    // pitch/tilt command
    double thrust_command = X_COM(3);   // forward thrust command
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

    // --------------------------- Position Control ---------------------------

    static pos_hold_state_t pos_hold_state = POS_HOLD_DISABLE;
    static double Vx_com = vel_x;
    static double Vy_com = vel_y;
    static double X_com = pos_x;
    static double Y_com = pos_y;

    Vector3d attitude_commands;
    attitude_commands.setZero();

    bool vehicle_stationary = false;

    if ((fabs(roll_command) < 2.0 * DEG2RAD) && (horiz_vel < 2.2))
    {
        if (tilt_mode)
        {
            if (fabs(thrust_command) < 2.0)
            {
                vehicle_stationary = true;
            }
        }
        else
        {
             if (fabs(pitch_command) < 2.0 * DEG2RAD)
             {
                 vehicle_stationary = true;
             }
        }
    }

    if (vehicle_stationary) // if manoeuvre stick is centred, and vehicle has sufficently slowed
    {
        if (pos_hold_state == POS_HOLD_DISABLE) // if not in a holding state
        {
            Vx_com = 0.0; // set commanded speed to zero
            Vy_com = 0.0;
            X_com = pos_x; //set target waypoint to current GPS position when the vehicle has sufficently slowed
            Y_com = pos_y;

            pos_hold_state = POS_HOLD_VELOCITY; // transition to velocity hold state
        }

        if (pos_hold_state == POS_HOLD_VELOCITY) // if in velocity hold state
        {
            X_com = pos_x; //set target waypoint to current GPS position when the vehicle has sufficently slowed
            Y_com = pos_y;

            if (horiz_vel < 0.7) // if horizontal velocity drops below threshold
            {
                pos_hold_state = POS_HOLD_POSITION; // transition to position hold state
            }
            else
            {
                attitude_commands = VelocityHold(Vx_com, Vy_com, vel_x, vel_y, roll, pitch, yaw, roll_rate, pitch_rate); // command vehicle speed (outputs roll command and pitch command)
            }

        }

        if (pos_hold_state == POS_HOLD_POSITION) // if in position hold state
        {
            attitude_commands = PositionHold(X_com, Y_com, pos_x, pos_y, vel_x, vel_y, roll, pitch, yaw, roll_rate, pitch_rate); // run position control (outputs roll command and pitch command)
        }
    }
    else // if stick commands have been issued (manoeuvre command - direct roll and pitch angle commands)
    {
        pos_hold_state = POS_HOLD_DISABLE; // transition to manoeuvre state
        Vx_com = vel_x; // set commanded speed to current speed
        Vy_com = vel_y;
        X_com = pos_x; //set target waypoint to current GPS position when the vehicle has sufficently slowed
        Y_com = pos_y;

        attitude_commands(0) = roll_command;
        attitude_commands(1) = pitch_command;
        attitude_commands(2) = thrust_command;
    }

    if (tilt_mode)
    {
        desired_roll_moment = RollControl(attitude_commands(0), roll, roll_rate);
        desired_pitch_moment = PitchControl(pitch_command, pitch, pitch_rate);
    }
    else
    {
        desired_roll_moment = RollControl(attitude_commands(0), roll, roll_rate);
        desired_pitch_moment = PitchControl(attitude_commands(1), pitch, pitch_rate);
    }

//     std::cout << std::setprecision(3)
//             << std::fixed
//             << pos_hold_state << ", "
//             << tilt_mode << " | "
//             << X_com << ", "
//             << pos_x << " | "
//             << Y_com << ", "
//             << pos_y << " | "
//             << vel_x << ", "
//             << vel_y << " | "
//             << attitude_commands(0) * RAD2DEG << ", "
//             << roll * RAD2DEG << " | "
//             << attitude_commands(1) * RAD2DEG << ", "
//             << pitch * RAD2DEG << " | "
//             << attitude_commands(2) << std::endl;

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
    F_desired_local(0) = attitude_commands(2);
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

Vector3d Controller::VelocityHold(double Vx_com, double Vy_com, double vel_x, double vel_y, double roll, double pitch, double yaw, double roll_rate, double pitch_rate)
{
    Vector3d attitude_commands;
    attitude_commands.setZero();

    double force_sum = (m_H(0) * cos(m_H(4)) + m_H(1) * cos(-m_H(4)) + m_H(2)); // total vertical force (body axis, magnitude)

    //align fb variables to current heading
    double vel_x_body = cos(yaw) * vel_x + sin(yaw) * vel_y; // heading aligned LVLH forward velocity
    double vel_y_body = cos(yaw) * vel_y - sin(yaw) * vel_x; // heading aligned LVLH lateral velocity

    //acceleration command in forward direction
    double acom1 = g_vel * (Vx_com - vel_x_body);

    //acceleration command in lateral direction
    double acom2 = g_vel * (Vy_com - vel_y_body);

    //roll angle command and limiter
    double temp_roll_rat = (MASS * acom2) / force_sum;

    // roll angle command (positive acom2 -> positive roll)
    double phicom = asin(temp_roll_rat);

    // roll command limiter
    if (phicom > roll_command_limit)
    {
        phicom = roll_command_limit;
    }

    if (phicom < (-roll_command_limit))
    {
        phicom = -roll_command_limit;
    }

    // pitch angle command and limiter
    double temp_pitch_rat = (MASS * -acom1) / (force_sum * cos(roll));

    // pitch angle command (positive acom1 -> negative pitch)
    double thtcom = asin(temp_pitch_rat);

    // pitch command limiter
    if (thtcom > pitch_command_limit)
    {
        thtcom = pitch_command_limit;
    }

    if (thtcom < (-pitch_command_limit))
    {
        thtcom = -pitch_command_limit;
    }

    attitude_commands(0) = RollControl(phicom, roll, roll_rate);
    attitude_commands(1) = PitchControl(thtcom, pitch, pitch_rate);
    attitude_commands(2) = acom1 * MASS;

    return attitude_commands;
}

Vector3d Controller::PositionHold(double X_com, double Y_com, double pos_x, double pos_y, double vel_x, double vel_y, double roll, double pitch, double yaw, double roll_rate, double pitch_rate)
{
    Vector3d attitude_commands;
    attitude_commands.setZero();

    double planar_vel_lim = 4.0;

    //align fb variables to current heading
    double X_com_body = cos(yaw) * X_com + sin(yaw) * Y_com;
    double Y_com_body = cos(yaw) * Y_com - sin(yaw) * X_com;
    double pos_x_body = cos(yaw) * pos_x + sin(yaw) * pos_y;
    double pos_y_body = cos(yaw) * pos_y - sin(yaw) * pos_x;

    // initialise integrator states
    static double xx = 0.0;
    static double xxd = 0.0;

    // x-position error
    double e_pos_x = X_com_body - pos_x_body;

    if (fabs(e_pos_x) < 3.0)
    {
        // integrator
        xx = IntegrateScalar(e_pos_x, xxd, xx);
        xxd = e_pos_x;
    }
    else
    {
        xx = 0.0;
        xxd = 0.0;
    }

    //velocity command in lateral direction and limiter
    double vcom1 = g_pos * e_pos_x + g_pos_i * xx;

    if (vcom1 > planar_vel_lim)
    {
        vcom1 = planar_vel_lim;
    }
    if (vcom1 < (-planar_vel_lim))
    {
        vcom1 = -planar_vel_lim;
    }

    // initialise integrator states
    static double yy = 0.0;
    static double yyd = 0.0;

    // y-position error
    double e_pos_y = Y_com_body - pos_y_body;

    if (fabs(e_pos_y) < 3.0)
    {
        // integrator
        yy = IntegrateScalar(e_pos_y, yyd, yy);
        yyd = e_pos_y;
    }
    else
    {
        yy = 0.0;
        yyd = 0.0;
    }

    //velocity command in lateral direction and limiter
    double vcom2 = g_pos * e_pos_y + g_pos_i * yy;

    if (vcom2 > planar_vel_lim)
    {
        vcom2 = planar_vel_lim;
    }
    if (vcom2 < (-planar_vel_lim))
    {
        vcom2 = -planar_vel_lim;
    }

    attitude_commands = VelocityHold(vcom1, vcom2, vel_x, vel_y, roll, pitch, yaw, roll_rate, pitch_rate);

    return attitude_commands;
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

VectorXd Controller::Actuators(VectorXd U_total)
{
    MatrixXd J(5,5);
    J.setZero();

    VectorXd U(5);
    U << U_total(0), U_total(2), U_total(3), U_total(4), U_total(5); // Fx, Fz, Mx, My, Mz

    // set reference input
    VectorXd U0 = U;

    // initial trim vector
    VectorXd XTrim(5);
    XTrim << 20.0, 20.0, 10.0, 0.01, -0.05; // F_L, F_R, F_T, mu, d_mu (TODO: possibly cache this from last estimate)

    // convergence tolerance
    double Tol = 1e-8;

    // perturbation
    double dXTrim = 1e-6;

    // intial error flag
    double Err = 1.0;

    // initial counter
    int n = 0;

    VectorXd XTrim_new = XTrim;

    while (Err > Tol) // while solution hasn't converged
    {
        // estimate current acutator values
        U = ControlMap(XTrim);

        // difference between reference and estimated input
        VectorXd dU = U0 - U;

        for (int i = 0; i < 5; i++)
        {
            // perturb inputs
            VectorXd XTrim_Pert = XTrim;

            // add perturbation to current estimate
            XTrim_Pert(i) = XTrim(i) + dXTrim;

            // calculate perturbed input estimates
            VectorXd U_Pert = ControlMap(XTrim_Pert);

            // difference between reference and perturbed input estimate
            VectorXd dU_Pert = U0 - U_Pert;

            // update Jacobian
            J.block<5,1>(0,i) = (dU_Pert - dU)/dXTrim;
        }

        // update state (gradient decent)
        XTrim_new = XTrim - J.inverse() * dU;

        // check for convergence
        Err = sqrt((XTrim_new(0) - XTrim(0))*(XTrim_new(0) - XTrim(0)) + (XTrim_new(1) - XTrim(1))*(XTrim_new(1) - XTrim(1)) + (XTrim_new(2) - XTrim(2))*(XTrim_new(2) - XTrim(2)) + (XTrim_new(3) - XTrim(3))*(XTrim_new(3) - XTrim(3)) + (XTrim_new(4) - XTrim(4))*(XTrim_new(4) - XTrim(4)));

        // update current state for next loop
        XTrim = XTrim_new;

        // increment counter
        n++;
    }

    return XTrim;
}

VectorXd Controller::ControlMap(VectorXd H)
{
    VectorXd U(5);
    U.setZero();

    // Fx
    U(0) = H(0)*sin(H(3) + H(4)) + H(1)*sin(H(3) - H(4));

    // Fz
    U(1) = -H(0)*cos(H(3) + H(4)) - H(1)*cos(H(3) - H(4)) - H(2);

    // Mx
    U(2) = -H(0)*L2*cos(H(3) + H(4)) + H(1)*L4*cos(H(3) - H(4));

    // My
    U(3) = H(0)*L1*cos(H(3) + H(4)) + H(1)*L3*cos(H(3) - H(4)) - H(2)*L5;

    // Mz
    U(4) = -H(0)*L2*sin(H(3) + H(4)) + H(1)*L4*sin(H(3) - H(4));

    return U;
}
