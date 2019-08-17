#include "Controller.h"
#include "Utils.h"
#include "VehicleParameters.h"
#include "EnvironmentParameters.h"
#include "MathConstants.h"
#include <cmath>
#include <iostream>
#include <iomanip>

Controller::Controller(double DT)
:m_roll(0.0),
m_pitch(0.0),
m_yaw(0.0),
m_roll_rate(0.0),
m_pitch_rate(0.0),
m_yaw_rate(0.0),
m_altitude(0.0),
m_vertical_speed(0.0),
m_pos_x(0.0),
m_pos_y(0.0),
m_vel_x(0.0),
m_vel_y(0.0),
m_vel_horizontal(0.0)
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

VectorXd Controller::RunController(VectorXd X_COM, VectorXd X_PVA, bool tilt_mode, int ctrl_mode)
{
    VectorXd dU(6);
    dU.setZero();

    // attitude control states
    m_roll = X_PVA(6);
    m_pitch = X_PVA(7);
    m_yaw = X_PVA(8); // current vehicle yaw
    m_roll_rate = X_PVA(9);
    m_pitch_rate = X_PVA(10);
    m_yaw_rate = X_PVA(11); // current vehicle yaw rate

    // altitude control states
    m_altitude = -X_PVA(2); // current vehicle altitude (positive up)
    m_vertical_speed = -X_PVA(5); // current vehicle vertical speed (positive up)

    // position control states
    m_pos_x = X_PVA(0);
    m_pos_y = X_PVA(1);
    m_vel_x = X_PVA(3);
    m_vel_y = X_PVA(4);
    m_vel_horizontal = sqrt((m_vel_x * m_vel_x) + (m_vel_y * m_vel_y));

    // throttle mode selected
    if (ctrl_mode == 0)
    {
        dU = FlyAttitudeThrottle(X_COM);
    }

    // altitude hold mode selected
    if (ctrl_mode ==  1)
    {
        dU = FlyAttitudeAltitude(X_COM, tilt_mode);
    }

    // position hold mode selected
    if (ctrl_mode == 2)
    {
        dU = FlyPosition(X_COM, tilt_mode);
    }

    // calculate motor and servo commands
    m_H = Actuators(dU);

    return dU;
}

VectorXd Controller::FlyAttitudeThrottle(VectorXd X_COM)
{
    Utils Utils;

    VectorXd U(6);
    U.setZero();

    // initialise desired commands
    double desired_vertical_force = 0.0;
    double desired_roll_moment = 0.0;
    double desired_pitch_moment = 0.0;
    double desired_yaw_moment = 0.0;

    // guidance commands
    double throttle_command = X_COM(0); // throttle command (lift, positive up)
    double roll_command = X_COM(1);     // roll command
    double pitch_command = X_COM(2);    // pitch/tilt command
    double yaw_rate_command = X_COM(4); // yaw rate command (body axis)

    // --------------------------- Altitude Control ---------------------------

    desired_vertical_force = throttle_command * MASS * GRAVITY;

    // --------------------------- Attitude Control ---------------------------

    desired_roll_moment = RollControl(roll_command);
    desired_pitch_moment = PitchControl(pitch_command);

    // --------------------------- Heading Control ----------------------------

    static yaw_hold_state_t yaw_hold_state = YAW_HOLD_DISABLE;
    static double yaw_command = m_yaw;

    if ((fabs(yaw_rate_command) < yaw_rate_deadband)) // if yaw stick is centred
    {
        if (yaw_hold_state == YAW_HOLD_DISABLE) // if not in a yaw holding state
        {
            yaw_rate_command = 0.0; // set commanded yaw rate to zero
            yaw_command = m_yaw; // update current yaw command

            yaw_hold_state = YAW_HOLD_YAW_RATE; // transition to velocity hold state
        }

        if (yaw_hold_state == YAW_HOLD_YAW_RATE) // if in velocity hold state
        {
            yaw_command = m_yaw; // update current altitude command

            if (fabs(m_yaw_rate) < yaw_hold_threshold) // if yaw rate drops below threshold
            {
                yaw_hold_state = YAW_HOLD_YAW; // transition to yaw hold state
            }
            else
            {
                // run yaw rate controller, output Mz
                desired_yaw_moment = YawRateControl(yaw_rate_command);
            }
        }

        if (yaw_hold_state == YAW_HOLD_YAW) // if in yaw hold state
        {
            // run yaw hold controller (only when yaw stick has been re-centred)
            desired_yaw_moment = YawControl(yaw_command);
        }
    }
    else // if yaw rate commands have been issued (yaw rate command)
    {
        yaw_hold_state = YAW_HOLD_DISABLE; // transition to manoeuvre state

        // run yaw rate controller, output Mz
        desired_yaw_moment = YawRateControl(yaw_rate_command);

        // update current yaw command
        yaw_command = m_yaw;
    }

    // ------------------------------------------------------------------------

    // command forces in navigation frame (NED)
    Vector3d F_desired_local;
    F_desired_local(0) = 0.0;
    F_desired_local(1) = 0.0;
    F_desired_local(2) = -desired_vertical_force;

    // navigation to body
    MatrixXd C_bn = Utils.DirectionCosineMatrix(m_roll, m_pitch, m_yaw);

    // command forces in body frame
    Vector3d F_desired_body = C_bn * F_desired_local;

    // if flying in normal mode
    U(0) = 0.0;
    U(1) = 0.0;
    U(2) = F_desired_body(2);
    U(3) = desired_roll_moment;
    U(4) = desired_pitch_moment;
    U(5) = desired_yaw_moment;

    return U;
}

VectorXd Controller::FlyAttitudeAltitude(VectorXd X_COM, bool tilt_mode)
{
    Utils Utils;

    VectorXd U(6);
    U.setZero();

    // initialise desired commands
    double desired_vertical_force = 0.0;
    double desired_roll_moment = 0.0;
    double desired_pitch_moment = 0.0;
    double desired_yaw_moment = 0.0;

    // guidance commands
    double throttle_command = X_COM(0); // throttle command (lift, positive up)
    double roll_command = X_COM(1);     // roll command
    double pitch_command = X_COM(2);    // pitch/tilt command
    double yaw_rate_command = X_COM(4); // yaw rate command (body axis)

    // --------------------------- Altitude Control ---------------------------

    static alt_hold_state_t alt_hold_state = ALT_HOLD_DISABLE;
    static double altitude_command = m_altitude;
    static double vertical_speed_command = m_vertical_speed;

    if ((fabs(1.0 - throttle_command) < altitude_hold_deadband)) // if throttle stick is centred
    {
        if (alt_hold_state == ALT_HOLD_DISABLE) // if not in an altitude holding state
        {
            vertical_speed_command = 0.0; // set commanded speed to zero
            altitude_command = m_altitude; // update current altitude command

            alt_hold_state = ALT_HOLD_VELOCITY; // transition to velocity hold state
        }

        if (alt_hold_state == ALT_HOLD_VELOCITY) // if in velocity hold state
        {
            altitude_command = m_altitude; // update current altitude command

            if (fabs(m_vertical_speed) < altitude_hold_threshold) // if vertical velocity drops below threshold
            {
                alt_hold_state = ALT_HOLD_ALTITUDE; // transition to altitude hold state
            }
            else
            {
                // run vertical speed controller, output Fz
                desired_vertical_force = VerticalSpeedControl(vertical_speed_command);
            }
        }

        if (alt_hold_state == ALT_HOLD_ALTITUDE) // if in altitude hold state
        {
            // run altitude hold controller (only when throttle has been re-centred)
            desired_vertical_force = AltitudeHold(altitude_command);
        }
    }
    else // if throttle commands have been issued (vertical speed command)
    {
        alt_hold_state = ALT_HOLD_DISABLE; // transition to manoeuvre state

        // vertical speed command (linear interp between 0 - 2 thrust factor, between upper and lower vertical speed limits
        vertical_speed_command = ((vertical_speed_limit_upper - vertical_speed_limit_lower)/thrust_factor_limit) * throttle_command + vertical_speed_limit_lower;

        // apply vertical speed limits
        VerticalSpeedLimiter(vertical_speed_command);

        // run vertical speed controller, output Fz
        desired_vertical_force = VerticalSpeedControl(vertical_speed_command);

        // update current altitude command
        altitude_command = m_altitude;
    }

    // --------------------------- Attitude Control ---------------------------

    desired_roll_moment = RollControl(roll_command);
    desired_pitch_moment = PitchControl(pitch_command);

    // --------------------------- Heading Control ----------------------------

    static yaw_hold_state_t yaw_hold_state = YAW_HOLD_DISABLE;
    static double yaw_command = m_yaw;

    if ((fabs(yaw_rate_command) < yaw_rate_deadband)) // if yaw stick is centred
    {
        if (yaw_hold_state == YAW_HOLD_DISABLE) // if not in a yaw holding state
        {
            yaw_rate_command = 0.0; // set commanded yaw rate to zero
            yaw_command = m_yaw; // update current yaw command

            yaw_hold_state = YAW_HOLD_YAW_RATE; // transition to velocity hold state
        }

        if (yaw_hold_state == YAW_HOLD_YAW_RATE) // if in velocity hold state
        {
            yaw_command = m_yaw; // update current altitude command

            if (fabs(m_yaw_rate) < yaw_hold_threshold) // if yaw rate drops below threshold
            {
                yaw_hold_state = YAW_HOLD_YAW; // transition to yaw hold state
            }
            else
            {
                // run yaw rate controller, output Mz
                desired_yaw_moment = YawRateControl(yaw_rate_command);
            }
        }

        if (yaw_hold_state == YAW_HOLD_YAW) // if in yaw hold state
        {
            // run yaw hold controller (only when yaw stick has been re-centred)
            desired_yaw_moment = YawControl(yaw_command);
        }
    }
    else // if yaw rate commands have been issued (yaw rate command)
    {
        yaw_hold_state = YAW_HOLD_DISABLE; // transition to manoeuvre state

        // run yaw rate controller, output Mz
        desired_yaw_moment = YawRateControl(yaw_rate_command);

        // update current yaw command
        yaw_command = m_yaw;
    }

    // ------------------------------------------------------------------------

    // command forces in navigation frame (NED)
    Vector3d F_desired_local;
    F_desired_local(0) = X_COM(3);
    F_desired_local(1) = 0.0;
    F_desired_local(2) = -desired_vertical_force;

    // navigation to body
    MatrixXd C_bn = Utils.DirectionCosineMatrix(m_roll, m_pitch, 0.0);

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

 VectorXd Controller::FlyPosition(VectorXd X_COM, bool tilt_mode)
{
    Utils Utils;

    VectorXd U(6);
    U.setZero();

    // initialise desired commands
    double desired_vertical_force = 0.0;
    double desired_roll_moment = 0.0;
    double desired_pitch_moment = 0.0;
    double desired_yaw_moment = 0.0;

    // guidance commands
    double throttle_command = X_COM(0); // throttle command (lift, positive up)
    double roll_command = X_COM(1);     // roll command
    double pitch_command = X_COM(2);    // pitch/tilt command
    double thrust_command = X_COM(3);   // forward thrust command
    double yaw_rate_command = X_COM(4); // yaw rate command (body axis)

    // --------------------------- Altitude Control ---------------------------

    static alt_hold_state_t alt_hold_state = ALT_HOLD_DISABLE;
    static double altitude_command = m_altitude;
    static double vertical_speed_command = m_vertical_speed;

    if ((fabs(1.0 - throttle_command) < altitude_hold_deadband)) // if throttle stick is centred
    {
        if (alt_hold_state == ALT_HOLD_DISABLE) // if not in an altitude holding state
        {
            vertical_speed_command = 0.0; // set commanded speed to zero
            altitude_command = m_altitude; // update current altitude command

            alt_hold_state = ALT_HOLD_VELOCITY; // transition to velocity hold state
        }

        if (alt_hold_state == ALT_HOLD_VELOCITY) // if in velocity hold state
        {
            altitude_command = m_altitude; // update current altitude command

            if (fabs(m_vertical_speed) < altitude_hold_threshold) // if vertical velocity drops below threshold
            {
                alt_hold_state = ALT_HOLD_ALTITUDE; // transition to altitude hold state
            }
            else
            {
                // run vertical speed controller, output Fz
                desired_vertical_force = VerticalSpeedControl(vertical_speed_command);
            }
        }

        if (alt_hold_state == ALT_HOLD_ALTITUDE) // if in altitude hold state
        {
            // run altitude hold controller (only when throttle has been re-centred)
            desired_vertical_force = AltitudeHold(altitude_command);
        }
    }
    else // if throttle commands have been issued (vertical speed command)
    {
        alt_hold_state = ALT_HOLD_DISABLE; // transition to manoeuvre state

        // vertical speed command (linear interp between 0 - 2 thrust factor, between upper and lower vertical speed limits
        vertical_speed_command = ((vertical_speed_limit_upper - vertical_speed_limit_lower)/thrust_factor_limit) * throttle_command + vertical_speed_limit_lower;

        // apply vertical speed limits
        VerticalSpeedLimiter(vertical_speed_command);

        // run vertical speed controller, output Fz
        desired_vertical_force = VerticalSpeedControl(vertical_speed_command);

        // update current altitude command
        altitude_command = m_altitude;
    }

    // --------------------------- Position Control ---------------------------

    static pos_hold_state_t pos_hold_state = POS_HOLD_DISABLE;
    static double Vx_com = m_vel_x;
    static double Vy_com = m_vel_y;
    static double X_com = m_pos_x;
    static double Y_com = m_pos_y;

    Vector3d attitude_commands;
    attitude_commands.setZero();

    bool vehicle_stationary = false;

    if ((fabs(roll_command) < 2.0 * DEG2RAD) && (m_vel_horizontal < velocity_hold_threshold))
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
            X_com = m_pos_x; //set target waypoint to current GPS position when the vehicle has sufficently slowed
            Y_com = m_pos_y;

            pos_hold_state = POS_HOLD_VELOCITY; // transition to velocity hold state
        }

        if (pos_hold_state == POS_HOLD_VELOCITY) // if in velocity hold state
        {
            X_com = m_pos_x; //set target waypoint to current GPS position when the vehicle has sufficently slowed
            Y_com = m_pos_y;

            if (m_vel_horizontal < position_hold_threshold) // if horizontal velocity drops below threshold
            {
                pos_hold_state = POS_HOLD_POSITION; // transition to position hold state
            }
            else
            {
                attitude_commands = VelocityHold(Vx_com, Vy_com); // command vehicle speed (outputs roll command and pitch command)
            }

        }

        if (pos_hold_state == POS_HOLD_POSITION) // if in position hold state
        {
            attitude_commands = PositionHold(X_com, Y_com); // run position control (outputs roll command and pitch command)
        }
    }
    else // if stick commands have been issued (manoeuvre command - direct roll and pitch angle commands)
    {
        pos_hold_state = POS_HOLD_DISABLE; // transition to manoeuvre state
        Vx_com = m_vel_x; // set commanded speed to current speed
        Vy_com = m_vel_y;
        X_com = m_pos_x; //set target waypoint to current GPS position when the vehicle has sufficently slowed
        Y_com = m_pos_y;

        attitude_commands(0) = roll_command;
        attitude_commands(1) = pitch_command;
        attitude_commands(2) = thrust_command;
    }

    if (tilt_mode)
    {
        desired_roll_moment = RollControl(attitude_commands(0));
        desired_pitch_moment = PitchControl(pitch_command);
    }
    else
    {
        desired_roll_moment = RollControl(attitude_commands(0));
        desired_pitch_moment = PitchControl(attitude_commands(1));
    }

    // --------------------------- Heading Control ----------------------------

    static yaw_hold_state_t yaw_hold_state = YAW_HOLD_DISABLE;
    static double yaw_command = m_yaw;

    if ((fabs(yaw_rate_command) < yaw_rate_deadband)) // if yaw stick is centred
    {
        if (yaw_hold_state == YAW_HOLD_DISABLE) // if not in a yaw holding state
        {
            yaw_rate_command = 0.0; // set commanded yaw rate to zero
            yaw_command = m_yaw; // update current yaw command

            yaw_hold_state = YAW_HOLD_YAW_RATE; // transition to velocity hold state
        }

        if (yaw_hold_state == YAW_HOLD_YAW_RATE) // if in velocity hold state
        {
            yaw_command = m_yaw; // update current altitude command

            if (fabs(m_yaw_rate) < yaw_hold_threshold) // if yaw rate drops below threshold
            {
                yaw_hold_state = YAW_HOLD_YAW; // transition to yaw hold state
            }
            else
            {
                // run yaw rate controller, output Mz
                desired_yaw_moment = YawRateControl(yaw_rate_command);
            }
        }

        if (yaw_hold_state == YAW_HOLD_YAW) // if in yaw hold state
        {
            // run yaw hold controller (only when yaw stick has been re-centred)
            desired_yaw_moment = YawControl(yaw_command);
        }
    }
    else // if yaw rate commands have been issued (yaw rate command)
    {
        yaw_hold_state = YAW_HOLD_DISABLE; // transition to manoeuvre state

        // run yaw rate controller, output Mz
        desired_yaw_moment = YawRateControl(yaw_rate_command);

        // update current yaw command
        yaw_command = m_yaw;
    }

    // ------------------------------------------------------------------------

    // command forces in navigation frame (NED)
    Vector3d F_desired_local;
    F_desired_local(0) = attitude_commands(2);
    F_desired_local(1) = 0.0;
    F_desired_local(2) = -desired_vertical_force;

    // navigation to body
    MatrixXd C_bn = Utils.DirectionCosineMatrix(m_roll, m_pitch, 0.0);

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

 double Controller::AltitudeHold(double altitude_command)
{
    // initialise integrator states
    static double ff = 0.0;
    static double ffd = 0.0;

    // compute altitude error
    double altitude_error = altitude_command - m_altitude;

    // vertical speed integrator loop
    if (fabs(altitude_error) < altitude_integrator_limit) // if vertical speed error is within 0.5 m/s, enable integrator
    {
        ff = IntegrateScalar(altitude_error, ffd, ff);
        ffd = altitude_error;
    }
    else // else, clear integrator states
    {
        ff = 0.0;
        ffd = 0.0;
    }

    //altitude rate command
    double vertical_speed_command = K_alt * altitude_error + K_i_alt * ff; // altitude loop, output is a vertical speed command (altitude positive up)

    // apply vertical speed limits
    VerticalSpeedLimiter(vertical_speed_command);

    // run vertical speed controllers
    double Fz = VerticalSpeedControl(vertical_speed_command);

    return Fz;
}

double Controller::VerticalSpeedControl(double vertical_speed_command)
{
    // initialise integrator states
    static double ee = 0.0;
    static double eed = 0.0;

    // compute vertical speed error
    double vertical_speed_error = vertical_speed_command - m_vertical_speed;

    // vertical speed integrator loop
    if (fabs(vertical_speed_error) < altitude_hold_threshold) // if vertical speed error is within 0.5 m/s, enable integrator
    {
        ee = IntegrateScalar(vertical_speed_error, eed, ee);
        eed = vertical_speed_error;
    }
    else // else, clear integrator states
    {
        ee = 0.0;
        eed = 0.0;
    }

    // // compute acceleration command, output is an acceleration command (positive up), add integral action, when hovering a_comm = AGRAV
    double a_comm = K_dalt * vertical_speed_error + K_i_dalt * ee + GRAVITY;

    // thrust command (vertical, should equal vehicle weight when hovering - this in navigation frame)
    double Fz = MASS * a_comm;

    return Fz;
}

double Controller::RollControl(double roll_command)
{
    // initialise integrator states
    static double aad = 0.0;
    static double aa = 0.0;

    // calculate controller gains
    double K_phi = wpcl * wpcl * Ixx; // proportional gain on roll error
    double K_p = 2.0 * zpcl * wpcl * Ixx; // derivative gain on roll rate error

    // roll angle error
    double ephi = roll_command - m_roll;

    // integrator
    aa = IntegrateScalar(ephi, aad, aa);
    aad = ephi;

    // desired roll moment (positive roll right, negative roll left)
    double Mx = (K_phi * ephi) + (K_p * (0.0 - m_roll_rate)) + (K_i_p * aa);

    return Mx;
}

double Controller::PitchControl(double pitch_command)
{
    // initialise integrator states
    static double bb = 0.0;
    static double bbd = 0.0;

    // calculate controller gains
    double K_tht = wqcl * wqcl * Iyy; // proportional gain on pitch error
    double K_q = 2.0 * zqcl * wqcl * Iyy; // derivative gain on pitch rate error

    // pitch angle error
    double etht = pitch_command - m_pitch;

    // pitch angle integrator loop
    if (fabs(etht) < pitch_integrator_limit) // if pitch angle error is within limit, enable integrator
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
    double My = (K_tht * etht) + (K_q * (0.0 - m_pitch_rate)) + (K_i_q * bb);

    return My;
}

double Controller::YawControl(double yaw_command)
{
    // initialise integrator states
    static double cc = 0.0;
    static double ccd = 0.0;

    // calculate controller gains
    double K_psi = wrcl * wrcl * Izz; // proportional gain on yaw angle error
    double K_r = 2.0 * zrcl * wrcl * Izz; // derivative gain on yaw rate error

    PiMinusPi(yaw_command);
    PiMinusPi(m_yaw);

    // yaw angle error
    double epsi = yaw_command - m_yaw;

    PiMinusPi(epsi);

    // integrator
    cc = IntegrateScalar(epsi, ccd, cc);
    ccd = epsi;

    // desired yaw moment (positive yaw right)
    double Mz = (K_psi * epsi) + (K_r * (0.0 - m_yaw_rate)) + (K_i_r * cc);

    return Mz;
}

double Controller::YawRateControl(double yaw_rate_command)
{
    // initialise integrator states
    static double zz = 0.0;
    static double zzd = 0.0;

    // calculate controller gains
    double K_r = wrrcl * Izz;

    // yaw rate error
    double e_rr = yaw_rate_command - m_yaw_rate;

    // integrator
    zz = IntegrateScalar(e_rr, zzd, zz);
    zzd = e_rr;

    // desired yaw moment (positive yaw right)
    double Mz = K_r * e_rr + K_i_rr * zz;

    return Mz;
}

Vector3d Controller::VelocityHold(double Vx_com, double Vy_com)
{
    Vector3d attitude_commands;
    attitude_commands.setZero();

    // total vertical force (body axis, magnitude)
    double force_sum = (m_H(0) * cos(-m_H(4)) + m_H(1) * cos(m_H(4)) + m_H(2));

    // align velocity to current heading
    double vel_x_body = cos(m_yaw) * m_vel_x + sin(m_yaw) * m_vel_y; // heading aligned LVLH forward velocity
    double vel_y_body = cos(m_yaw) * m_vel_y - sin(m_yaw) * m_vel_x; // heading aligned LVLH lateral velocity

    // forward acceleration command
    double Ax_com = g_vel * (Vx_com - vel_x_body);

    // lateral acceleration command
    double Ay_com = g_vel * (Vy_com - vel_y_body);

    // roll angle command and limiter
    double temp_roll_rat = (MASS * Ay_com) / force_sum;

    // roll angle command (positive Ay_com -> positive roll)
    double roll_command = asin(temp_roll_rat);

    // roll command limiter
    if (roll_command > roll_command_limit)
    {
        roll_command = roll_command_limit;
    }

    if (roll_command < (-roll_command_limit))
    {
        roll_command = -roll_command_limit;
    }

    // pitch angle command and limiter
    double temp_pitch_rat = (MASS * -Ax_com) / (force_sum * cos(m_roll));

    // pitch angle command (positive Ax_com -> negative pitch)
    double pitch_command = asin(temp_pitch_rat);

    // pitch command limiter
    if (pitch_command > pitch_command_limit)
    {
        pitch_command = pitch_command_limit;
    }

    if (pitch_command < (-pitch_command_limit))
    {
        pitch_command = -pitch_command_limit;
    }

    attitude_commands(0) = RollControl(roll_command);
    attitude_commands(1) = PitchControl(pitch_command);
    attitude_commands(2) = Ax_com * MASS;

    return attitude_commands;
}

Vector3d Controller::PositionHold(double X_com, double Y_com)
{
    Vector3d attitude_commands;
    attitude_commands.setZero();

    //align fb variables to current heading
    double X_com_body = cos(m_yaw) * X_com + sin(m_yaw) * Y_com;
    double Y_com_body = cos(m_yaw) * Y_com - sin(m_yaw) * X_com;
    double pos_x_body = cos(m_yaw) * m_pos_x + sin(m_yaw) * m_pos_y;
    double pos_y_body = cos(m_yaw) * m_pos_y - sin(m_yaw) * m_pos_x;

    // initialise integrator states
    static double xx = 0.0;
    static double xxd = 0.0;

    // x-position error
    double e_pos_x = X_com_body - pos_x_body;

    if (fabs(e_pos_x) < position_integrator_limit)
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
    double Vx_com = g_pos * e_pos_x + g_pos_i * xx;

    if (Vx_com > planar_vel_limit)
    {
        Vx_com = planar_vel_limit;
    }
    if (Vx_com < (-planar_vel_limit))
    {
        Vx_com = -planar_vel_limit;
    }

    // initialise integrator states
    static double yy = 0.0;
    static double yyd = 0.0;

    // y-position error
    double e_pos_y = Y_com_body - pos_y_body;

    if (fabs(e_pos_y) < position_integrator_limit)
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
    double Vy_com = g_pos * e_pos_y + g_pos_i * yy;

    if (Vy_com > planar_vel_limit)
    {
        Vy_com = planar_vel_limit;
    }
    if (Vy_com < (-planar_vel_limit))
    {
        Vy_com = -planar_vel_limit;
    }

    attitude_commands = VelocityHold(Vx_com, Vy_com);

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
    XTrim << 20.0, 20.0, 10.0, 0.01, -0.05; // F_R, F_L, F_T, mu, d_mu (TODO: possibly cache this from last estimate)

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
//     U(3) = H(0)*L1*cos(H(3) + H(4)) + H(1)*L3*cos(H(3) - H(4)) - H(2)*L5;
    U(3) = H(0)*L1*cos(H(3) + H(4)) + H(0)*L6*sin(H(3) + H(4)) + H(1)*L3*cos(H(3) - H(4)) + H(1)*L7*sin(H(3) - H(4)) - H(2)*L5;

    // Mz
    U(4) = -H(0)*L2*sin(H(3) + H(4)) + H(1)*L4*sin(H(3) - H(4));

    return U;
}

VectorXd Controller::Actuators2(VectorXd U_total)
{
    MatrixXd J(5,5);
    J.setZero();

    VectorXd U(5);
    U << U_total(0), U_total(2), U_total(3), U_total(4), U_total(5); // Fx, Fz, Mx, My, Mz

    // set reference input
    VectorXd U0 = U;

    // initial trim vector
    VectorXd XTrim(5);
    XTrim << 20.0, 20.0, 10.0, 0.01, -0.05; // F_R, F_L, F_T, mu, d_mu (TODO: possibly cache this from last estimate)

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
        U = ControlMap2(XTrim);

        // difference between reference and estimated input
        VectorXd dU = U0 - U;

        for (int i = 0; i < 5; i++)
        {
            // perturb inputs
            VectorXd XTrim_Pert = XTrim;

            // add perturbation to current estimate
            XTrim_Pert(i) = XTrim(i) + dXTrim;

            // calculate perturbed input estimates
            VectorXd U_Pert = ControlMap2(XTrim_Pert);

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

VectorXd Controller::ControlMap2(VectorXd H)
{
    VectorXd U(5);
    U.setZero();

    // Fx
    U(0) = H(0)*sin(H(3) + H(4)) + H(1)*sin(H(3) - H(4));

    // Fz
    U(1) = -H(0)*cos(H(3) + H(4)) - H(1)*cos(H(3) - H(4)) - H(2);

    // Mx
    U(2) = -H(0)*sf2b2*cos(H(3) + H(4)) - H(1)*sf1b2*cos(H(3) - H(4));

    // My
    U(3) = H(0)*sf2b1*cos(H(3) + H(4)) + H(0)*sf2b3*sin(H(3) + H(4)) + H(1)*sf1b1*cos(H(3) - H(4)) + H(1)*sf1b3*sin(H(3) - H(4)) + H(2)*sf3b1;

    // Mz
    U(4) = -H(0)*sf2b2*sin(H(3) + H(4)) - H(1)*sf1b2*sin(H(3) - H(4));

    return U;
}
