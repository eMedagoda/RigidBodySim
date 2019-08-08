#include "Eigen-3.3/Eigen/Eigen"

using namespace Eigen;

typedef enum altholdstate {
    ALT_HOLD_DISABLE,
    ALT_HOLD_VELOCITY,
    ALT_HOLD_ALTITUDE
} alt_hold_state_t;

typedef enum yawholdstate {
    YAW_HOLD_DISABLE,
    YAW_HOLD_YAW_RATE,
    YAW_HOLD_YAW
} yaw_hold_state_t;

typedef enum posholdstate {
    POS_HOLD_DISABLE,
    POS_HOLD_VELOCITY,
    POS_HOLD_POSITION
} pos_hold_state_t;

class Controller
{
    public:

        Controller(double DT);
        ~Controller();

        VectorXd RunController(VectorXd X_COM, VectorXd X_PVA, bool tilt_mode);
        VectorXd Actuators(VectorXd U);
        VectorXd Actuators2(VectorXd U);

    private:

        VectorXd FlyAttitudeAltitude(VectorXd X_COM, bool tilt_mode);

        VectorXd FlyPosition(VectorXd X_COM, bool tilt_mode);

        double AltitudeHold(double altitude_command);
        double VerticalSpeedControl(double vertical_speed_command);
        double RollControl(double roll_command);
        double PitchControl(double pitch_command);
        double YawControl(double yaw_command);
        double YawRateControl(double yaw_rate_command);
        Vector3d VelocityHold(double Vx_com, double Vy_com);
        Vector3d PositionHold(double X_com, double Y_com);

        VectorXd ControlMap(VectorXd H);
        VectorXd ControlMap2(VectorXd H);

        void VerticalSpeedLimiter(double &vertical_speed_command);
        void PiMinusPi(double& input);
        double IntegrateScalar(double dydx_new, double dydx, double y);

        double m_DT;
        VectorXd m_H;

        double m_roll;
        double m_pitch;
        double m_yaw;

        double m_roll_rate;
        double m_pitch_rate;
        double m_yaw_rate;

        double m_altitude;
        double m_vertical_speed;

        double m_pos_x;
        double m_pos_y;
        double m_vel_x;
        double m_vel_y;
        double m_vel_horizontal;
};
