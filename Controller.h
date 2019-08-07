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

    private:

        VectorXd FlyAttitudeAltitude(VectorXd X_COM, VectorXd X_PVA, bool tilt_mode);

        VectorXd FlyPosition(VectorXd X_COM, VectorXd X_PVA, bool tilt_mode);

        double AltitudeHold(double altitude_command, double altitude, double vertical_speed);
        double VerticalSpeedControl(double vertical_speed_command, double vertical_speed);
        double RollControl(double roll_command, double roll_angle, double roll_rate);
        double PitchControl(double pitch_command, double pitch_angle, double pitch_rate);
        double YawControl(double yaw_command, double yaw_angle, double yaw_rate);
        double YawRateControl(double yaw_rate_command, double yaw_rate);
        Vector3d VelocityHold(double Vx_com, double Vy_com, double vel_x, double vel_y, double roll, double pitch, double yaw, double roll_rate, double pitch_rate);
        Vector3d PositionHold(double X_com, double Y_com, double pos_x, double pos_y, double vel_x, double vel_y, double roll, double pitch, double yaw, double roll_rate, double pitch_rate);

        VectorXd ControlMap(VectorXd H);

        void VerticalSpeedLimiter(double &vertical_speed_command);
        void PiMinusPi(double& input);
        double IntegrateScalar(double dydx_new, double dydx, double y);

        double m_DT;
        VectorXd m_H;

};
