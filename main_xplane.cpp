#include "Vehicle.h"
#include "Sensors.h"
#include "Utils.h"
#include "EKF.h"
#include "Controller.h"
#include "VehicleParameters.h"
#include "EnvironmentParameters.h"
#include "MathConstants.h"
#include "Eigen-3.3/Eigen/Eigen"
#include "Joystick/joystick.hh"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <time.h>
#include <string>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include "xpcExample/src/xplaneConnect.h"

using namespace Eigen;

int main(int argc, char** argv)
{
    // Create an instance of Joystick
    Joystick joystick("/dev/input/js0");
    Joystick::jinput inputs; // structure of axis inputs

    // Ensure that it was found and that we can use it
    if (!joystick.isFound())
    {
    printf("open failed.\n");
    exit(1);
    }

    //-------------------------------------------------------------------

    // Open Socket to XPlane
	const char* IP = "127.0.0.1";      //IP Address of computer running XPlane
	XPCSocket sock = openUDP(IP);
	float tVal[1];
	int tSize = 1;
	if (getDREF(sock, "sim/test/test_float", tVal, &tSize) < 0)
	{
		printf("Error establishing connecting. Unable to read data from X-Plane.");
		return EXIT_FAILURE;
	}

    //-------------------------------------------------------------------

    Vehicle Veh;
    Utils Utils;
    Sensors Sensors;

    double DT_SIM_0 = 0.001;    // simulation time step
    double DT_IMU_0 = 0.01;     // control/IMU time step
    double DT_GPS_0 = 0.1;      // GPS time step

    int n_out = 15; // number of tracked outputs (states)
    int n_cont = 6; // number of control inputs
    int n_com = 5;  // number of guidance commands

    // initialise states and controls
    VectorXd X(n_out);
    X.setZero();
    VectorXd X_COM(n_com);
    X_COM.setZero();

    VectorXd U(n_cont);
    U.setZero();
    VectorXd H(5);
    H.setZero();
    VectorXd H2 = H;
    VectorXd U_Trim(n_cont);
    U_Trim.setZero();
    VectorXd U_imu(n_cont);
    U_imu.setZero();

    // initial states
    double VelTrim = 0;
    double AltTrim = 30.0;
    double ThetaTrim = 0.0 * DEG2RAD;
    double PsiTrim   = 0.0 * DEG2RAD;
    double LonTrim   = 153.121 * DEG2RAD;
    double LatTrim   = -27.39 * DEG2RAD;

    // convert joystick inputs into
    bool tilt_mode = false;
    bool tilt_switch_high = false;

    double throttle = 1.0; // vertical velocity command (navigation frame)
    double thrust = 0.0; // forward thrust (navigation frame)
    double phiC = 0.0; // bank angle command
    double thtC = 0.0; // pitch angle command
    double thtC_tilt = 0.0; // pitch command in tilt mode
    double qC = 0.0;   // pitch rate command
    double rC = 0.0;   // yaw angle command

    if (tilt_mode)
    {
        X_COM << throttle, phiC, thtC_tilt, thrust, rC; // command vector
    }
    else
    {
        X_COM << throttle, phiC, thtC, 0.0, rC; // command vector
    }

    // determine trim states and controls
    Veh.Trim(X, U, VelTrim, AltTrim, ThetaTrim, PsiTrim, LonTrim, LatTrim);

    // initialise controller object
    Controller CTRL(DT_IMU_0);

    // calculate motor and servo commands
    H = CTRL.Actuators(U);
    H2 = CTRL.Actuators2(U);

    MatrixXd C_bn = Utils.DirectionCosineMatrix(X(6),X(7),X(8));
    VectorXd body_vel(3);
    body_vel << X(0), X(1), X(2);
    VectorXd nav_vel = C_bn.transpose() * body_vel;
    VectorXd quat_truth(4);
    quat_truth = Utils.EulerToQuaternion(X(6),X(7),X(8));

    // initial filter states
    VectorXd X_EKF(19);
    X_EKF << X(9), X(10), X(11), nav_vel(0), nav_vel(1), nav_vel(2), quat_truth(0), quat_truth(1), quat_truth(2), quat_truth(3), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // set static accelerometer and gyro biases
    Vector3d ACC_STATIC;
    ACC_STATIC << 0.0, 0.0, 0.0;
    Vector3d GYR_STATIC;
    GYR_STATIC << 0.0, 0.0, 0.0;

    // initialise EKF object
    EKF EKF(X_EKF, ACC_STATIC, GYR_STATIC);

    // estimate drag (body)
    Vector3d F_drag_body = Veh.DragModel(X);

    // initial IMU inputs
    U_imu = U;
    U_imu(0) -= F_drag_body(0);
    U_imu(1) -= F_drag_body(1);
    U_imu(2) -= F_drag_body(2);

    // initialise sensor measurements
    VectorXd imu = Sensors.IMU(X,U_imu);
    double baro_alt = Sensors.Barometer(X);
    VectorXd gps = Sensors.GPS(X);

    // initialise processed measurements
    EKF.RunEKF(imu, gps, baro_alt, DT_IMU_0);

    std::cout << "---------------- RUN SIMULATION ----------------" << std::endl;

    std::ofstream myfile;

    myfile.open ("output.csv");

    VectorXd X_PVA(12);
    X_PVA = EKF.GetPVA();

    X_EKF = EKF.GetX(); // get filter outputs

    VectorXd euler = EKF.GetEuler();

    myfile << 0 << ", "
           << X(0) << ", "
           << X(1) << ", "
           << X(2) << ", "
           << X(3) << ", "
           << X(4) << ", "
           << X(5) << ", "
           << X(6) << ", "
           << X(7) << ", "
           << X(8) << ", "         //10
           << X(9) << ", "
           << X(10) << ", "
           << X(11) << ", "
           << U(0) << ", "
           << U(1) << ", "
           << U(2) << ", "
           << U(3) << ", "
           << U(4) << ", "
           << U(5) << ", "
           << imu(0) << ", "       //20
           << imu(1) << ", "
           << imu(2) << ", "
           << imu(3) << ", "
           << imu(4) << ", "
           << imu(5) << ", "
           << imu(6) << ", "
           << imu(7) << ", "
           << imu(8) << ", "
           << gps(0) << ", "
           << gps(1) << ", "       //30
           << gps(2) << ", "
           << gps(3) << ", "
           << gps(4) << ", "
           << gps(5) << ", "
           << baro_alt << ", "
           << X_EKF(0) << ", "
           << X_EKF(1) << ", "
           << X_EKF(2) << ", "
           << X_EKF(3) << ", "
           << X_EKF(4) << ", "     //40
           << X_EKF(5) << ", "
           << X_EKF(6) << ", "
           << X_EKF(7) << ", "
           << X_EKF(8) << ", "
           << X_EKF(9) << ", "
           << X_EKF(10) << ", "
           << X_EKF(11) << ", "
           << X_EKF(12) << ", "
           << X_EKF(13) << ", "
           << X_EKF(14) << ", "    //50
           << X_EKF(15) << ", "
           << X_EKF(16) << ", "
           << X_EKF(17) << ", "
           << X_EKF(18) << ", "
           << euler(0) << ", "
           << euler(1) << ", "
           << euler(2) << ", "
           << H(0) << ", "
           << H(1) << ", "
           << H(2) << ", "         // 60
           << H(3) << ", "
           << H(4) << std::endl;

    //-------------------------------------------------------------------

	// Set Location/Orientation (sendPOSI)
	// Set Up Position Array
	float POSI[9] = {0.0};
	POSI[0] = (float) X(9);  // X
	POSI[1] = (float) X(10); // Y
	POSI[2] = (float) X(11); // Z
	POSI[3] = 0.0;          // Pitch
	POSI[4] = 0.0;          // Roll
	POSI[5] = (float) (PsiTrim * RAD2DEG); // Heading

	// Set position of the player aircraft
	sendPOSI(sock, POSI, 7, 0);

	// pauseSim
	pauseSim(sock, 1); // Sending 1 to pause
	sleep(5); // Pause for 5 seconds

	// Unpause
	pauseSim(sock, 0); // Sending 0 to unpause
	printf("- Resuming Simulation\n");

    const char* dref = "sim/operation/override/override_planepath"; // flight model overide data reference
	float result[8] = {1,0,0,0,0,0,0,0};
	sendDREF(sock, dref, result, 1); // send data to xplane

    //-------------------------------------------------------------------

    // clock variable for simulation execution
    clock_t t;
    int f;
    t = clock(); // current system time (micro seconds)
    clock_t t_sim_prev = t;
    clock_t t_imu_prev = t;
    clock_t t_gps_prev = t;

    double DT_SIM, DT_IMU, DT_GPS;
    int i_log = 0; // log counter

    while (true)
    {
        t = clock(); // current system time

        DT_SIM = (float)(t - t_sim_prev)/CLOCKS_PER_SEC; // elapsed simulation time (seconds)
        DT_IMU = (float)(t - t_imu_prev)/CLOCKS_PER_SEC; // elapsed control/imu time (seconds)
        DT_GPS = (float)(t - t_gps_prev)/CLOCKS_PER_SEC; // elapsed gps time (seconds)

        // Attempt to sample an event from the joystick
        JoystickEvent event;
        if (joystick.sample(&event))
        {
            joystick.GetInputs(inputs, event);
        }

        if(DT_SIM >= DT_SIM_0) // main simulation loop (truth)
        {
            t_sim_prev = t; // update time

            Veh.Integrate(X,U,DT_SIM_0); // update vehicle states

            //-------------------------------------------------------------------

            // populate position vector of current true states for xplane
            POSI[0] = (float) (X(9));   // X
            POSI[1] = (float) (X(10));  // Y
            POSI[2] = (float) (X(11));  // Z
            POSI[3] = (float) (X(7) * RAD2DEG);     // Pitch
            POSI[4] = (float) (X(6) * RAD2DEG);     // Roll
            POSI[5] = (float) (X(8) * RAD2DEG);     // Heading/Yaw

            // Set position of aircraft to xplane
            sendPOSI(sock, POSI, 7, 0);

            //-------------------------------------------------------------------

            if (DT_IMU >= DT_IMU_0) // control loop
            {
                t_imu_prev = t;

                // ------------------------ GUIDANCE -----------------------------

                // tilt mode switch
                if (inputs.ax7 == 1 && !tilt_switch_high)
                {
                    tilt_mode = !tilt_mode;
                    tilt_switch_high = true;
                }
                else if (inputs.ax7 == 0)
                {
                    tilt_switch_high = false;
                }

                if (tilt_mode)
                {
                    // convert joystick inputs into commands
                    throttle = 1.0 + inputs.ax3; // vertical velocity command (navigation frame)
                    phiC = roll_command_limit *  inputs.ax0; // bank angle command
                    thrust = -thrust_command_limit * inputs.ax1; // forward thrust command
                    qC = pitch_rate_command_limit * inputs.ax5;   // pitch rate command
                    rC = yaw_rate_command_limit * inputs.ax2;   // yaw rate command

                    // commanded tilt angle
                    thtC_tilt += qC * DT_IMU_0;

                    // tilt angle limiter
                    if (thtC_tilt >= pitch_command_limit)
                    {
                        thtC_tilt = pitch_command_limit;
                    }
                    else if (thtC_tilt <= -pitch_command_limit)
                    {
                        thtC_tilt = -pitch_command_limit;
                    }

                    X_COM << throttle, phiC, thtC_tilt, thrust, rC; // command vector (tilt mode)
                }
                else
                {
                    // convert joystick inputs into commands
                    throttle = 1.0 + inputs.ax3; // vertical velocity command (navigation frame)
                    phiC = roll_command_limit * inputs.ax0; // bank angle command
                    thtC = pitch_command_limit * inputs.ax1; // pitch angle command
                    rC = yaw_rate_command_limit * inputs.ax2;   // yaw rate command
                    thtC_tilt = 0.0;

                    X_COM << throttle, phiC, thtC, 0.0, rC; // command vector (normal mode)
                }

                // ----------------------- NAVIGATION ----------------------------

                // estimate drag (body axis)
                F_drag_body = Veh.DragModel(X);

                // IMU inputs
                U_imu = U;
                U_imu(0) -= F_drag_body(0);
                U_imu(1) -= F_drag_body(1);
                U_imu(2) -= F_drag_body(2);

                imu = Sensors.IMU(X,U_imu); // update IMU measurements
                baro_alt = Sensors.Barometer(X); // update barometer measurements

                if (DT_GPS >= DT_GPS_0) // update gps measurements at GPS rate
                {
                    t_gps_prev = t;

                    gps = Sensors.GPS(X);
                }

                // estimate vehicle states
                EKF.RunEKF(imu, gps, baro_alt, DT_IMU_0);

                // get PVA data
                X_PVA = EKF.GetPVA();

                // ------------------------- CONTROL -----------------------------

                // control input vector (body force input vector)
                U = CTRL.RunController(X_COM, X_PVA, tilt_mode);

                // calculate motor and servo commands
                H = CTRL.Actuators(U);
                H2 = CTRL.Actuators2(U);

                // ------------------------- LOGGING -----------------------------

                X_EKF = EKF.GetX(); // get EKF state vector

                euler = EKF.GetEuler(); // get estimated euler angles

                quat_truth = Utils.EulerToQuaternion(X(6),X(7),X(8)); // true quaternion vector

                // true local velocity
                Matrix3d C_bn = Utils.DirectionCosineMatrix(X(6), X(7), X(8)); // inertial to body transformation
                Vector3d VelBody;
                VelBody << X(0), X(1), X(2); // body velocity vector
                Vector3d VelNav = C_bn.transpose() * VelBody; // NED velocity vector

                // send to output file for logging at control rate
                myfile << (i_log + 1)*DT_IMU_0 << ", "
                        << X(0) << ", "
                        << X(1) << ", "
                        << X(2) << ", "
                        << X(3) << ", "
                        << X(4) << ", "
                        << X(5) << ", "
                        << X(6) << ", "
                        << X(7) << ", "
                        << X(8) << ", "         //10
                        << X(9) << ", "
                        << X(10) << ", "
                        << X(11) << ", "
                        << U(0) << ", "
                        << U(1) << ", "
                        << U(2) << ", "
                        << U(3) << ", "
                        << U(4) << ", "
                        << U(5) <<  ", "
                        << imu(0) << ", "       //20
                        << imu(1) << ", "
                        << imu(2) << ", "
                        << imu(3) << ", "
                        << imu(4) << ", "
                        << imu(5) << ", "
                        << imu(6) << ", "
                        << imu(7) << ", "
                        << imu(8) << ", "
                        << gps(0) << ", "
                        << gps(1) << ", "       //30
                        << gps(2) << ", "
                        << gps(3) << ", "
                        << gps(4) << ", "
                        << gps(5) << ", "
                        << baro_alt << ", "
                        << X_EKF(0) << ", "
                        << X_EKF(1) << ", "
                        << X_EKF(2) << ", "
                        << X_EKF(3) << ", "
                        << X_EKF(4) << ", "     //40
                        << X_EKF(5) << ", "
                        << X_EKF(6) << ", "
                        << X_EKF(7) << ", "
                        << X_EKF(8) << ", "
                        << X_EKF(9) << ", "
                        << X_EKF(10) << ", "
                        << X_EKF(11) << ", "
                        << X_EKF(12) << ", "
                        << X_EKF(13) << ", "
                        << X_EKF(14) << ", "    //50
                        << X_EKF(15) << ", "
                        << X_EKF(16) << ", "
                        << X_EKF(17) << ", "
                        << X_EKF(18) << ", "
                        << euler(0) << ", "
                        << euler(1) << ", "
                        << euler(2) << ", "
                        << H(0) << ", "
                        << H(1) << ", "
                        << H(2) << ", "         // 60
                        << H(3) << ", "
                        << H(4) << std::endl;

                std::cout << std::setprecision(3)
                        << std::fixed
                        << "EKF: "
                        << X_EKF(0) << ", "
                        << X_EKF(1) << ", "
                        << X_EKF(2) << " | "
                        << X_EKF(3) << ", "
                        << X_EKF(4) << ", "
                        << X_EKF(5) << " | "
                        << std::setprecision(3)
                        << X_EKF(6) << ", "
                        << X_EKF(7) << ", "
                        << X_EKF(8) << ", "
                        << X_EKF(9) << "| "
                        << euler(0) * RAD2DEG << ", "
                        << euler(1) * RAD2DEG << ", "
                        << euler(2) * RAD2DEG << " | "
                        << H(0) << ", " << H2(0) << ", "
                        << H(1) << ", " << H2(1) << ", "
                        << H(2) << ", " << H2(2) << ", "
                        << H(3) * RAD2DEG << ", " << H2(3) * RAD2DEG << ", "
                        << H(4) * RAD2DEG << ", " << H2(4) * RAD2DEG << std::endl;

//                 std::cout << std::setprecision(3)
//                         << std::fixed
//                         << "TRU: "
//                         << X(9) << ", "
//                         << X(10) << ", "
//                         << X(11) << " | "
//                         << VelNav(0) << ", "
//                         << VelNav(1) << ", "
//                         << VelNav(2) << " | "
//                         << std::setprecision(3)
//                         << quat_truth(0) << ", "
//                         << quat_truth(1) << ", "
//                         << quat_truth(2) << ", "
//                         << quat_truth(3) << "| "
//                         << X(6) * RAD2DEG << ", "
//                         << X(7) * RAD2DEG << ", "
//                         << X(8) * RAD2DEG << std::endl;

                i_log++; // increment log counter
            }

        }

    }

//     myfile.close();

    std::cout << "---------------- SIMULATION END ----------------" << std::endl;

    return 1;
}

