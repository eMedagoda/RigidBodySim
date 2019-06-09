#include "Vehicle.h"
#include "PID.h"
#include "Sensors.h"
#include "Utils.h"
#include "VehicleParameters.h"
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
    
    Vehicle Veh;
    Utils Utils;
    Sensors Sensors;

    double DT_SIM_0 = 0.0001;    // simulation time step
    double DT_IMU_0 = 0.01;     // control/IMU time step
    double DT_GPS_0 = 0.1;      // GPS time step
    
    int n_out = 15;   // number of tracked outputs (states)
    int n_cont = 6;   // number of control inputs
    
    // initialise states and controls
    VectorXd X(n_out);
    X.setZero();
    VectorXd U(n_cont);
    U.setZero();
    VectorXd U_Trim(n_cont);
    U_Trim.setZero();
        
    // initial states
    double VelTrim = 0;
    double AltTrim = 30.0;
    double ThetaTrim = 0.0 * DEG2RAD;
    double PsiTrim   = 0.0 * DEG2RAD;
    double LonTrim   = 153.121 * DEG2RAD;
    double LatTrim   = -27.39 * DEG2RAD;    
    
    // initial guidance states
    double VfC = VelTrim; // forward speed command
    double VvC = 0.0; // vertical speed command
    double phiC = 0.0; // bank angle command
    double thetaC = ThetaTrim; // pitch angle command
    double dpsiC = 0.0; // yaw rate command
    double qC = 0.0; // pitch rate command
    double u_sum = 0.0; // forward speed (body axis) integral sum
    double w_sum = 0.0; // vertical speed (body axis) integral sum
    
    // determine trim states and controls
    Veh.Trim(X, U, VelTrim, AltTrim, ThetaTrim, PsiTrim, LonTrim, LatTrim);
    
    // initial control vector
    U_Trim = U;
    
    // initialise sensor measurements
    VectorXd imu = Sensors.IMU(X,U);
    double baro_alt = Sensors.Barometer(X);
    VectorXd gps = Sensors.GPS(X);
  
    std::cout << "---------------- RUN SIMULATION ----------------" << std::endl;
  
    std::ofstream myfile;
    
    myfile.open ("output.csv");
    
    myfile << 0 << ", " 
           << X(0) << ", " // true states
           << X(1) << ", " 
           << X(2) << ", " 
           << X(3) << ", "  
           << X(4) << ", " 
           << X(5) << ", " 
           << X(6) << ", " 
           << X(7) << ", " 
           << X(8) << ", "  
           << X(9) << ", " 
           << X(10) << ", "  
           << X(11) << ", "
           << X(12) << ", " 
           << X(13) << ", " 
           << X(14) << ", " 
           << U(0) << ", " // control inputs
           << U(1) << ", " 
           << U(2) << ", " 
           << U(3) << ", " 
           << U(4) << ", " 
           << U(5) << ", "
           << imu(0) << ", " // sensor measurements
           << imu(1) << ", " 
           << imu(2) << ", " 
           << imu(3) << ", "  
           << imu(4) << ", " 
           << imu(5) << ", " 
           << imu(6) << ", " 
           << imu(7) << ", " 
           << imu(8) << ", "
           << gps(0) << ", " 
           << gps(1) << ", "  
           << gps(2) << ", "
           << gps(3) << ", " 
           << gps(4) << ", "  
           << gps(5) << ", "
           << baro_alt << std::endl;
           
    // ---------------
	
	// Set Location/Orientation (sendPOSI)
	// Set Up Position Array
	float POSI[9] = {0.0};
	POSI[0] = (float) LatTrim;     // Lat
	POSI[1] = (float) LonTrim;       // Lon
	POSI[2] = (float) AltTrim;       // Alt
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
    
    //------------------
    
           
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
            
            // populate position vector of current true states for xplane
            POSI[0] = (float) (X(13) * RAD2DEG);    // Lat
            POSI[1] = (float) (X(12) * RAD2DEG);    // Lon
            POSI[2] = (float) (X(14));              // Alt
            POSI[3] = (float) (X(7) * RAD2DEG);     // Pitch
            POSI[4] = (float) (X(6) * RAD2DEG);     // Roll
            POSI[5] = (float) (X(8) * RAD2DEG);     // Heading/Yaw     
            
            // Set position of aircraft to xplane
            sendPOSI(sock, POSI, 7, 0);
            
            if (DT_IMU >= DT_IMU_0) // control loop
            {
                t_imu_prev = t;
                
                // ----------------------- NAVIGATION ----------------------------                
                               
                // sensor updates
                imu = Sensors.IMU(X,U); // update IMU measurements 
                baro_alt = Sensors.Barometer(X); // update barometer measurements
                
                if (DT_GPS >= DT_GPS_0) // update gps measurements at GPS rate
                {
                    t_gps_prev = t;
                    
                    gps = Sensors.GPS(X);
                }
                
                // ------------------------ GUIDANCE -----------------------------
            
                // convert joystick inputs into
                VfC = -5.0 * inputs.ax1; // forward velocity command (navigation frame)
                VvC = -2.5 * inputs.ax3; // vertical velocity command (navigation frame)
                
                phiC = 15.0 * inputs.ax0 * DEG2RAD;     // bank angle command
                dpsiC = 30.0 * inputs.ax2 * DEG2RAD;    // pitch angle command
                qC = 15.0 * inputs.ax5 * DEG2RAD;       // pitch rate command
                                          
                VectorXd dU(n_cont);
                dU.setZero();            
               
                MatrixXd C_bn_LVLH = Utils.DirectionCosineMatrix(X(6),X(7),0.0);
                VectorXd VnC(3);
                VnC << VfC, 0.0, VvC;
                VectorXd VbC = C_bn_LVLH * VnC; // body frame speed commands
                
                // ------------------------- CONTROL -----------------------------
                                
                double u_error = VbC(0) - X(0);
                dU(0) = 30.0 * u_error + 10.0 * u_sum * DT_IMU_0;
                
                if (fabs(u_error) < 5.0)
                {
                    u_sum += u_error;
                }
                else
                {
                    u_sum = 0.0;
                }            
                
                double w_error = VbC(2) - X(2);
                dU(2) = 30.0 * w_error + 10.0 * w_sum * DT_IMU_0;
                
                if (fabs(w_error) < 5.0)
                {
                    w_sum += w_error;
                }
                else
                {
                    w_sum = 0.0;
                }
                           
                PID phi_cont(1.5, 0.0, 0.0);
                PID p_cont(0.3, 0.0, 0.0);
                PID v_cont(0.0, 0.0, 0.0);        
                double p_C = phi_cont.Control(phiC, X(6), DT_IMU_0);
                double p_out = p_cont.Control(p_C, X(3), DT_IMU_0);
                double v_out = v_cont.Control(VbC(1), X(1), DT_IMU_0);        
                dU(3) = p_out + v_out;   
                
                PID q_cont(0.5, 0.0, 0.0);        
                double q_out = q_cont.Control(qC, X(4), DT_IMU_0);
                dU(4) = q_out;    
                
                PID dpsi_cont(0.2, 0.0, 0.0);        
                double dpsi_out = dpsi_cont.Control(dpsiC, X(5), DT_IMU_0);
                dU(5) = dpsi_out;
                
                U = U_Trim + dU;
                
                // ---------------------------------------------------------------
                
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
                        << X(8) << ", "     //10  
                        << X(9) << ", " 
                        << X(10) << ", "  
                        << X(11) << ", " 
                        << U(0) << ", " 
                        << U(1) << ", " 
                        << U(2) << ", " 
                        << U(3) << ", " 
                        << U(4) << ", " 
                        << U(5) <<  ", "
                        << imu(0) << ", "   //20
                        << imu(1) << ", " 
                        << imu(2) << ", " 
                        << imu(3) << ", "  
                        << imu(4) << ", " 
                        << imu(5) << ", " 
                        << imu(6) << ", " 
                        << imu(7) << ", " 
                        << imu(8) << ", "
                        << gps(0) << ", " 
                        << gps(1) << ", "   //30
                        << gps(2) << ", "
                        << gps(3) << ", " 
                        << gps(4) << ", "  
                        << gps(5) << ", "
                        << baro_alt << std::endl;
                        
                std::cout << std::setprecision(2) 
                        << std::fixed
                        << (i_log + 1)*DT_IMU_0 << ", " 
                        << X(0) << ", " 
                        << X(1) << ", " 
                        << X(2) << ", " 
                        << X(3) * RAD2DEG << ", "  
                        << X(4) * RAD2DEG << ", " 
                        << X(5) * RAD2DEG << ", " 
                        << X(6) * RAD2DEG << ", " 
                        << X(7) * RAD2DEG << ", " 
                        << X(8) * RAD2DEG << ", "  
                        << X(9) << ", " 
                        << X(10) << ", "  
                        << X(11) << ", " 
                        << "| "
                        << U(0) << ", " 
                        << U(1) << ", " 
                        << U(2) << ", " 
                        << U(3) << ", " 
                        << U(4) << ", " 
                        << U(5) <<  std::endl;
                
                i_log++; // increment log counter            
            }      
            
        }
        
    }                    
   
//     myfile.close();
    
    std::cout << "---------------- SIMULATION END ----------------" << std::endl;
    
    return 1;
}
