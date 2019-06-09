#include "Vehicle.h"
#include "PID.h"
#include "VehicleParameters.h"
#include "Eigen-3.3/Eigen/Eigen"

#include "UDP/ClientSocket.h"
#include "UDP/SocketException.h"

#include "Joystick/joystick.hh"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <time.h>
#include <string>
#include <unistd.h>

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
    
    Vehicle Veh;

    double DT = 0.1;
    double TIME = 100;
    double T_SIZE = TIME/DT;
    
    int n_out = 15;    // number of tracked outputs (states)
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
    double AltTrim = 10.0;
    double ThetaTrim = 0.0 * DEG2RAD;
    double PsiTrim   = 90.0 * DEG2RAD;
    double LonTrim   = 153.0 * DEG2RAD;
    double LatTrim   = -27.0 * DEG2RAD;    
    
    // initial guidance states
    double VfC = VelTrim; // forward speed command
    double VvC = 0.0; // vertical speed command
    double phiC = 0.0; // bank angle command
    double thetaC = ThetaTrim; // pitch angle command
    double dpsiC = 0.0; // yaw rate command
    static double u_sum = 0.0;
    static double w_sum = 0.0;
    
    // determine trim states and controls
    Veh.Trim(X, U, VelTrim, AltTrim, ThetaTrim, PsiTrim, LonTrim, LatTrim);
    
    U_Trim = U;
  
    std::cout << "---------------- RUN SIMULATION ----------------" << std::endl;
  
    std::ofstream myfile;
    
    myfile.open ("output.csv");
    
    myfile << 0 << ", " 
           << X(0) << ", " 
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
           << U(0) << ", " 
           << U(1) << ", " 
           << U(2) << ", " 
           << U(3) << ", " 
           << U(4) << ", " 
           << U(5) <<  std::endl;
         
    //----------------------- RAPID --------------------------------------------------
    
//     for (int i = 0; i < (int)T_SIZE; i++)
//     {       
//         Veh.Integrate(X,U,DT); // update vehicle states
//         
//         //--------------------------- GUIDANCE SEGMENT -------------------------------            
//         
//         VectorXd dU(n_cont);
//         dU.setZero();
//     
//         MatrixXd C_bn = Veh.DirectionCosineMatrix(X(6),X(7),X(8));
//         
//         if (i > (10.0/DT))
//         {
//             VfC = 6.0;
//             VvC = 0.0;
//             phiC = 0.0 * DEG2RAD;
//             thetaC = 0.0 * DEG2RAD;
//             dpsiC = 15.0 * DEG2RAD;
//         }       
//         
//         if (i > (30.0/DT))
//         {
//             VfC = 6.0;
//             VvC = 0.0;
//             phiC = 0.0 * DEG2RAD;
//             thetaC = 0.0 * DEG2RAD;
//             dpsiC = 0.0 * DEG2RAD;
//         } 
//         
//         VectorXd VnC(3);
//         VnC << VfC*cos(X(8)), VfC*sin(X(8)), VvC; // navigation frame speed commands (NED) (XdotC, YdotC, ZdotC)        
//         
//         VectorXd VbC = C_bn * VnC; // body frame speed commands
//         
//         //--------------------------- CONTROL SEGMENT --------------------------------                                         
// 
//         // forward speed control
//         PID u_cont(10.0, 0.1, 0.0);
//         dU(0) = u_cont.Control(VbC(0), X(0), DT);        
//         
//         // vertical speed control
//         PID w_cont(10.0, 0.1, 0.0);
//         dU(2) = w_cont.Control(VbC(2), X(2), DT);         
//         
//         PID phi_cont(0.01, 0.0, 0.0);
//         PID p_cont(0.05, 0.0, 0.0);
//         PID v_cont(0.0001, 0.0, 0.0);        
//         double phi_out = phi_cont.Control(phiC, X(6), DT);
//         double p_out = p_cont.Control(0.0, X(3), DT);
//         double v_out = v_cont.Control(VbC(1), X(1), DT);        
//         dU(3) = phi_out + p_out + v_out;
//         
//         PID theta_cont(0.003, 0.0, 0.0);
//         PID q_cont(0.01, 0.0, 0.0);        
//         double pitch_out = theta_cont.Control(thetaC, X(7), DT);
//         double q_out = q_cont.Control(0.0, X(4), DT);
//         dU(4) = pitch_out + q_out;        
//         
//         PID dpsi_cont(0.02, 0.0, 0.0);        
//         double dpsi_out = dpsi_cont.Control(dpsiC, X(5), DT);
//         dU(5) = dpsi_out;
//         
//         U = U_Trim + dU;
//             
//         //----------------------------------------------------------------------------                
//        
//         std::cout << std::setprecision(2) 
//            << std::fixed
//            << (i + 1)*DT << ", " 
//            << X(0) << ", " 
//            << X(1) << ", " 
//            << X(2) << ", " 
//            << X(3) << ", "  
//            << X(4) << ", " 
//            << X(5) << ", " 
//            << X(6) << ", " 
//            << X(7) << ", " 
//            << X(8) << ", "  
//            << X(9) << ", " 
//            << X(10) << ", "  
//            << X(11) << ", "
//            << "| "
//            << U(0) << ", " 
//            << U(1) << ", " 
//            << U(2) << ", " 
//            << U(3) << ", " 
//            << U(4) << ", " 
//            << U(5) <<  std::endl;        
//        
//         myfile << (i + 1)*DT << ", " 
//            << X(0) << ", " 
//            << X(1) << ", " 
//            << X(2) << ", " 
//            << X(3) << ", "  
//            << X(4) << ", " 
//            << X(5) << ", " 
//            << X(6) << ", " 
//            << X(7) << ", " 
//            << X(8) << ", "  
//            << X(9) << ", " 
//            << X(10) << ", "  
//            << X(11) << ", "
//            << X(12) << ", " 
//            << X(13) << ", " 
//            << X(14) << ", " 
//            << U(0) << ", " 
//            << U(1) << ", " 
//            << U(2) << ", " 
//            << U(3) << ", " 
//            << U(4) << ", " 
//            << U(5) <<  std::endl;
//         
//     }    
    
    //--------------------- REAL TIME -----------------------------------
    
    clock_t t;
    int f;
    t = clock(); // current system time (micro seconds)
    clock_t t_prev = t;
    
    double dt,DT2;
    int i = 0;
    
    try
    {

      ClientSocket client_socket ( "localhost", 30000 );
      
        while (true)
        {        
            std::string reply;
            
            t = clock(); // current system time
            
            DT2 = (float)(t - t_prev)/CLOCKS_PER_SEC; // elapsed time (seconds)
            
            // Attempt to sample an event from the joystick
            JoystickEvent event;
            if (joystick.sample(&event))
            {        
                joystick.GetInputs(inputs, event);        
            }
            
            // convert joystick inputs into
            VfC = 0.0;
            VvC = -2.5 * inputs.ax3;
            phiC = 10.0 * inputs.ax0 * DEG2RAD;
            thetaC = 20.0 * inputs.ax1 * DEG2RAD;
            dpsiC = 30.0 * inputs.ax2 * DEG2RAD;
            
            VectorXd dU(n_cont);
            dU.setZero();
        
            MatrixXd C_bn = Veh.DirectionCosineMatrix(X(6),X(7),X(8));
            
            VectorXd VnC(3);
            VnC << VfC*cos(X(8)), VfC*sin(X(8)), VvC; // navigation frame speed commands (NED) (XdotC, YdotC, ZdotC)       
            VectorXd VbC = C_bn * VnC; // body frame speed commands
            
            double u_error = VbC(0) - X(0);
            dU(0) = 10.0 * u_error + 0.1 * u_sum * DT;
            
            if (fabs(u_error) < 2.0)
            {
                u_sum += u_error;
            }
            else
            {
                u_sum = 0.0;
            }            
            
            double w_error = VbC(2) - X(2);
            dU(2) = 10.0 * w_error + 0.1 * w_sum * DT;
            
            if (fabs(w_error) < 2.0)
            {
                w_sum += w_error;
            }
            else
            {
                w_sum = 0.0;
            }
            
//             // forward speed control
//             PID u_cont(10.0, 0.1, 0.0);
//             dU(0) = u_cont.Control(VbC(0), X(0), DT);        
//             
//             // vertical speed control
//             PID w_cont(10.0, 0.1, 0.0);
//             dU(2) = w_cont.Control(VbC(2), X(2), DT);         
            
            PID phi_cont(1.5, 0.0, 0.0);
            PID p_cont(0.3, 0.0, 0.0);
            PID v_cont(0.0, 0.0, 0.0);        
            double p_C = phi_cont.Control(phiC, X(6), DT);
            double p_out = p_cont.Control(p_C, X(3), DT);
            double v_out = v_cont.Control(VbC(1), X(1), DT);        
            dU(3) = p_out + v_out;
            
            PID theta_cont(0.3, 0.0, 0.0);
            PID q_cont(0.2, 0.0, 0.0);        
            double pitch_out = theta_cont.Control(thetaC, X(7), DT);
            double q_out = q_cont.Control(0.0, X(4), DT);
            dU(4) = pitch_out + q_out;        
            
            PID dpsi_cont(0.2, 0.0, 0.0);        
            double dpsi_out = dpsi_cont.Control(dpsiC, X(5), DT);
            dU(5) = dpsi_out;
            
            U = U_Trim + dU;           
           
            if(DT2 >= DT)
            {            
                t_prev = t; // update time
                
                Veh.Integrate(X,U,DT2); // update vehicle states                     
                            
                std::cout << std::setprecision(2) 
                            << std::fixed
                            << (i + 1)*DT << ", " 
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
                
                // send to output file for logging
                myfile << (i + 1)*DT << ", " 
                        << X(0) << ", " 
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
                        << U(0) << ", " 
                        << U(1) << ", " 
                        << U(2) << ", " 
                        << U(3) << ", " 
                        << U(4) << ", " 
                        << U(5) <<  std::endl;
            
            // send data UDP packet to server
            try
            {                
                // convert relevant data into a string
                std::string data = std::to_string(phiC*RAD2DEG);
                std::string data2 = std::to_string(thetaC*RAD2DEG);
                
                // append all data into a single string for transmission to server
                std::string data_all = data + "," + data2;
                    
                client_socket << data_all; // send data to server
                client_socket >> reply; // received data from server
            }
            
            catch ( SocketException& ) {}

//             std::cout << "We received this response from the server: " << reply << std::endl;
                
                i++;
                
            }
            
//             try
//             {                
//                 // convert relevant data into a string
//                 std::string data = std::to_string(X(9));
//                 std::string data2 = std::to_string(X(10));
//                 
//                 // append all data into a single string for transmission to server
//                 std::string data_all = data + "," + data2;
//                     
//                 client_socket << data_all; // send data to server
//                 client_socket >> reply; // received data from server
//             }
//             
//             catch ( SocketException& ) {}
// 
//             std::cout << "We received this response from the server: " << reply << std::endl;
            
        }
                    
    }
    
    catch ( SocketException& e )
    {
      std::cout << "Exception was caught:" << e.description() << "\n";
    }

// ======================== ORIGINAL ================================     
//     while (1)
//     {        
//         t = clock(); // current system time
//         
//         DT2 = (float)(t - t_prev)/CLOCKS_PER_SEC; // elapsed time (seconds)
//         
//         if(DT2 >= DT)
//         {            
//             t_prev = t; // update time
//             
//             Veh.Integrate(X,U,DT2); // update vehicle states                     
//                         
//             std::cout << std::setprecision(2) 
//                         << std::fixed
//                         << (i + 1)*DT << ", " 
//                         << X(0) << ", " 
//                         << X(1) << ", " 
//                         << X(2) << ", " 
//                         << X(3) << ", "  
//                         << X(4) << ", " 
//                         << X(5) << ", " 
//                         << X(6) << ", " 
//                         << X(7) << ", " 
//                         << X(8) << ", "  
//                         << X(9) << ", " 
//                         << X(10) << ", "  
//                         << X(11) << ", " 
//                         << "| "
//                         << U(0) << ", " 
//                         << U(1) << ", " 
//                         << U(2) << ", " 
//                         << U(3) << ", " 
//                         << U(4) << ", " 
//                         << U(5) <<  std::endl;
//             
//             myfile << (i + 1)*DT << ", " 
//                     << X(0) << ", " 
//                     << X(1) << ", " 
//                     << X(2) << ", " 
//                     << X(3) << ", "  
//                     << X(4) << ", " 
//                     << X(5) << ", " 
//                     << X(6) << ", " 
//                     << X(7) << ", " 
//                     << X(8) << ", "  
//                     << X(9) << ", " 
//                     << X(10) << ", "  
//                     << X(11) << ", " 
//                     << U(0) << ", " 
//                     << U(1) << ", " 
//                     << U(2) << ", " 
//                     << U(3) << ", " 
//                     << U(4) << ", " 
//                     << U(5) <<  std::endl;
//             
//             i++;
//             
//         }
//         
//     }
    // ==============================================================
    
    //----------------------------------------------------------
    
    myfile.close();
    
    std::cout << "---------------- SIMULATION END ----------------" << std::endl;
    
    return 1;
}

