#include "Vehicle.h"
#include "PID.h"
#include "VehicleParameters.h"
#include "ClientSocket.h"
#include "SocketException.h"
#include "Eigen-3.3/Eigen/Eigen"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <time.h>
#include <string>

using namespace Eigen;

int main()
{

    Vehicle Veh;

    double DT = 0.1;
    double TIME = 60;
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
    double u_sum = 0.0;
    double w_sum = 0.0;
    
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
    
    for (int i = 0; i < (int)T_SIZE; i++)
    {       
        Veh.Integrate(X,U,DT); // update vehicle states
        
        //--------------------------- GUIDANCE SEGMENT -------------------------------            
        
        VectorXd dU(n_cont);
        dU.setZero();
    
        MatrixXd C_bn = Veh.DirectionCosineMatrix(X(6),X(7),X(8));
        
        if (i > (10.0/DT))
        {
            VfC = 5.0;
            VvC = -3.0;
            phiC = 0.0 * DEG2RAD;
            thetaC = 0.0 * DEG2RAD;
            dpsiC = 0.0 * DEG2RAD;
        }       
        
        if (i > (20.0/DT))
        {
            VfC = 0.0;
            VvC = 0.0;
            phiC = 0.0 * DEG2RAD;
            thetaC = 0.0 * DEG2RAD;
            dpsiC = 0.0 * DEG2RAD;
        } 
        
        VectorXd VnC(3);
        VnC << VfC*cos(X(8)), VfC*sin(X(8)), VvC; // navigation frame speed commands (NED) (XdotC, YdotC, ZdotC)        
        
        VectorXd VbC = C_bn * VnC; // body frame speed commands
        
        //--------------------------- CONTROL SEGMENT --------------------------------                                         

        // forward speed control
        PID u_cont(10.0, 0.0, 0.1);
        dU(0) = u_cont.Control(VbC(0), X(0), DT);        
        
        // vertical speed control
        PID w_cont(10.0, 0.0, 0.1);
        dU(2) = w_cont.Control(VbC(2), X(2), DT);
        
        // roll regulator
        double e_phi = phiC - X(6);
        dU(3) = 0.01 * e_phi + 0.02 * (0.0 - X(3)) + 0.0001 * (VbC(1) - X(1));
        
        // pitch control
        double e_theta = thetaC - X(7);
        dU(4) = 0.003 * e_theta + 0.01 * (0.0 - X(4));        
        
        // heading controller
        double e_dpsi = dpsiC - X(5);  
        dU(5) = 0.02 * e_dpsi;
        
        U = U_Trim + dU;
            
        //----------------------------------------------------------------------------                
       
        std::cout << std::setprecision(2) 
           << std::fixed
           << (i + 1)*DT << ", " 
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
           << "| "
           << U(0) << ", " 
           << U(1) << ", " 
           << U(2) << ", " 
           << U(3) << ", " 
           << U(4) << ", " 
           << U(5) <<  std::endl;        
       
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
           << X(12) << ", " 
           << X(13) << ", " 
           << X(14) << ", " 
           << U(0) << ", " 
           << U(1) << ", " 
           << U(2) << ", " 
           << U(3) << ", " 
           << U(4) << ", " 
           << U(5) <<  std::endl;
        
    }    
    
    //--------------------- REAL TIME -----------------------------------
    
//     clock_t t;
//     int f;
//     t = clock(); // current system time (micro seconds)
//     clock_t t_prev = t;
//     
//     double dt,DT2;
//     int i = 0;
//     
//     try
//     {
// 
//       ClientSocket client_socket ( "localhost", 30000 );
//       
//         while (true)
//         {        
//             std::string reply;
//             
//             t = clock(); // current system time
//             
//             DT2 = (float)(t - t_prev)/CLOCKS_PER_SEC; // elapsed time (seconds)
//             
//             if(DT2 >= DT)
//             {            
//                 t_prev = t; // update time
//                 
//                 Veh.Integrate(X,U,DT2); // update vehicle states                     
//                             
//                 std::cout << std::setprecision(2) 
//                             << std::fixed
//                             << (i + 1)*DT << ", " 
//                             << X(0) << ", " 
//                             << X(1) << ", " 
//                             << X(2) << ", " 
//                             << X(3) << ", "  
//                             << X(4) << ", " 
//                             << X(5) << ", " 
//                             << X(6) << ", " 
//                             << X(7) << ", " 
//                             << X(8) << ", "  
//                             << X(9) << ", " 
//                             << X(10) << ", "  
//                             << X(11) << ", " 
//                             << "| "
//                             << U(0) << ", " 
//                             << U(1) << ", " 
//                             << U(2) << ", " 
//                             << U(3) << ", " 
//                             << U(4) << ", " 
//                             << U(5) <<  std::endl;
//                 
//                 // send to output file for logging
//                 myfile << (i + 1)*DT << ", " 
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
//                         << U(0) << ", " 
//                         << U(1) << ", " 
//                         << U(2) << ", " 
//                         << U(3) << ", " 
//                         << U(4) << ", " 
//                         << U(5) <<  std::endl;
//             
//             // send data UDP packet to server
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
// //             std::cout << "We received this response from the server: " << reply << std::endl;
//                 
//                 i++;
//                 
//             }
//             
// //             try
// //             {                
// //                 // convert relevant data into a string
// //                 std::string data = std::to_string(X(9));
// //                 std::string data2 = std::to_string(X(10));
// //                 
// //                 // append all data into a single string for transmission to server
// //                 std::string data_all = data + "," + data2;
// //                     
// //                 client_socket << data_all; // send data to server
// //                 client_socket >> reply; // received data from server
// //             }
// //             
// //             catch ( SocketException& ) {}
// // 
// //             std::cout << "We received this response from the server: " << reply << std::endl;
//             
//         }
//                     
//     }
//     
//     catch ( SocketException& e )
//     {
//       std::cout << "Exception was caught:" << e.description() << "\n";
//     }

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

