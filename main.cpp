#include "Vehicle.h"
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

    Vehicle Veh1;

    double DT = 0.1;
    double TIME = 60;
    double T_SIZE = TIME/DT;
    
    int n_out = 15;    // number of tracked outputs (states)
    int n_cont = 6;   // number of control inputs
    
    // initialise states and controls
    VectorXd X(n_out);
    VectorXd U(n_cont);
        
    // initial states
    double VelTrim = 10.35;
    double AltTrim = 10.0;
    double ThetaTrim = -10.0 * DEG2RAD;
    double PsiTrim   = 90.0 * DEG2RAD;
    double LonTrim   = 153.0 * DEG2RAD;
    double LatTrim   = -27.0 * DEG2RAD;
    
    // determine trim states and controls
    Veh1.Trim(X, U, VelTrim, AltTrim, ThetaTrim, PsiTrim, LonTrim, LatTrim);
  
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
//         Veh1.Integrate(X,U,DT); // update vehicle states              
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
            
            if(DT2 >= DT)
            {            
                t_prev = t; // update time
                
                Veh1.Integrate(X,U,DT2); // update vehicle states                     
                            
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
                std::string data = std::to_string(X(9));
                std::string data2 = std::to_string(X(10));
                
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
//             Veh1.Integrate(X,U,DT2); // update vehicle states                     
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

