#include "ServerSocket.h"
#include "SocketException.h"
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>

int main ()
{
  std::cout << "running....\n";

  try
    {
      // Create the socket
      ServerSocket server ( 30000 );

      while ( true )
    {

      ServerSocket new_sock;
      server.accept ( new_sock );

      try
        {
          while ( true )
        {
          std::string data; // initialise recieved data string
          new_sock >> data; // read data from new_sock
          
        // parse data string from client into doubles
        std::stringstream ss(data);
        std::vector<double> output;
        double i;
        while (ss >> i)
        {
            output.push_back(i); // create output vector of doubles

            if (ss.peek() == ',') // only ignore commas
            ss.ignore();
        }                      
          
          std::cout << "client data: " << data << std::endl;
          std::cout << std::setprecision(8) << "processed data: " << output[0] << ", " << output[1] << std::endl;
          
          new_sock << data; // send data to new_sock (back to client)
        }
        }
      catch ( SocketException& ) {}

    }
    }
  catch ( SocketException& e )
    {
      std::cout << "Exception was caught:" << e.description() << "\nExiting.\n";
    }

  return 0;
}
