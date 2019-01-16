#include "ClientSocket.h"
#include "SocketException.h"
#include <iostream>
#include <string>
#include <cmath>

int main ()
{
    
    int i = 0;
          
    try
    {

      ClientSocket client_socket ( "localhost", 30000 );
      
      while (true)
      {      
            std::string reply;            

            try
            {
                
            // convert relevant data into a string
            std::string data = std::to_string(sin(i));
            std::string data2 = std::to_string(cos(3*i));
            
            // append all data into a single string for transmission to server
            std::string data_all = data + "," + data2;
                
            client_socket << data_all; // send data to server
            client_socket >> reply; // received data from server
            
            i++;
            }
            catch ( SocketException& ) {}

            std::cout << "We received this response from the server: " << reply << std::endl;

        }
            
    }
    
    catch ( SocketException& e )
    {
      std::cout << "Exception was caught:" << e.description() << "\n";
    }

  return 0;
}
