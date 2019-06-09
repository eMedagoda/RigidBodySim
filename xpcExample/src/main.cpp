//
//  main.cpp
//  xpcExample
//
//  Created by Chris Teubert on 3/27/14.
//  Copyright (c) 2014 Chris Teubert. All rights reserved.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "xplaneConnect.h"
#include <iostream>
#ifdef WIN32
#include <Windows.h>
#define sleep(n) Sleep(n * 1000)
#endif

int main()
{
	printf("XPlaneConnect Example Script\n- Setting up Simulation\n");

	// Open Socket
	const char* IP = "127.0.0.1";      //IP Address of computer running X-Plane
	XPCSocket sock = openUDP(IP);
	float tVal[1];
	int tSize = 1;
	if (getDREF(sock, "sim/test/test_float", tVal, &tSize) < 0)
	{
		printf("Error establishing connecting. Unable to read data from X-Plane.");
		return EXIT_FAILURE;
	}

	float alt = 2500;
	
	// Set Location/Orientation (sendPOSI)
	// Set Up Position Array
	float POSI[9] = { 0.0 };
	POSI[0] = -27.0;     // Lat
	POSI[1] = 153.0;       // Lon
	POSI[2] = alt;       // Alt
	POSI[3] = 0;          // Pitch
	POSI[4] = 0;          // Roll
	POSI[5] = 0;          // Heading
	POSI[6] = 1;          // Gear

	// Set position of the player aircraft
	sendPOSI(sock, POSI, 7, 0);

	// pauseSim
	pauseSim(sock, 1); // Sending 1 to pause	
	sleep(5); // Pause for 5 seconds

	// Unpause
	pauseSim(sock, 0); // Sending 0 to unpause
	printf("- Resuming Simulation\n");
    
    const char* dref = "sim/operation/override/override_planepath"; // Gear handle data reference
	float result[8] = {1,0,0,0,0,0,0,0};    

	sendDREF(sock, dref, result, 1); // Set gear to stow
    
    clock_t t;
    t = clock(); // current system time (micro seconds)
    clock_t t_prev = t;
    
    double dt, DT2;
    
    while (true)
    {         
        t = clock(); // current system time
        
        DT2 = (float)(t - t_prev)/CLOCKS_PER_SEC; // elapsed time (seconds)
        
        if(DT2 >= 0.1)
        {
            t_prev = t; // update time                   
            
            alt += -20.0 * 0.1;            
            
            POSI[0] = -27.0;     // Lat
            POSI[1] = 153.0;       // Lon
            POSI[2] = alt;       // Alt
            POSI[3] = 0;          // Pitch
            POSI[4] = 0;          // Roll
            POSI[5] = 0;          // Heading
            POSI[6] = 1;          // Gear     
            
            std::cout << POSI[2] << std::endl;
            
//             sendPOSI(sock, POSI, 7, 0);
        }
        
        // Set position of the player aircraft
        sendPOSI(sock, POSI, 7, 0);
    }    

	printf("---End Program---\n");

	return 0;
}
