// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Copyright Drew Noakes 2013-2016

#include "joystick.hh"
#include <unistd.h>
#include <iostream>

// void GetInputs(Joystick::jinput &inputs, JoystickEvent event);

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

  while (true)
  {
    // Restrict rate
    usleep(1000); // milliseconds

    // Attempt to sample an event from the joystick
    JoystickEvent event;
    if (joystick.sample(&event))
    {
//       if (event.isButton())
//       {
//         printf("Button %u is %s\n",
//           event.number,
//           event.value == 0 ? "up" : "down");
//       }
//       else if (event.isAxis() && event.number == 0)
//       {
//         printf("Axis %u is at position %d\n", event.number, event.value);        
//       }
//     } 
        
        joystick.GetInputs(inputs, event);        
    }
    
    std::cout << "Axis values: " 
              << inputs.ax0 << ", " 
              << inputs.ax1 << ", " 
              << inputs.ax2 << ", " 
              << inputs.ax3 << ", " 
              << inputs.ax4 << ", " 
              << inputs.ax5 << std::endl;
    
  }
  
}

// void GetInputs(Joystick::jinput &inputs, JoystickEvent event)
// {
//     if (event.isAxis() && event.number == 0)
//     {
//         inputs.ax0 = event.value;
//     }
//     if (event.isAxis() && event.number == 1)
//     {
//         inputs.ax1 = event.value;
//     }
//     if (event.isAxis() && event.number == 2)
//     {
//         inputs.ax2 = event.value;
//     }
//     if (event.isAxis() && event.number == 3)
//     {
//         inputs.ax3 = event.value;
//     }
//     if (event.isAxis() && event.number == 4)
//     {
//         inputs.ax4 = event.value;
//     }
//     if (event.isAxis() && event.number == 5)
//     {
//         inputs.ax5 = event.value;
//     }
// }
    