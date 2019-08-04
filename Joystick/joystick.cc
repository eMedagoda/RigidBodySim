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

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sstream>
#include "unistd.h"

Joystick::Joystick()
{
  openPath("/dev/input/js0");
}

Joystick::Joystick(int joystickNumber)
{
  std::stringstream sstm;
  sstm << "/dev/input/js" << joystickNumber;
  openPath(sstm.str());
}

Joystick::Joystick(std::string devicePath)
{
  openPath(devicePath);
}

Joystick::Joystick(std::string devicePath, bool blocking)
{
  openPath(devicePath, blocking);
}

void Joystick::openPath(std::string devicePath, bool blocking)
{
  // Open the device using either blocking or non-blocking
  _fd = open(devicePath.c_str(), blocking ? O_RDONLY : O_RDONLY | O_NONBLOCK);
}

bool Joystick::sample(JoystickEvent* event)
{
  int bytes = read(_fd, event, sizeof(*event));

  if (bytes == -1)
    return false;

  // NOTE if this condition is not met, we're probably out of sync and this
  // Joystick instance is likely unusable
  return bytes == sizeof(*event);
}

bool Joystick::isFound()
{
  return _fd >= 0;
}

Joystick::~Joystick()
{
  close(_fd);
}

void Joystick::GetInputs(jinput &inputs, JoystickEvent event)
{
    if (event.isAxis() && event.number == 0)
    {
        inputs.ax0 = event.value/((double) JoystickEvent::MAX_AXES_VALUE); // roll
    }
    if (event.isAxis() && event.number == 1)
    {
        inputs.ax1 = event.value/((double) JoystickEvent::MAX_AXES_VALUE); // pitch
    }
    if (event.isAxis() && event.number == 2)
    {
        inputs.ax2 = event.value/((double) JoystickEvent::MAX_AXES_VALUE); // yaw
    }
    if (event.isAxis() && event.number == 3)
    {
        inputs.ax3 = (-event.value)/((double) JoystickEvent::MAX_AXES_VALUE); // throttle
    }
    if (event.isAxis() && event.number == 4)
    {
        inputs.ax4 = event.value/((double) JoystickEvent::MAX_AXES_VALUE); // top hat, lateral
    }
    if (event.isAxis() && event.number == 5)
    {
        inputs.ax5 = event.value/((double) JoystickEvent::MAX_AXES_VALUE); // top hat, longitudinal
    }
    if (event.isButton() && event.number == 0)
    {
        inputs.ax6 = event.value;
    }
    if (event.isButton() && event.number == 1)
    {
        inputs.ax7 = event.value;
    }
}

std::ostream& operator<<(std::ostream& os, const JoystickEvent& e)
{
  os << "type=" << static_cast<int>(e.type)
     << " number=" << static_cast<int>(e.number)
     << " value=" << static_cast<int>(e.value);
  return os;
}
