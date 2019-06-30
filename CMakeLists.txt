cmake_minimum_required(VERSION 3.0)
project (RigidBodySim)

include_directories(Eigen-3.3)

add_executable(
    RigidBodySim 
    main_xplane.cpp 
    Vehicle.cpp
    PID.cpp
    Sensors.cpp
    Utils.cpp
    EKF.cpp
    UDP/Socket.cpp
    UDP/ClientSocket.cpp
    Joystick/joystick.cc
    xpcExample/src/xplaneConnect.c)
    
set_property(TARGET RigidBodySim PROPERTY CXX_STANDARD 11)
