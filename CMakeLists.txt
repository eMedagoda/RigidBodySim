cmake_minimum_required(VERSION 3.0)
project (RigidBodySim)

include_directories(Eigen-3.3)

add_executable(
    RigidBodySim 
    main.cpp 
    Vehicle.cpp 
    Socket.cpp
    ClientSocket.cpp)
    
set_property(TARGET RigidBodySim PROPERTY CXX_STANDARD 11)
