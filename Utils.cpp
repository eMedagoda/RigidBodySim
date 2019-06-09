#include "Utils.h"
#include "VehicleParameters.h"
#include <cmath>
#include <iostream>

Utils::Utils()
{
}

Utils::~Utils()
{
}

MatrixXd Utils::DirectionCosineMatrix(double phi, double theta, double psi)
{
    // rotational transformation about z-axis
    MatrixXd C3 = L3(psi);

    // rotational transformation about y-axis
    MatrixXd C2 = L2(theta);

    // rotational transformation about x-axis
    MatrixXd C1 = L1(phi);

    // transformation matrix (navigation frame to body frame)
    MatrixXd C_bn = C1 * C2 * C3;
    
    return C_bn;

}

MatrixXd Utils::L1(double phi)
{
    MatrixXd Cx(3,3);
    
    // rotation about x-axis
    Cx << 1.0, 0.0, 0.0,
          0.0,  cos(phi),  sin(phi),
          0.0,  -sin(phi), cos(phi);
          
    return Cx; 
    
}

MatrixXd Utils::L2(double theta)
{
    MatrixXd Cy(3,3);
    
    // rotation about y-axis
    Cy << cos(theta), 0.0,  -sin(theta),
          0.0, 1.0, 0.0,
          sin(theta), 0.0, cos(theta);
          
    return Cy; 
    
}

MatrixXd Utils::L3(double psi)
{
    MatrixXd Cz(3,3);
        
    // rotation about z-axis
    Cz << cos(psi),  sin(psi), 0.0,
         -sin(psi),  cos(psi), 0.0,
         0.0, 0.0, 1.0;
          
    return Cz; 
    
}

void Utils::PiMinusPi(double& input)
{
    
    if (input <= -PI)
    {
        input += 2.0 * PI;
    }
    else if (input >= PI)
    {
        input -= 2.0 * PI;
    }
    
}