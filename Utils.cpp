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

VectorXd Utils::EulerToQuaternion(double phi, double theta, double psi)
{ 
    VectorXd quat(4);
    quat.setZero();
    
    double sphi = sin(phi/2.0);
    double cphi = cos(phi/2.0);
    double spsi = sin(psi/2.0);
    double cpsi = cos(psi/2.0);
    double stht = sin(theta/2.0);
    double ctht = cos(theta/2.0);

    quat(0) = cpsi*ctht*cphi+spsi*stht*sphi;
    quat(1) = cpsi*ctht*sphi-spsi*stht*cphi;
    quat(2) = cpsi*stht*cphi+spsi*ctht*sphi;
    quat(3) = -cpsi*stht*sphi+spsi*ctht*cphi;
    
    QuaternionNormalise(quat);
    
    return quat;
}

VectorXd Utils::QuatToEuler(VectorXd quat)
{
    VectorXd euler(3);
    euler.setZero();
    
    return euler;
}

MatrixXd Utils::QuatToDirectionCosineMatrix(VectorXd quat)
{
    MatrixXd C_bn(3,3);
    
    C_bn(0,0) = quat(0)*quat(0) + quat(1)*quat(1) - quat(2)*quat(2) - quat(3)*quat(3);
	C_bn(0,1) = 2.0*(quat(1)*quat(2) + quat(0)*quat(3));
    C_bn(0,2) = 2.0*(quat(1)*quat(3) - quat(0)*quat(2));
    
    C_bn(1,0) = 2.0*(quat(1)*quat(2) - quat(0)*quat(3));
    C_bn(1,1) = quat(0)*quat(0) - quat(1)*quat(1) + quat(2)*quat(2) - quat(3)*quat(3);
    C_bn(1,2) = 2.0*(quat(2)*quat(3) + quat(0)*quat(1));
    
    C_bn(2,0) = 2.0*(quat(1)*quat(3) + quat(0)*quat(2));
    C_bn(2,1) = 2.0*(quat(2)*quat(3) - quat(0)*quat(1));
    C_bn(2,2) = quat(0)*quat(0) - quat(1)*quat(1) - quat(2)*quat(2) + quat(3)*quat(3);
    
    return C_bn;
}

MatrixXd Utils::QuaternionRateMap(VectorXd quat)
{
    MatrixXd Q(4,3);    
    
    Q(0,0) = -quat(1,0);
    Q(0,1) = -quat(2,0);
    Q(0,2) = -quat(3,0);

    Q(1,0) = quat(0,0);
    Q(1,1) = -quat(3,0);
    Q(1,2) = quat(2,0);

    Q(2,0) = quat(3,0);
    Q(2,1) = quat(0,0);
    Q(2,2) = -quat(1,0);

    Q(3,0) = -quat(2,0);
    Q(3,1) = quat(1,0);
    Q(3,2) = quat(0,0);

    return Q;
}

VectorXd Utils::QuaternionTranspose(VectorXd quat)
{
    VectorXd quat_transpose(4);
    
    quat_transpose(0) = quat(0);
    quat_transpose(1) = -quat(1);
    quat_transpose(2) = -quat(2);
    quat_transpose(3) = -quat(3);
    
    QuaternionNormalise(quat_transpose);
    
    return quat_transpose;
}

void Utils::QuaternionNormalise(VectorXd& quat)
{
    double quatnorm = sqrt(quat(0)*quat(0) + quat(1)*quat(1) + quat(2)*quat(2) + quat(3)*quat(3));
    
    quat = quat/quatnorm;
}

VectorXd Utils::QuaternionProduct(VectorXd q, VectorXd p)
{
    VectorXd quat_out(4);
    quat_out.setZero();
    
    MatrixXd Q(4,4);
    Q << q(0,0), -q(1,0), -q(2,0), -q(3,0),
         q(1,0),  q(0,0),  q(3,0), -q(2,0),
         q(2,0), -q(3,0),  q(0,0),  q(1,0),
         q(3,0),  q(2,0), -q(1,0),  q(0,0);
        
    quat_out = Q * p;
    
    return quat_out;

}    