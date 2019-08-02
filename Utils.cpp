#include "Utils.h"
#include "VehicleParameters.h"
#include "MathConstants.h"
#include <cmath>
#include <iostream>

#define sign_check(x) (( (x) > 0 ) ? (1) : (-1))

Utils::Utils()
{
}

Utils::~Utils()
{
}

Matrix3d Utils::DirectionCosineMatrix(double phi, double theta, double psi)
{
    // rotational transformation about z-axis
    Matrix3d C3 = L3(psi);

    // rotational transformation about y-axis
    Matrix3d C2 = L2(theta);

    // rotational transformation about x-axis
    Matrix3d C1 = L1(phi);

    // transformation matrix (navigation frame to body frame)
    Matrix3d C_bn = C1 * C2 * C3;

    return C_bn;

}

Matrix3d Utils::L1(double phi)
{
    Matrix3d Cx;

    // rotation about x-axis
    Cx << 1.0, 0.0, 0.0,
          0.0,  cos(phi),  sin(phi),
          0.0,  -sin(phi), cos(phi);

    return Cx;

}

Matrix3d Utils::L2(double theta)
{
    Matrix3d Cy;

    // rotation about y-axis
    Cy << cos(theta), 0.0,  -sin(theta),
          0.0, 1.0, 0.0,
          sin(theta), 0.0, cos(theta);

    return Cy;

}

Matrix3d Utils::L3(double psi)
{
    Matrix3d Cz;

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

Vector3d Utils::QuatToEuler(VectorXd quat)
{
    Vector3d euler;
    euler.setZero();

    euler(0) = atan2((2.0 * (quat(0)*quat(1) + quat(2)*quat(3))),(1.0 - 2.0 * (quat(1)*quat(1) + quat(2)*quat(2))));
    euler(1) = asin(2.0 * (quat(0)*quat(2) - quat(3)*quat(1)));
    euler(2) = atan2((2.0 * (quat(0)*quat(3) + quat(1)*quat(2))),(1.0 - 2.0 * (quat(2)*quat(2) + quat(3)*quat(3))));

    return euler;
}

Matrix3d Utils::QuatToDirectionCosineMatrix(VectorXd quat)
{
    Matrix3d C_bn;
    C_bn.setZero();

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
    Q.setZero();

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

    return quat_transpose;
}

VectorXd Utils::QuaternionChangeHand(VectorXd quat)
{
    VectorXd quat_change_hand(4);

    quat_change_hand(0) = -quat(0);
    quat_change_hand(1) = -quat(1);
    quat_change_hand(2) = -quat(2);
    quat_change_hand(3) = -quat(3);

    return quat_change_hand;
}

VectorXd Utils::QuaternionProduct(VectorXd p, VectorXd q)
{
    VectorXd quat_out(4);
    quat_out.setZero();

    MatrixXd Q(4,4);
    Q.setZero();

    Q(0,0) = q(0,0);
    Q(0,1) = -q(1,0);
    Q(0,2) = -q(2,0);
    Q(0,3) = -q(3,0);

    Q(1,0) = q(1,0);
    Q(1,1) = q(0,0);
    Q(1,2) = q(3,0);
    Q(1,3) = -q(2,0);

    Q(2,0) = q(2,0);
    Q(2,1) = -q(3,0);
    Q(2,2) = q(0,0);
    Q(2,3) = q(1,0);

    Q(3,0) = q(3,0);
    Q(3,1) = q(2,0);
    Q(3,2) = -q(1,0);
    Q(3,3) = q(0,0);

    quat_out = Q * p;

    return quat_out;
}

void Utils::QuaternionNormalise(VectorXd& quat)
{
    double quatnorm = sqrt(quat(0)*quat(0) + quat(1)*quat(1) + quat(2)*quat(2) + quat(3)*quat(3));

    quat = quat/quatnorm;
}

Matrix3d Utils::VectorRotation(Vector3d a, Vector3d b)
{
    Vector3d v = a.cross(b);
    double s = sqrt(v(0) * v(0) + v(1) * v(1) + v(2) * v(2));
    double c = a.dot(b);

    Matrix3d I;
    I.setIdentity();

    Matrix3d V_skew;
    V_skew.setZero();

    V_skew(0,1) = -v(2);
    V_skew(0,2) = v(1);
    V_skew(1,0) = v(2);
    V_skew(1,2) = -v(0);
    V_skew(2,0) = -v(1);
    V_skew(2,1) = v(0);

    Matrix3d R = I + V_skew + ((1.0 / (1.0 + c)) * V_skew * V_skew);

    return R;
}

VectorXd Utils::QuaternionTwoVectors(Vector3d u, Vector3d v)
{
    Vector3d w = u.cross(v);
    VectorXd quat(4);
    quat(0) = 1.0 + u.dot(v);
    quat(1) = w(0);
    quat(2) = w(1);
    quat(3) = w(2);

    QuaternionNormalise(quat);

    return quat;
}

Vector3d Utils::DirectionCosineMatrixToEuler(Matrix3d TBL)
{
    Vector3d Euler;
//     double ctht = 0.0;
//     double cpsi = 0.0;
//     double cphi = 0.0;
//
//     // Extract Euler angles
//     if(fabs(TBL(0,2)) < 1.0)
//     {
//         Euler(1) = asin(-TBL(0,2));
//         ctht = cos(Euler(1));
//     }
//     else
//     {
//         Euler(1) = (PI/2.0) * sign_check(-TBL(0,2));
//         ctht = EPS;
//     }
//
//     //note: to avoid rare errors: #IND (quite NaN) of angles Euler[2]
//     //      and Euler[0] (acos, asin problem), cpsi and sphi are forced
//     //      to be always <= (1.-EPS)
//
//     cpsi = TBL(0,0)/ctht;
//     if(fabs(cpsi) >= 1.0)
//     {
//         cpsi = (1.0 - EPS) * sign_check(cpsi);
//     }
//
//     cphi = TBL(2,2)/ctht;
//     if(fabs(cphi) >= 1.0)
//     {
//         cphi = (1.0 - EPS) * sign_check(cphi);
//     }
//
//     Euler(2) = acos(cpsi) * sign_check(TBL(0,1));
//     Euler(0) = acos(cphi) * sign_check(TBL(1,2));

    Euler(0) = atan2(TBL(1,2),TBL(2,2));
    Euler(1) = -asin(TBL(0,2));
    Euler(2) = atan2(TBL(0,1),TBL(0,0));

    PiMinusPi(Euler(2));

    return Euler;
}
