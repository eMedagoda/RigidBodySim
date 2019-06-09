#include "Vehicle.h"
#include "Utils.h"
#include "VehicleParameters.h"
#include <cmath>

Vehicle::Vehicle()
{
}

Vehicle::~Vehicle()
{
}

void Vehicle::Linearise(VectorXd X, VectorXd U, MatrixXd& A, MatrixXd& B)
{
    VectorXd Xdot = StateRates(X, U);

    double dX = 1e-6;

    for (int i = 0; i < X.size(); i++)
    {
        VectorXd Xpert = X;

        Xpert(i) = Xpert(i) + dX;

        VectorXd Xdot_pert = StateRates(Xpert, U);

        for (int j = 0; j < X.size(); j++)
        {
            A(j,i) = (Xdot_pert(j) - Xdot(j))/dX;
        }
    }

    for (int i = 0; i < U.size(); i++)
    {
        VectorXd Upert = U;

        Upert(i) = Upert(i) + dX;

        VectorXd Xdot_pert = StateRates(X, Upert);

        for (int j = 0; j < X.size(); j++)
        {
            B(j,i) = (Xdot_pert(j) - Xdot(j))/dX;
        }
    }
}

void Vehicle::Integrate(VectorXd& X, VectorXd U, double DT)
{
    Utils Utils;
    
    VectorXd Xdot1 = StateRates(X, U);	
    VectorXd An = Xdot1 * DT;

    VectorXd Xdot2 = StateRates(X + 0.5*An, U);
    VectorXd Bn = Xdot2 * DT;

    VectorXd Xdot3 = StateRates(X + 0.5*Bn, U);
    VectorXd Cn = Xdot3 * DT;

    VectorXd Xdot4 = StateRates(X + Cn, U);
    VectorXd Dn = Xdot4 * DT;

    // Runge-Kutta 4th order integration
    X = X + (An + 2.0*Bn + 2.0*Cn + Dn)/6.0;
    
    Utils.PiMinusPi(X(8));
}

void Vehicle::Trim(VectorXd& X0, 
                   VectorXd& U0, 
                   double VelTrim, 
                   double AltTrim, 
                   double ThetaTrim, 
                   double PsiTrim, 
                   double LonTrim, 
                   double LatTrim)
{
    // initial trim vector
    VectorXd XTrim(3); 
    XTrim << 1.0, -1.0, 1.0; // Fx, Fz, My

    // convergence tolerance
    double Tol = 1e-10;

    // perturbation
    double dXTrim = 1e-6;

    // initial state vector
    VectorXd X(15);
    X.setZero();

    // initial control vector
    VectorXd U(6);
    U.setZero();

    // initialise jacobian
    MatrixXd J(3,3);
    J.setZero();

    // intial error flag
    double Err = 1;

    // initial counter
    int n = 1;
    
    VectorXd Xdot(12);
    Xdot.setZero();
    
    VectorXd XTrimDot(3);
    XTrimDot.setZero();
    
    VectorXd XTrim_new(3);
    XTrim_new.setZero();
    
    VectorXd XTrimDotPert(3);
    XTrimDot.setZero();
    
    double Pert = 0.0;

    while (Err > Tol)
    {
        // set trim states
        X(0) = VelTrim*cos(ThetaTrim);
        X(2) = VelTrim*sin(ThetaTrim);
        X(7) = ThetaTrim;
        X(11) = -AltTrim;
        
        // set trim controls
        U(0) = XTrim(0);
        U(2) = XTrim(1);
        U(4) = XTrim(2);
        
        Xdot = StateRates(X,U);        

        XTrimDot << Xdot(0), Xdot(2), Xdot(4);
            
        // perturb controls (Fx)
        Pert = XTrim(0) + dXTrim;
        
        U(0) = Pert;
        U(2) = XTrim(1);
        U(4) = XTrim(2);
        
        Xdot = StateRates(X,U);
        
        XTrimDotPert << Xdot(0), Xdot(2), Xdot(4);
        
        J.block<3,1>(0,0) = (XTrimDotPert - XTrimDot)/dXTrim;    
     
        // perturb controls (Fz)
        Pert = XTrim(1) + dXTrim;
        
        U(0) = XTrim(0);
        U(2) = Pert;
        U(4) = XTrim(2);
        
        Xdot = StateRates(X,U);
        
        XTrimDotPert << Xdot(0), Xdot(2), Xdot(4);
        
        J.block<3,1>(0,1) = (XTrimDotPert - XTrimDot)/dXTrim;        
              
        // perturb controls (My)
        Pert = XTrim(2) + dXTrim;
        
        U(0) = XTrim(0);
        U(2) = XTrim(1);
        U(4) = Pert;
        
        Xdot = StateRates(X,U);
        
        XTrimDotPert << Xdot(0), Xdot(2), Xdot(4);
        
        J.block<3,1>(0,2) = (XTrimDotPert - XTrimDot)/dXTrim;
        
        // check for convergence
        VectorXd XTrim_new = XTrim - J.inverse() * XTrimDot;
        
        // check for convergence
        Err = sqrt((XTrim_new(0) - XTrim(0))*(XTrim_new(0) - XTrim(0)) + (XTrim_new(1) - XTrim(1))*(XTrim_new(1) - XTrim(1)) + (XTrim_new(2) - XTrim(2))*(XTrim_new(2) - XTrim(2)));
        
        XTrim = XTrim_new;
        
        n = n + 1;
    }        
    
    X0.setZero();    
    U0.setZero();    
   
    // set trim states
    X0(0) = VelTrim*cos(ThetaTrim);
    X0(2) = VelTrim*sin(ThetaTrim);
    X0(7) = ThetaTrim;
    X0(8) = PsiTrim;
    X0(11) = -AltTrim;
    X0(12) = LonTrim;
    X0(13) = LatTrim;
    X0(14) = AltTrim;
    
    // set trim controls
    U0(0) = XTrim(0);
    U0(2) = XTrim(1);
    U0(4) = XTrim(2);
         
}

VectorXd Vehicle::StateRates(VectorXd X, VectorXd U)
{
    VectorXd Xdot(15);
    Xdot.setZero();
    
    Utils Utils;
    
    // calculate body axis rotational inertias
    double c0 = Ixx*Izz-Ixz*Ixz;
    double c1 = Izz/c0;
    double c2 = Ixz/c0;
    double c3 = c2*(Ixx-Iyy+Izz);
    double c4 = c1*(Iyy-Izz)-c2*Ixz;
    double c5 = 1.0/Iyy;
    double c6 = c5*Ixz;
    double c7 = c5*(Izz-Ixx);
    double c8 = Ixx/c0;
    double c9 = c8*(Ixx-Iyy)+c2*Ixz;
    
    double u     = X(0);
    double v     = X(1);
    double w     = X(2);
    double p     = X(3);
    double q     = X(4);
    double r     = X(5);
    double phi   = X(6);
    double theta = X(7);
    double psi   = X(8);
    
    double lon   = X(12);
    double lat   = X(13);
    double alt   = X(14);

    // course lateral drag model
    double ydrag  = 5.0 * v * v;;
    
    if (v < 0.0)
    {
        ydrag = -ydrag;
    }
    
    // force and moment inputs
    double F_x = U(0);
    double F_y = U(1) - ydrag;
    double F_z = U(2);
    double M_x = U(3);
    double M_y = U(4);
    double M_z = U(5);

    double udot = -GRAVITY*sin(theta)          + r*v - q*w + F_x/MASS;
    double vdot =  GRAVITY*sin(phi)*cos(theta) - r*u + p*w + F_y/MASS;
    double wdot =  GRAVITY*cos(phi)*cos(theta) + q*u - p*v + F_z/MASS;

    double pdot = c3*p*q + c4*q*r + c1*M_x + c2*M_z;
    double qdot = c7*p*r - c6*(p*p - r*r)  + c5*M_y;
    double rdot = c9*p*q - c3*q*r + c2*M_x + c8*M_z;
    
    double phidot   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    double thetadot = q*cos(phi) - r*sin(phi);
    double psidot   = q*sin(phi)*(1.0/cos(theta)) + r*cos(phi)*(1.0/cos(theta));

    // navigation to body
    MatrixXd C_bn = Utils.DirectionCosineMatrix(phi, theta, psi);

    // position rates (navigation frame)
    VectorXd SpeedVec(3);
    SpeedVec << u, v, w;
    VectorXd PosRates = C_bn.transpose() * SpeedVec;    
    
    // geodetic rates
    double N_RE = R_EA/sqrt(1.0 - ECC*ECC*sin(lat)*sin(lat)); // prime vertical radius of curvature    
    double M_RE = (R_EA * (1.0 - ECC*ECC))/sqrt((1.0 - ECC*ECC*sin(lat)*sin(lat))*(1.0 - ECC*ECC*sin(lat)*sin(lat))*(1.0 - ECC*ECC*sin(lat)*sin(lat))); // meridian radius of curvature
    double londot = PosRates(1)/((N_RE + alt)*cos(lat)); // longitude rate
    double latdot = PosRates(0)/(M_RE + alt); // latitude rate
    double altdot = -PosRates(2); // altitude rate    
    
    // populate state rate vector
    Xdot << udot, vdot, wdot, pdot, qdot, rdot, phidot, thetadot, psidot, PosRates(0), PosRates(1), PosRates(2), londot, latdot, altdot;
    
    return Xdot;

}
