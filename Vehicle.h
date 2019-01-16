#include "Eigen-3.3/Eigen/Eigen"
#include <iostream>

using namespace Eigen;

class Vehicle
{
    public:            

        Vehicle();
        ~Vehicle();

        void Linearise(VectorXd X, VectorXd U, MatrixXd& A, MatrixXd& B);
        void Integrate(VectorXd& X, VectorXd U, double DT);
        
        void Trim(VectorXd& X_trim, VectorXd& U_trim, double VelTrim, double AltTrim, double ThetaTrim, double PsiTrim, double LonTrim, double LatTrim);
        
    private:
    
        MatrixXd DirectionCosineMatrix(double phi, double theta, double psi);
        VectorXd StateRates(VectorXd X, VectorXd U);
        MatrixXd L1(double phi);
        MatrixXd L2(double theta);
        MatrixXd L3(double psi);
        
};