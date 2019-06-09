#include "Eigen-3.3/Eigen/Eigen"

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
        
        VectorXd StateRates(VectorXd X, VectorXd U);        
        
};
