#include "Eigen-3.3/Eigen/Eigen"

using namespace Eigen;

class Utils
{
    public:            

        Utils();
        
        ~Utils();
        
        MatrixXd DirectionCosineMatrix(double phi, double theta, double psi);
        
        VectorXd EulerToQuaternion(double phi, double theta, double psi);
        
        VectorXd QuatToEuler(VectorXd quat);
        
        MatrixXd QuatToDirectionCosineMatrix(VectorXd quat);
        
        MatrixXd QuaternionRateMap(VectorXd quat);
        
        VectorXd QuaternionTranspose(VectorXd quat);
        
        void QuaternionNormalise(VectorXd& quat);
        
        VectorXd QuaternionProduct(VectorXd q, VectorXd p);
        
        void PiMinusPi(double& input);
                        
    private:

        MatrixXd L1(double phi);
        
        MatrixXd L2(double theta);
        
        MatrixXd L3(double psi);        
        
};