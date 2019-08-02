#include "Eigen-3.3/Eigen/Eigen"

using namespace Eigen;

class Utils
{
    public:

        Utils();
        ~Utils();

        Matrix3d DirectionCosineMatrix(double phi, double theta, double psi);
        VectorXd EulerToQuaternion(double phi, double theta, double psi);
        Vector3d QuatToEuler(VectorXd quat);
        Matrix3d QuatToDirectionCosineMatrix(VectorXd quat);
        MatrixXd QuaternionRateMap(VectorXd quat);
        VectorXd QuaternionTranspose(VectorXd quat);
        VectorXd QuaternionChangeHand(VectorXd quat);
        void QuaternionNormalise(VectorXd& quat);
        VectorXd QuaternionProduct(VectorXd q, VectorXd p);
        void PiMinusPi(double& input);
        Matrix3d VectorRotation(Vector3d a, Vector3d b);
        Vector3d DirectionCosineMatrixToEuler(Matrix3d TBL);
        VectorXd QuaternionTwoVectors(Vector3d u, Vector3d v);

    private:

        Matrix3d L1(double phi);
        Matrix3d L2(double theta);
        Matrix3d L3(double psi);

};
