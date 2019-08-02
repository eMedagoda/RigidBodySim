#include "Eigen-3.3/Eigen/Eigen"

using namespace Eigen;

class Sensors
{
    public:

        Sensors();
        ~Sensors();

        VectorXd IMU(VectorXd X, VectorXd U);
        VectorXd GPS(VectorXd X);
        double Barometer(VectorXd X);

    private:

};
