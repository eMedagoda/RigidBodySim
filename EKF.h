#include "Eigen-3.3/Eigen/Eigen"

using namespace Eigen;

class EKF
{
    public:

        EKF(VectorXd X, VectorXd acc_static_bias, VectorXd gyr_static_bias);
        ~EKF();

        void RunEKF(VectorXd imu_data, VectorXd gps_data, double baro_data, double DT);
        VectorXd GetX();
        MatrixXd GetP();
        Vector3d GetPos();
        Vector3d GetVel();
        Vector3d GetEuler();
        VectorXd GetPVA();
    private:

        MatrixXd StateTransitionMatrix(VectorXd QUATNL, VectorXd WBECB, VectorXd FSPCB);
        VectorXd m_X;
        MatrixXd m_P;
        MatrixXd m_Q;
        MatrixXd m_R;
        MatrixXd m_H;
        VectorXd m_acc_static_bias;
        VectorXd m_gyr_static_bias;
};
