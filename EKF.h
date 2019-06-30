#include "Eigen-3.3/Eigen/Eigen"

using namespace Eigen;

class EKF
{
    public:            

        EKF(VectorXd X);
        
        ~EKF();

        void RunEKF(VectorXd imu_data, VectorXd gps_data, double baro_data, double DT);        
       
        VectorXd GetX();
        
        MatrixXd GetP();
                       
    private:
        
        MatrixXd StateTransitionMatrix(VectorXd QUATNL, VectorXd WBECB, VectorXd FSPCB);
        
        VectorXd m_X;
        
        VectorXd m_eX;
        
        MatrixXd m_P;
        
        MatrixXd m_Q;
        
        MatrixXd m_R;
        
        MatrixXd m_H;
       
};