#include "EKF.h"
#include "Utils.h"
#include "VehicleParameters.h"
#include <cmath>
#include <iostream>

EKF::EKF(VectorXd X)
{
    // initial state vector
    m_X = VectorXd(16);
    m_X = X;  
    
    // initial state error vector
    m_eX = VectorXd(16);
    m_eX.setZero();

    // initial covariance matrix
    m_P = MatrixXd(16, 16);
    m_P.setZero();
    
    m_P(0,0) = p_pos;
    m_P(1,1) = p_pos;
    m_P(2,2) = p_pos;
    
    m_P(3,3) = p_vel;
    m_P(4,4) = p_vel;
    m_P(5,5) = p_vel;
    
    m_P(6,6) = p_att;
    m_P(7,7) = p_att;
    m_P(8,8) = p_att;
    m_P(9,9) = p_att;
    
    m_P(10,10) = p_bia;
    m_P(11,11) = p_bia;
    m_P(12,12) = p_bia;
    
    m_P(13,13) = p_bia;
    m_P(14,14) = p_bia;
    m_P(15,15) = p_bia;

    // set procress noise covariance
    m_Q = MatrixXd(16, 16);
    m_Q.setZero();
    
    m_Q(0,0) = q_pos;
    m_Q(1,1) = q_pos;
    m_Q(2,2) = q_alt;
    
    m_Q(3,3) = q_vel;
    m_Q(4,4) = q_vel;
    m_Q(5,5) = q_dalt;
    
    m_Q(6,6) = q_att;
    m_Q(7,7) = q_att;
    m_Q(8,8) = q_att;
    m_Q(9,9) = q_att;
    
    m_Q(10,10) = q_bia;
    m_Q(11,11) = q_bia;
    m_Q(12,12) = q_bia;
    
    m_Q(13,13) = q_bia;
    m_Q(14,14) = q_bia;
    m_Q(15,15) = q_bia;
    
    // set measurement noise covariance
    m_R = MatrixXd(11, 11);
    m_R.setZero();
    
    m_R(0,0)   = r_att;
    m_R(1,1)   = r_att;
    m_R(2,2)   = r_att;
    m_R(3,3)   = r_att;
    
    m_R(4,4)   = r_pos;
    m_R(5,5)   = r_pos;
    m_R(6,6)   = r_alt;
    
    m_R(7,7)   = r_vel;
    m_R(8,8)   = r_vel;
    m_R(9,9)   = r_vel;
    m_R(10,10) = r_dalt;
    
    // set measurement model
    m_H = MatrixXd(11, 16);
    m_H.setZero();
    
    m_H(0, 6)  = 1.0;    // q0
    m_H(1, 7)  = 1.0;    // q1
    m_H(2, 8)  = 1.0;    // q2
    m_H(3, 9)  = 1.0;    // q3
    m_H(4, 0)  = 1.0;    // X
    m_H(5, 1)  = 1.0;    // Y
    m_H(6, 2)  = 1.0;    // Z
    m_H(7, 3)  = 1.0;    // Vx
    m_H(8, 4)  = 1.0;    // Vy
    m_H(9, 5)  = 1.0;    // Vz (GPS)
    m_H(10, 5) = 0.0;    // Vz (baro)

}

EKF::~EKF()
{
}

void EKF::RunEKF(VectorXd imu_data, VectorXd gps_data, double baro_data, double DT)
{
    Utils Utils;    
  
    VectorXd specific_forces(3);
    specific_forces(0) = imu_data(0);
    specific_forces(1) = imu_data(1);
    specific_forces(2) = imu_data(2);
    
    VectorXd gyro_rates(3);
    gyro_rates(0) = imu_data(3);
    gyro_rates(1) = imu_data(4);
    gyro_rates(2) = imu_data(5);
    
    VectorXd mag_attitude(3);
    mag_attitude(0) = imu_data(6);
    mag_attitude(1) = imu_data(7);
    mag_attitude(2) = imu_data(8);
   
    static double baro_data_old = 0.0;
    static double baro_alt_rate_old = 0.0;
    
    double baro_alt_rate_raw = (baro_data - baro_data_old) / DT;
    double baro_alt_rate = 0.05 * baro_alt_rate_raw + 0.95 * baro_alt_rate_old;
    baro_data_old = baro_data;
    baro_alt_rate_old = baro_alt_rate;
    
    // convert attitude measurement from magnetometer into quaternion vector
    VectorXd quat_measurement = Utils.EulerToQuaternion(mag_attitude(0), mag_attitude(1), mag_attitude(2));
    
    VectorXd Z(11);
    Z.setZero();
    
    // attitude measurements
    Z(0) = quat_measurement(0);
    Z(1) = quat_measurement(1);
    Z(2) = quat_measurement(2);
    Z(3) = quat_measurement(3);
    
    // position measurements
    Z(4) = gps_data(0);
    Z(5) = gps_data(1);
    Z(6) = -baro_data;

    // velocity measurements
    Z(7) = gps_data(3);
    Z(8) = gps_data(4);
    Z(9) = gps_data(5);
    Z(10) = -baro_alt_rate;   
        
    // debias raw acclerometer measurements
    specific_forces(0) = specific_forces(0) - m_X(10);
    specific_forces(1) = specific_forces(1) - m_X(11);
    specific_forces(2) = specific_forces(2) - m_X(12);
    
    // debias raw gyro measurements
    gyro_rates(0) =  gyro_rates(0) - m_X(13);
    gyro_rates(1) =  gyro_rates(1) - m_X(14);
    gyro_rates(2) =  gyro_rates(2) - m_X(15);    
   
    // perform position, velocity and attitude (quaternion) prediction X_pred
    
    //Attitude prediction
    VectorXd quat(4);
    quat(0) = m_X(6);
    quat(1) = m_X(7);
    quat(2) = m_X(8);
    quat(3) = m_X(9);
    
    MatrixXd Q(4,3);
    Q = Utils.QuaternionRateMap(quat);    
    quat += 0.5 * Q * gyro_rates * DT; // quaternion prediction
    MatrixXd C_bn = Utils.QuatToDirectionCosineMatrix(quat);    
    
    VectorXd gravity(3);
    gravity.setZero();
    gravity(2) = GRAVITY;
    
    VectorXd acceleration = C_bn.transpose() * specific_forces + gravity;    
    
    m_X(3) += acceleration(0) * DT;
    m_X(4) += acceleration(1) * DT;
    m_X(5) += acceleration(2) * DT;
    m_X(0) += m_X(3) * DT;
    m_X(1) += m_X(4) * DT;
    m_X(2) += m_X(5) * DT;
    m_X(6) = quat(0);
    m_X(7) = quat(1);
    m_X(8) = quat(2);
    m_X(9) = quat(3);
            
    // construct measurment prediction vector
    VectorXd Z_prediction(11);  
    Z_prediction(0) = m_X(6);
    Z_prediction(1) = m_X(7);
    Z_prediction(2) = m_X(8);
    Z_prediction(3) = m_X(9);
    Z_prediction(4) = m_X(0);
    Z_prediction(5) = m_X(1);
    Z_prediction(6) = m_X(2);
    Z_prediction(7) = m_X(3);
    Z_prediction(8) = m_X(4);
    Z_prediction(9) = m_X(5);
    Z_prediction(10) = m_X(5);
    
    // formulate measurement error vector e_Z = Z_pred - Z
    VectorXd e_Z = Z_prediction - Z;
        
    // --------------------------------------------------------------------
    
    // initial m_eX = 0.0
    m_eX.setZero();
    
    // formulate state transition matrix F
    MatrixXd F = StateTransitionMatrix(quat, gyro_rates, specific_forces);
    
    // perform prediction step
    MatrixXd PHI(16,16);
    PHI.setZero();
    PHI.Identity(16,16);
    PHI += F * DT;

	m_eX = PHI * m_eX;
	m_P = (PHI * m_P * PHI.transpose()) + m_Q;
    
    // perform update step   
	
    // Computing filter gain
    MatrixXd S = m_H * m_P * m_H.transpose() + m_R;
	MatrixXd K = m_P * m_H.transpose() * S.inverse(); 
	
	// State Correction
	m_eX = m_eX + K * (e_Z - m_H * m_eX);

    MatrixXd I(16,16);
    I.setZero();
	I.Identity(16,16);
    
	// Covariance Correction
	m_P = (I - K * m_H) * m_P;
    
    // --------------------------------------------------------------------
  
    // position correction
    m_X(0) -= m_eX(0);
    m_X(1) -= m_eX(1);
    m_X(2) -= m_eX(2);
    
    // velocity correction
    m_X(3) -= m_eX(3);
    m_X(4) -= m_eX(4);
    m_X(5) -= m_eX(5);
    
    // attitude correction
    VectorXd q_unit(4);
    q_unit << 1.0, 0.0, 0.0, 0.0;
    
    VectorXd equat(4);
    equat << m_eX(6), m_eX(7), m_eX(8), m_eX(9);
    
    VectorXd quat_transpose = Utils.QuaternionTranspose(quat);    
    
    VectorXd equat_quat_transpose = Utils.QuaternionProduct(equat, quat_transpose);    
    VectorXd quat_correction = q_unit + equat_quat_transpose;
    Utils.QuaternionNormalise(quat_correction);    
    VectorXd quat_correction_transpose = Utils.QuaternionTranspose(quat_correction);
    
    quat = Utils.QuaternionProduct(quat_correction_transpose, quat); // apply attitude correction to prediction
    
    m_X(6) = quat(0);
    m_X(7) = quat(1);
    m_X(8) = quat(2);
    m_X(9) = quat(3);
    
    // bias correction
    m_X(10) += m_eX(10);
    m_X(11) += m_eX(11);
    m_X(12) += m_eX(12);
    m_X(13) += m_eX(13);
    m_X(14) += m_eX(14);
    m_X(15) += m_eX(15);    
    
}

MatrixXd EKF::StateTransitionMatrix(VectorXd QUATNL, VectorXd WBECB, VectorXd FSPCB)
{	
	Utils Utils;
    
    // Create local matrices
	MatrixXd F_GPS(16,16); // state transition matrix (GPS nav)
	MatrixXd F_SKS(3,3); // specific force skew symetric matrix
	MatrixXd Q_ASYM(3,4); // quaternion matrix
	MatrixXd FQ_RES(3,4);
    VectorXd FSPCL(3); // [EM] specific forces in local frame (m/s^2)
	
	F_GPS.setZero();
	F_SKS.setZero();
	Q_ASYM.setZero();
	FQ_RES.setZero();
    FSPCL.setZero(); // [EM] added to match derivation
    
    MatrixXd TNL = Utils.QuatToDirectionCosineMatrix(QUATNL);
    
    FSPCL = TNL.transpose() * FSPCB; // [EM] body to local transformation of specific forces

	// unpack inputs
	double p = WBECB(0,0);
	double q = WBECB(1,0);
	double r = WBECB(2,0);
	double q0 = QUATNL(0);
	double q1 = QUATNL(1);
	double q2 = QUATNL(2);
	double q3 = QUATNL(3);    
   
	double f1 = FSPCL(0);
	double f2 = FSPCL(1);
	double f3 = FSPCL(2);

	double t11 = TNL(0,0);
	double t12 = TNL(0,1);
	double t13 = TNL(0,2);
	double t21 = TNL(1,0);
	double t22 = TNL(1,1);
	double t23 = TNL(1,2);
	double t31 = TNL(2,0);
	double t32 = TNL(2,1);
	double t33 = TNL(2,2);	

	// prelim calcs
	F_SKS(0,1) = f3;
	F_SKS(0,2) = -f2;
	F_SKS(1,0) = -f3;
	F_SKS(1,2) = f1;
	F_SKS(2,0) = f2;
	F_SKS(2,1) = -f1;

	Q_ASYM(0,0) = q1;
	Q_ASYM(0,1) = -q0;
	Q_ASYM(0,2) = q3;
	Q_ASYM(0,3) = -q2;
	Q_ASYM(1,0) = q2;
	Q_ASYM(1,1) = -q3;
	Q_ASYM(1,2) = -q0;
	Q_ASYM(1,3) = q1;
	Q_ASYM(2,0) = q3;
	Q_ASYM(2,1) = q2;
	Q_ASYM(2,2) = -q1;
	Q_ASYM(2,3) = -q0;

	FQ_RES = F_SKS * Q_ASYM;
	FQ_RES = FQ_RES * -2.0;

	// populate state transition matrix
	F_GPS(0,3) = 1.0;
	F_GPS(1,4) = 1.0;
	F_GPS(2,5) = 1.0;
	F_GPS(3,6) = FQ_RES(0,0);
	F_GPS(3,7) = FQ_RES(0,1);
	F_GPS(3,8) = FQ_RES(0,2);
	F_GPS(3,9) = FQ_RES(0,3);
	F_GPS(4,6) = FQ_RES(1,0);
	F_GPS(4,7) = FQ_RES(1,1);
	F_GPS(4,8) = FQ_RES(1,2);
	F_GPS(4,9) = FQ_RES(1,3);
	F_GPS(5,6) = FQ_RES(2,0);
	F_GPS(5,7) = FQ_RES(2,1);
	F_GPS(5,8) = FQ_RES(2,2);
	F_GPS(5,9) = FQ_RES(2,3);
	F_GPS(3,10) = t11;
	F_GPS(3,11) = t21;
	F_GPS(3,12) = t31;
	F_GPS(4,10) = t12;
	F_GPS(4,11) = t22;
	F_GPS(4,12) = t32;
	F_GPS(5,10) = t13;
	F_GPS(5,11) = t23;
	F_GPS(5,12) = t33;
	F_GPS(6,7) = -p*0.5;
	F_GPS(6,8) = -q*0.5;
	F_GPS(6,9) = -r*0.5;
	F_GPS(7,6) = p*0.5;
	F_GPS(7,8) = r*0.5;	
	F_GPS(7,9) = -q*0.5;
	F_GPS(8,6) = q*0.5;
	F_GPS(8,7) = -r*0.5;
	F_GPS(8,9) = p*0.5;
	F_GPS(9,6) = r*0.5;
	F_GPS(9,7) = q*0.5;
	F_GPS(9,8) = -p*0.5;
	F_GPS(6,13) = -q1*0.5;
	F_GPS(6,14) = -q2*0.5;
	F_GPS(6,15) = -q3*0.5;
	F_GPS(7,13) = q0*0.5;
	F_GPS(7,14) = -q3*0.5;
	F_GPS(7,15) = -q2*0.5;
	F_GPS(8,13) = q3*0.5;
	F_GPS(8,14) = q0*0.5;
	F_GPS(8,15) = -q1*0.5;
	F_GPS(9,13) = -q2*0.5;
	F_GPS(9,14) = q1*0.5;
	F_GPS(9,15) = q0*0.5;

	return F_GPS;
}

VectorXd EKF::GetX()
{
    return m_X;
}

MatrixXd EKF::GetP()
{
    return m_P;
}