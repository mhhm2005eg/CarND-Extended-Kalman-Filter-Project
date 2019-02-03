#include "tools.h"
#include "kalman_filter.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

using std::cout;
using std::endl;
/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {
    cout << "KalmanFilter: IN" << endl;
}

KalmanFilter::~KalmanFilter() {
    cout << "KalmanFilter: Out" << endl;
}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in, 
                        MatrixXd &R_laser_in, MatrixXd &R_radar_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  R_laser_ = R_laser_in;
  R_radar_ = R_radar_in;        
  u_ = VectorXd::Zero(4);

  I = MatrixXd::Identity(4, 4);
  tools = Tools();

}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    
  MatrixXd &R_ = R_laser_;

  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

#if 0
void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
    	// KF Prediction step
	x_ = F_ * x_ + u_;
	P_ = F_ * P_*F_.transpose() + Q_;

}


void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
        MatrixXd &R_ = R_laser_;
                
    	VectorXd y(2);
	MatrixXd S(2, 4);
	MatrixXd K(4, 4);
	y = z - H_ * x_;
	S = H_ * P_*H_.transpose() + R_;
	K = P_ * H_.transpose()*S.inverse();
	// new state

	x_ = x_ + K * y;
	P_ = (I - K * H_)*P_;

}
#endif

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
    
    MatrixXd &R_ = R_radar_;

    MatrixXd Hj;
    VectorXd z_ = tools.PolarToCartesian(z);
    Hj = tools.CalculateJacobian(x_);

    VectorXd y(3);
    MatrixXd S(3, 3);
    MatrixXd K(4, 4);
    
    // Recalculate x object state to rho, theta, rho_dot coordinates
    VectorXd h = VectorXd(3); // h(x_)
    h = tools.CartesianToPolar(x_);
    y = z -h; //- Hj*z_;//
    /*if (y[1] > M_PI){
        y[1] -= 2*M_PI;
    }
    else if (y[1] < -M_PI){
        y[1] += 2*M_PI;
    }*/
    y[1] = atan2(tan(y[1]) , 1);
    S = Hj * P_*Hj.transpose() + R_;
    K = P_ * Hj.transpose()*S.inverse();
    // new state    
    x_ = x_ + (K * y);
    P_ = (I - K * Hj)*P_;

}
void KalmanFilter::update_Q(double delta_t, double variance_ax, double variance_ay) {
	// Q = G * a * a.T * G.T
	double delta_t4 = pow(delta_t, 4) / 4;
	double delta_t3 = pow(delta_t, 3) / 2;
	double delta_t2 = pow(delta_t, 2);
	Eigen::MatrixXd ret(4, 4);
	ret << delta_t4 * variance_ax, 0, delta_t3* variance_ax, 0,
		0, delta_t4* variance_ay, 0, delta_t3* variance_ay,
		delta_t3* variance_ax, 0, delta_t2* variance_ax, 0,
		0, delta_t3* variance_ay, 0, delta_t2* variance_ay;
	Q_ = ret;
}
#if 0
void KalmanFilter::update_Q(double dt, double noise_ax, double noise_ay) {
	// Q = G * a * a.T * G.T
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;
    Eigen::MatrixXd ret = MatrixXd(4, 4);
        ret <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
                   0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
                   dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
                   0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
	Q_ = ret;
}
#endif
void KalmanFilter::init_R_laser(double variance_px, double variance_py) {
	Eigen::MatrixXd ret(2, 2);
	ret << variance_px, 0, 0,variance_py;
	R_ = ret;
}
void KalmanFilter::init_R_Radar(double variance_p, double variance_Q, double variance_Qd) {
	Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(3, 3);
        Eigen::VectorXd D(3);
        D << variance_p, variance_Q,  variance_Qd;
	ret = D.asDiagonal().toDenseMatrix();
	R_ = ret;
}
void KalmanFilter::update_F(double delta_t) {
	Eigen::MatrixXd ret = I;
        ret(0, 2) = delta_t;
        ret(1, 3) = delta_t;
	F_ = ret;
}
void KalmanFilter::update_H(double delta_t) {
	Eigen::MatrixXd ret = MatrixXd(2,4);
        ret <<1, 0, delta_t,0, 
              0, 1, 0      , delta_t;
	H_ = ret;
}