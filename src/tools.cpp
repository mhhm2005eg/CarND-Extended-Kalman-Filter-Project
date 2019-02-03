#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

#if 0
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
}
#endif

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
	const vector<VectorXd> &ground_truth) {
    //cout<<"Tools::CalculateRMSE"<<01<<endl;
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if (estimations.size() != ground_truth.size()
		|| estimations.size() == 0) {
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	// accumulate squared residuals
	for (unsigned int i = 0; i < estimations.size(); ++i) {

		VectorXd residual = estimations[i] - ground_truth[i];

		// coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	// calculate the mean
	rmse = rmse / estimations.size();

	// calculate the squared root
	rmse = rmse.array().sqrt();

        //cout<<"Tools::CalculateRMSE"<<02<<endl;

	// return the result
	return rmse;
}

Eigen::VectorXd Tools::CalculateRMSE(const Eigen::VectorXd &estimations,
	const Eigen::VectorXd &ground_truth) {
    //cout<<"Tools::CalculateRMSE"<<01<<endl;
	Eigen::VectorXd rmse(1);
	//rmse << 0, 0, 0, 0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if (estimations.size() != ground_truth.size()
		|| estimations.size() == 0) {
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

        float t_rmse = 0;
	// accumulate squared residuals
	for (unsigned int i = 0; i < estimations.size(); ++i) {

		float residual = estimations[i] - ground_truth[i];

		// coefficient-wise multiplication
		residual = pow(residual, 2);
		t_rmse += residual;
	}

	// calculate the mean
	t_rmse = t_rmse / estimations.size();

	// calculate the squared root
	rmse << sqrt(t_rmse);

        //cout<<"Tools::CalculateRMSE"<<02<<endl;

	// return the result
	return rmse;
}
#if 0
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    //cout<<"Tools::CalculateJacobian"<<0<<endl;
	MatrixXd Hj(3, 4);
	// recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// pre-compute a set of terms to avoid repeated calculation
	float c1 = px * px + py * py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);

	// check division by zero
	if (fabs(c1) < 0.0001) {
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}

	// compute the Jacobian matrix
	Hj << (px / c2), (py / c2), 0, 0,
		-(py / c1), (px / c1), 0, 0,
		py*(vx*py - vy * px) / c3, px*(px*vy - py * vx) / c3, px / c2, py / c2;

        //cout<<"Tools::CalculateJacobian"<<9<<endl;

	return Hj;
}

#else
#define EPS 0.0001 // A very small number
#define EPS2 0.0000001
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  // Code from lectures quizes
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  MatrixXd Hj(3,4);
  // Deal with the special case problems
  if (fabs(px) < EPS and fabs(py) < EPS){
	  px = EPS;
	  py = EPS;
  }
  // Pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  // Check division by zero
  if(fabs(c1) < EPS2){
	c1 = EPS2;
  }
  float c2 = sqrt(c1);
  float c3 = (c1*c2);
  // Compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
       -(py/c1), (px/c1), 0, 0,
        py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
  return Hj;
}
#endif
VectorXd Tools::PolarToCartesian(const VectorXd& polar) {
	VectorXd cartesian = VectorXd(4);

	float rho = polar(0);
	float phi = polar(1);
	float drho = polar(2);

	float px = rho * cos(phi);
	float py = rho * sin(phi);
	float vx = drho * cos(phi);
	float vy = drho * sin(phi);

	cartesian << px, py, vx, vy;
	return cartesian;
}


VectorXd Tools::CartesianToPolar(const VectorXd& cartesian) {
  VectorXd polar = VectorXd(3);
  float THRESH = 0.0001;
  float px = cartesian(0);
  float py = cartesian(1);
  float vx = cartesian(2);
  float vy = cartesian(3);

  float rho = sqrt(px*px + py*py);
  float phi = atan2(py, px);//atan(py/ px);//
  float drho = 0;
  if (rho > THRESH) {
    drho = (px*vx + py*vy) / rho;
  }
  else{
      return polar;
  }

  polar << rho, phi, drho;
  return polar;
}