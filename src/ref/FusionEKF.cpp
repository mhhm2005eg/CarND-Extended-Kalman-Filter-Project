#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

#define RADAR_ACTIVE true
#define LIDAR_ACTIVE true

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  cout << "FusionEKF: IN" << endl;
  
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */

    int no_of_states = 4;
    int no_of_measurments = 2;
    //x = VectorXd(2);
    static VectorXd x(no_of_states);
    x << 0.01, 0.01, 0.0, 0.0;
    static MatrixXd I = MatrixXd::Identity(no_of_states, no_of_states);
    static MatrixXd P(no_of_states, no_of_states);// = I * 1000;
    static VectorXd u = VectorXd::Zero(no_of_states); // External forces (if we can measure it)
    static MatrixXd F = MatrixXd(no_of_states, no_of_states);
    static MatrixXd H = MatrixXd(no_of_measurments, no_of_states);
    static MatrixXd R = MatrixXd(no_of_measurments, no_of_measurments); // uncertainty within the measuement function
    static MatrixXd Q = MatrixXd(no_of_states, no_of_states); // uncertainty within the transition function

    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;
    P <<    1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
  ekf_.Init(x, P, F, H, R, Q, R_laser_, R_radar_);

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {

    cout << "FusionEKF: Out" << endl;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
    delta_timestamp_ = measurement_pack.timestamp_ - previous_timestamp_;
    delta_timestamp_ /= 1000000.0f;
    //cout<<delta_timestamp_<<"rr"<<endl;
    previous_timestamp_ = measurement_pack.timestamp_;
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    //ekf_.x_ = VectorXd(4);
    //ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
     //ekf_.init_R_Radar(0.09, 0.0009, 0.09); 
#if false    
     ekf_.UpdateEKF(measurement_pack.raw_measurements_);
#else     
          float rho = measurement_pack.raw_measurements_[0]; // range
	  float phi = measurement_pack.raw_measurements_[1]; // bearing
	  float rho_dot = measurement_pack.raw_measurements_[2]; // velocity of rho
	  // Coordinates convertion from polar to cartesian
	  float x = rho * cos(phi); 
	  float y = rho * sin(phi);
	  float vx = rho_dot * cos(phi);
	  float vy = rho_dot * sin(phi);
	  ekf_.x_ << x, y, vx , vy;
#endif    
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
     //ekf_.init_R_laser(0.0225, 0.0225);  
#if false        
     ekf_.Update(measurement_pack.raw_measurements_);
#else
     ekf_.x_<< measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1],0, 0;
#endif

    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    cout<<"End init"<<endl;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  ekf_.update_F(delta_timestamp_);
  ekf_.update_Q(delta_timestamp_, 25, 25);
  ekf_.Predict();


  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
#if RADAR_ACTIVE == true
     //ekf_.init_R_Radar(0.09, 0.0009, 0.09);
     ekf_.UpdateEKF(measurement_pack.raw_measurements_);
#endif
  } else {
    // TODO: Laser updates
#if LIDAR_ACTIVE == true
    ekf_.update_H(delta_timestamp_);
    //ekf_.init_R_laser(0.0225, 0.0225);      
    ekf_.Update(measurement_pack.raw_measurements_);
#endif

  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}

