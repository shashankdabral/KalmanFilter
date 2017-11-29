#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
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
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ << 1,0,0,0,
	      0,1,0,0;

  Hj_      << 0,0,0,0,
	      0,0,0,0,
	      0,0,0,0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement

    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    ekf_.P_  = MatrixXd(4,4);
    ekf_.P_  << 1 0 0 0 ,
                0 1 0 0 ,
		0 0 1 0 ,
		0 0 0 1 ;
    cout << "EKF: 2 " << endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      x = rho.cos(phi), y = rho.sin(phi)
      */
      cout  << "Initializing for Radar data"<<endl;
      float theta = measurement_pack.raw_measurements_(1);
      float ro    = measurement_pack.raw_measurements_(0); 
      ekf_.x_  <<  ro*cos(theta),ro*sin(theta),0,0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state with position and 0 velocity
      */
      cout  << "Initializing for Laser data"<<endl;
      ekf_.x_ << measurement_pack.raw_measurements_(0),measurement_pack.raw_measurements_(1),0,0 ;
    }

    // done initializing, no need to predict or update
    previous_timestamp_  = measurement_pack.timestamp_;
    is_initialized_ = true;
    cout << "Initialization Complete " << endl;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  float noise_ax = 9;
  float noise_ay = 9;
  /* Create F Matrix */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000;
  previous_timestamp_  = measurement_pack.timestamp_;
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  ekf_.F_    = MatrixXd(4, 4);
  ekf_.F_      << 1,0,dt,0,
	          0,1,0,dt,
	          0,0,1,0,
	          0,0,0,1;

  /* Create Q Matrix */
  ekf_.Q_    = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
              0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
	      dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
	      0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
  cout << "Prior to predict" <<endl;
  cout <<" x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
  cout << "Calling Predict " << endl;
  ekf_.Predict();
  cout << "Predict Completed " << endl;
  cout << "After predict" <<endl;
  cout <<" x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.R_ =  R_radar_;
    /* Calculate Hj and store in ekf_.H_ */

    cout << "Calling Update for Radar " << endl;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    cout << "Radar Update Completed " << endl;
  } else {
    // Laser updates
    ekf_.H_ = H_laser_ ;
    ekf_.R_ = R_laser_ ;
    cout << "Calling Update for Laser " << endl;
    ekf_.Update(measurement_pack.raw_measurements_);
    cout << "Laser Update Completed " << endl;
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
