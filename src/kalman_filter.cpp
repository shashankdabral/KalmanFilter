#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_  = F_ *x_;
  MatrixXd Ft  = F_.transpose();
  P_           = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /*
   * Linear Kalman filter equations for Lidar *
  */
   VectorXd z_pred = H_ * x_;
   VectorXd y = z - z_pred; /* Calculate error */
   MatrixXd Ht = H_.transpose();
   MatrixXd S = H_ * P_ * Ht + R_;
   MatrixXd Si = S.inverse();
   MatrixXd PHt = P_ * Ht;
   MatrixXd K  = PHt * Si;
//   cout <<"R  = "<< R_ <<endl;

   // New estimate
   x_  = x_ + (K*y);
   long x_size = x_.size();
   MatrixXd I = MatrixXd::Identity(x_size,x_size);
   P_ = (I - K*H_)* P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    *  Non linear update using Taylor (Jacobian) for Radar *
  */
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  Eigen::MatrixXd Hj_;
  Hj_ = MatrixXd(3, 4);
  Tools tools;

  Hj_ = tools.CalculateJacobian(x_);

  
  VectorXd z_pred (3);
  float  ro     = z(0);
  float  theta  = z(1);
  float  ro_dot = z(2);
  float  c4     = sqrt(px*px + py*py);
  float  c5     = atan2(py,px);
  while (c5 > 3.14) {
    c5 = c5 - 2*3.14;
  }
  while (c5 < -3.14) {
    c5 = c5 + 2*3.14;
  }

  float  c6     = px*vx + py*vy;
  z_pred        << c4,c5,(c6/c5);
  VectorXd  y = z - z_pred; /* Calculate error */
  MatrixXd Ht = Hj_.transpose();
  MatrixXd S = Hj_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K  = PHt * Si;

  // New estimate
  x_  = x_ + (K*y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size,x_size);
  P_ = (I - K*Hj_)* P_;

}
