#include "kalman_filter.h"
#define PI_ 3.1415
#define PI2_ 6.183


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
    // lecture 5.7, but noise is 0...
    //x = F * x + u;
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  MatrixXd Ht = H_.transpose();
    
  VectorXd y = z - H_ * x_;;
  
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  
  MatrixXd P_Ht = P_ * Ht;
  
  MatrixXd K = P_Ht * Si;

  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    
  float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  float phi = atan2(x_(1), x_(0));
  float rho_prime = (x_(0)*x_(2) + x_(1)*x_(3))/rho;;
  
  VectorXd z_hat(3);
  z_hat << rho, phi, rho_prime;
  VectorXd y = z - z_hat;
  
  while(y(1) > PI_) y(1)-=PI2_;
  while(y(1) < (-1.*PI_)) y(1)+=PI2_;
  
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd P_Ht = P_ * Ht;
  MatrixXd K = P_Ht * Si;

  //new estimate
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());
  
  P_ = (I - K * H_) * P_;

}
