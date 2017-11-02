#include "kalman_filter.h"

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
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();

  //new estimate
  x_ = x_ + K * y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  float rho = sqrt(pow(px, 2) + pow(py, 2));
  float phi = atan(py/px);
  float rhodot = (px*vx + py*vy)/rho;
  VectorXd hx = VectorXd(3);
  hx << rho, phi, rhodot;
  //Calculate difference between measured and calculated sensor data
  VectorXd y = z - hx;

  H_ = CalculateJacobian(x_);

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();

  //new estimate
  x_ = x_ + K * y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

MatrixXd KalmanFilter::CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //TODO: YOUR CODE HERE 

  //check division by zero
  if(px == 0 && py == 0) {
      Hj << 0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0;
  } else {
  //compute the Jacobian matrix
      float px2 = pow(px, 2);
      float py2 = pow(py, 2);
      float dpdpx = px/sqrt(px2 + py2);
      float dpdpy = py/sqrt(px2 + py2);
      float drdpx = -py/(px2 + py2);
      float drdpy = px/(px2 + py2);
      float dpddpx = py*(vx*py - vy*px)/(sqrt(pow(px2 + py2, 3)));
      float dpddpy = px*(vy*px - vx*py)/(sqrt(pow(px2 + py2, 3)));
      float dpddvx = dpdpx;
      float dpddvy = dpdpy;
        Hj << dpdpx, dpdpy, 0, 0,
              drdpx, drdpy, 0, 0,
              dpddpx, dpddpy, dpddvx, dpddvy;
  }
  return Hj;
}
