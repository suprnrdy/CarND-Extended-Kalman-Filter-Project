#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

    // TODO: YOUR CODE HERE

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  // ... your code here
  if(estimations.size() == 0 || estimations.size() != ground_truth.size()) return rmse;
    
  //accumulate squared residuals
  VectorXd sum(4);
  sum << 0, 0, 0, 0;
  for(int i=0; i < estimations.size(); ++i){
    // ... your code here
    VectorXd mult = (estimations[i] - ground_truth[i]);
    VectorXd sqr = mult.array() * mult.array();
    sum = sum + sqr;
  }

  //calculate the mean
  // ... your code here
  VectorXd mean = sum.array()/estimations.size();

  //calculate the squared root
  // ... your code here
  rmse = mean.array().sqrt();

  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
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
