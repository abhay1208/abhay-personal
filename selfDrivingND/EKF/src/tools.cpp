#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{

  VectorXd rmse = Eigen::VectorXd::Zero(4);
  int N = estimations.size();

  for (int i = 0; i < N; ++i)
  {
    VectorXd current_error = ground_truth[i] - estimations[i];
    current_error = current_error.array() * current_error.array();
    rmse += current_error;
  }

  rmse /= N;
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state)
{

  MatrixXd J = Eigen::MatrixXd::Zero(3, 4);
  float px = x_state[0];
  float py = x_state[1];
  float vx = x_state[2];
  float vy = x_state[3];
  float mag = px * px + py * py;

  if (std::fabs(mag < .0001))
  {
    std::cout << "Calculuate Jacobian Error: Division by Zero" << std::endl;
    return J;
  }

  J << px / sqrt(mag), py / sqrt(mag), 0, 0,
      -py / mag, px / mag, 0, 0,
      py * (vx * py - vy * px) / pow(mag, 1.5), px * (vy * px - vx * py) / pow(mag, 1.5), px / sqrt(mag), py / sqrt(mag);

  return J;
}

VectorXd Tools::MapStateToLidarOutput(const VectorXd &x_state)
{
  VectorXd hx(3);
  float px = x_state[0];
  float py = x_state[1];
  float vx = x_state[2];
  float vy = x_state[3];
  float mag = px * px + py * py;
  float theta = atan2(py, px);

  hx << sqrt(mag),
      theta,
      (px * vx + py * vy) / sqrt(mag);
  return hx;
}
