#include "tools.h"

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

void Tools::normalizeTheta(double &yaw)
{
   while (yaw > M_PI)
      yaw -= 2. * M_PI;
   while (yaw < -M_PI)
      yaw += 2. * M_PI;
}
