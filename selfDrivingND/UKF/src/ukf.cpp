#include "ukf.h"
#include <iostream>

#define debug(x) << #x << x << "\n"
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF()
{
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 3;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  // Radar measurement
  n_z_radar_ = 3;
  R_radar_ = Eigen::Vector3d(pow(std_radr_, 2), pow(std_radphi_, 2), pow(std_radrd_, 2)).asDiagonal();

  // Laser measurement
  n_z_laser_ = 2;
  R_laser_ = Eigen::Vector2d(pow(std_laspx_, 2), pow(std_laspy_, 2)).asDiagonal();

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // initial state vector
  x_ = VectorXd::Zero(n_x_);

  // initial covariance matrix to identity matrix
  P_ = Eigen::MatrixXd::Identity(n_x_, n_x_);

  // initial sigma points
  Xsig_pred_ = MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);

  // Weights
  // Weights of sigma points
  weights_ = Eigen::VectorXd::Zero(2 * n_aug_ + 1);
  weights_[0] = lambda_ / (lambda_ + n_aug_);
  weights_.tail(2 * n_aug_) = 0.5 / (lambda_ + n_aug_) * Eigen::VectorXd::Ones(2 * n_aug_);
}

UKF::~UKF() {}

void UKF::initializeState(MeasurementPackage meas_package)
{
  // Set all states to zero and fill them up with the measurement
  x_.fill(0);

  //Update time
  time_us_ = meas_package.timestamp_;

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
    x_[0] = meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]);
    x_[1] = meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]);
    x_[2] = meas_package.raw_measurements_[2];
    x_[3] = meas_package.raw_measurements_[1];
    is_initialized_ = true;
  }
  if (meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
    x_[0] = meas_package.raw_measurements_[0];
    x_[1] = meas_package.raw_measurements_[1];
    is_initialized_ = true;
  }
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
  if (!is_initialized_)
  {
    initializeState(meas_package);
    return;
  }

  double dT = 1e-6 * (meas_package.timestamp_ - time_us_);
  Prediction(dT);

  if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && use_radar_)
  {
    UpdateRadar(meas_package);
  }

  if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && use_laser_)
  {
    UpdateLidar(meas_package);
  }

  time_us_ = meas_package.timestamp_;
  std::cout << "%%%%%%%%%%%%%%%%% Next Iteration %%%%%%%%%%%%%%\n";
}

void UKF::Prediction(double dT)
{
  // Generate sigma points for the current state augmented
  Eigen::MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  generateAugmentedSigmaPoints(Xsig_aug);

  // Propagte sigma points using model

  predictSigmaPoints(Xsig_aug, dT);

  // Get mean and covar for the predicted sigma points. This will give us mean and covar for prediction step
  predictMeanAndCovar();
}

void UKF::UpdateLidar(MeasurementPackage meas_package)
{
  /* 
  * As state to lidar measurement is linear, we can get away with simpler KF equations.
  * We don't have to use UKF for lidar measurements
  */

  // Measurement Vector
  // [px, py]
  Eigen::Vector2d z(meas_package.raw_measurements_[0],
                    meas_package.raw_measurements_[1]);

  // Measurement Matrix
  Eigen::MatrixXd H_lidar = Eigen::MatrixXd::Zero(n_z_laser_, n_x_);
  H_lidar(0, 0) = 1;
  H_lidar(1, 1) = 1;
  VectorXd z_pred = H_lidar * x_;
  Eigen::VectorXd y = z - z_pred;
  Eigen::MatrixXd S = H_lidar * P_ * H_lidar.transpose() + R_laser_;
  Eigen::MatrixXd K = P_ * H_lidar.transpose() * S.inverse();
  Eigen::MatrixXd I = MatrixXd::Identity(n_x_, n_x_);

  // Update state and covariance matrix with innovation
  x_ += K * y;
  P_ -= K * H_lidar * P_;

  double nis = UKF::getNIS(z, z_pred, S);
  std::cout << "NIS For Lidar Measurement: " << nis << std::endl;
}

void UKF::UpdateRadar(MeasurementPackage meas_package)
{
  /**
   * As measurement function is non-linear, we will use predicted sigma points
   * and map them to measurement space to get measurement space sigma points
   * We then calculate mean and covariance for measurement using UKF approach 
   * similar to how we did it for the process model.
   */

  // Measurement Vector
  // z is [pho, theta, pho_dot]
  Eigen::Vector3d z(meas_package.raw_measurements_[0],
                    meas_package.raw_measurements_[1],
                    meas_package.raw_measurements_[2]);

  // create matrix for sigma points in measurement space
  Eigen::MatrixXd Zsig(n_z_radar_, 2 * n_aug_ + 1);

  // mean predicted measurement
  Eigen::VectorXd z_pred(n_z_radar_);

  // measurement covariance matrix S
  Eigen::MatrixXd S = MatrixXd::Zero(n_z_radar_, n_z_radar_);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double psi = Xsig_pred_(3, i);
    Zsig(0, i) = sqrt(pow(px, 2) + pow(py, 2));
    Zsig(1, i) = atan2(py, px);
    Zsig(2, i) = (px * cos(psi) * v + py * sin(psi) * v) / Zsig(0, i);
  }

  // calculate mean predicted measurement
  for (int i = 0; i < n_z_radar_; ++i)
  {
    Eigen::MatrixXd temp = weights_.array() * Zsig.row(i).transpose().array();
    z_pred(i) = temp.sum();
  }

  // calculate measurement covariance matrix S
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    Eigen::MatrixXd temp1 = Zsig.col(i) - z_pred;
    S += weights_(i) * temp1 * temp1.transpose();
  }

  S += R_radar_;

  updateState(z, Zsig, z_pred, S);
}

void UKF::generateAugmentedSigmaPoints(Eigen::MatrixXd &Xsig_aug)
{
  // Augment the state vector with noise state
  Eigen::VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(n_x_) = x_;
  x_aug.tail(2) = Eigen::VectorXd::Zero(2);

  // create augmented state covariance
  Eigen::MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  Eigen::MatrixXd Q = MatrixXd(n_aug_ - n_x_, n_aug_ - n_x_);
  Q << std_a_ * std_a_, 0,
      0, std_yawdd_ * std_yawdd_;
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug.bottomRightCorner(n_aug_ - n_x_, n_aug_ - n_x_) = Q;

  // create square root matrix
  Eigen::MatrixXd P_aug_root = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 1; i <= n_aug_; ++i)
  {
    Xsig_aug.col(i) = x_aug + sqrt(n_aug_ + lambda_) * P_aug_root.col(i - 1);
    Xsig_aug.col(n_aug_ + i) = x_aug - sqrt(n_aug_ + lambda_) * P_aug_root.col(i - 1);
  }
}

void UKF::predictSigmaPoints(const Eigen::MatrixXd &x_sig_aug,
                             double dT)
{
  // std::cout << "dT: " << dT << std::endl;
  for (size_t i = 0; i < x_sig_aug.cols(); ++i)
  {
    Eigen::VectorXd x = x_sig_aug.col(i);
    double px = x[0];
    double py = x[1];
    double v = x[2];
    double yaw = x[3];
    double yaw_dot = x[4];
    double sigma_a = x[5];
    double sigma_yaw_ddot = x[6]; //It's actuall yaw_rate_derivative

    if (std::fabs(yaw_dot) < 1e-3)
    {
      Xsig_pred_(0, i) = px + v * cos(yaw) * dT +
                         0.5 * dT * dT * sigma_a * cos(yaw); //px
      Xsig_pred_(1, i) = py + v * sin(yaw) * dT +
                         0.5 * dT * dT * sigma_a * sin(yaw); //py
    }
    else
    {
      Xsig_pred_(0, i) = px + (v / yaw_dot) * (sin(yaw + yaw_dot * dT) - sin(yaw)) +
                         0.5 * dT * dT * sigma_a * cos(yaw); //px
      Xsig_pred_(1, i) = py + (v / yaw_dot) * (-cos(yaw + yaw_dot * dT) + cos(yaw)) +
                         0.5 * dT * dT * sigma_a * sin(yaw); //py
    }

    Xsig_pred_(2, i) = v + sigma_a * dT;                                    //radial velocity
    Xsig_pred_(3, i) = yaw + yaw_dot * dT + 0.5 * dT * dT * sigma_yaw_ddot; //yaw
    Xsig_pred_(4, i) = yaw_dot + sigma_yaw_ddot * dT;                       // yaw rate
  }
}

void UKF::predictMeanAndCovar()
{
  // predict state mean
  for (int i = 0; i < n_x_; ++i)
  {
    Eigen::MatrixXd temp = weights_.array() * Xsig_pred_.row(i).transpose().array();
    x_(i) = temp.sum();
  }

  P_.fill(0); // reset process covariance matrix

  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // Yaw normalization
    Tools::normalizeTheta(x_diff(3));

    P_ += weights_(i) * x_diff * x_diff.transpose();
  }
}

void UKF::updateState(const Eigen::VectorXd &z,
                      const Eigen::MatrixXd &Zsig,
                      const Eigen::VectorXd &z_pred,
                      const Eigen::MatrixXd &S)
{
  /**
   * For UKF, we calculate innovation factor and update the state and covariance 
   * matrix
  */

  // create matrix for cross correlation Tc
  Eigen::MatrixXd Tc = MatrixXd::Zero(n_x_, z_pred.rows());

  // calculate cross correlation matrix

  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    Eigen::VectorXd xdiff = Xsig_pred_.col(i) - x_;
    Tools::normalizeTheta(xdiff(3));
    Eigen::VectorXd zdiff = Zsig.col(i) - z_pred;
    Tools::normalizeTheta(zdiff(1));
    Tc += weights_(i) * xdiff * zdiff.transpose();
  }

  // calculate Kalman gain K;
  Eigen::MatrixXd K = Tc * S.inverse();

  // update state mean and covariance matrix

  x_ += K * (z - z_pred);
  P_ -= K * S * K.transpose();

  double nis = UKF::getNIS(z, z_pred, S);
  std::cout << "NIS For Radar Measurement: " << nis << std::endl;
}

double UKF::getNIS(const Eigen::VectorXd &z,
                   const Eigen::VectorXd &z_pred,
                   const Eigen::MatrixXd &S)
{
  /**
   * Get Normalized innovation squared for the filter. This is an indicator of how well Kalman Filter is
   * working and how reasonable you noise parameters are
   */

  return (z - z_pred).transpose() * S.inverse() * (z - z_pred);
}
