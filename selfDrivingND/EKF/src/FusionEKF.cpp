#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF()
{
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

  H_laser_ << 1, 0, 0, 0,
      0, 1, 0, 0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
  /**
   * Initialization
   */
  if (!is_initialized_)
  {

    // first measurement
    VectorXd x_init(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
      // Convert radar from polar to cartesian coordinates
      //         and initialize state.
      float rho = measurement_pack.raw_measurements_[0];
      float theta = measurement_pack.raw_measurements_[1];
      x_init << rho * cos(theta),
          rho * sin(theta),
          0,
          0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
      x_init << measurement_pack.raw_measurements_[0],
          measurement_pack.raw_measurements_[1],
          0,
          0;
    }

    MatrixXd P_init = Eigen::MatrixXd::Zero(4, 4);
    ekf_.Init(x_init, P_init, P_init, H_laser_, R_laser_, P_init);

    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    return;
  }

  /**
   * Prediction
   */

  float dT = (measurement_pack.timestamp_ - previous_timestamp_) * 1e-6;
  float dT2 = dT * dT;
  float dT3 = dT2 * dT;
  float dT4 = dT3 * dT;
  float sigma_ax = 9.0;
  float sigma_ay = 9.0;

  ekf_.F_ << 1, 0, dT, 0,
      0, 1, 0, dT,
      0, 0, 1, 0,
      0, 0, 0, 1;

  float var_ax = pow(sigma_ax, 2);
  float var_ay = pow(sigma_ay, 2);

  ekf_.Q_ << dT4 * var_ax / 4, 0, dT3 * var_ax / 2, 0,
      0, dT4 * var_ay / 4, 0, dT3 * var_ay / 2,
      dT3 * var_ax / 2, 0, dT2 * var_ax, 0,
      0, dT3 * var_ay / 2, 0, dT2 * var_ay;

  ekf_.Predict();

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  else
  {

    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }
  previous_timestamp_ = measurement_pack.timestamp_;

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
