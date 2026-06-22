#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

using std::cout;
using std::endl;
using Eigen::MatrixXd;
using Eigen::VectorXd;

UKF::UKF() {
  use_laser_ = true;
  use_radar_ = true;

  x_ = VectorXd(5);

  P_ = MatrixXd(5, 5);

  std_a_ = 3;
  std_yawdd_ = 0.8465;

  std_laspx_ = 0.15;
  std_laspy_ = 0.15;

  std_radr_ = 0.3;
  std_radphi_ = 0.03;
  std_radrd_ = 0.3;

  is_initialized_ = false;
  time_us_ = 0;

  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;

  P_.setIdentity();

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; i++)
    weights_(i) = 0.5 / (lambda_ + n_aug_);
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  if (!is_initialized_) {
    x_.setZero();

    if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
    } 
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
      double rho = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      double rhod = meas_package.raw_measurements_(2);
      x_(0) = rho * cos(phi);
      x_(1) = rho * sin(phi);
      double vx = rhod * cos(phi);
      double vy = rhod * sin(phi);
      x_(2) = std::sqrt(vx * vx + vy * vy);
    }

    P_.setIdentity();
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  // stub: future snapshots will add Prediction and Update calls
}

void UKF::Prediction(double delta_t) {
  // stub: not yet implemented
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  // stub: not yet implemented
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  // stub: not yet implemented
}
