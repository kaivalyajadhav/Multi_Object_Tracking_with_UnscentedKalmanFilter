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

  double dt = (meas_package.timestamp_ - time_us_) / 1e6;
  time_us_ = meas_package.timestamp_;

  Prediction(dt);

  // stub: update steps will be added in future snapshots
}

void UKF::Prediction(double delta_t) {
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(n_x_) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.setZero();
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  MatrixXd L = P_aug.llt().matrixL();

  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.col(0) = x_aug;
  double sqrt_lambda_n_aug = std::sqrt(lambda_ + n_aug_);
  for (int i = 0; i < n_aug_; ++i) {
    Xsig_aug.col(i + 1)          = x_aug + sqrt_lambda_n_aug * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt_lambda_n_aug * L.col(i);
  }

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    double p_x = Xsig_aug(0, i);
    double p_y = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    double px_p, py_p;

    if (fabs(yawd) > 1e-3) {
      px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (-cos(yaw + yawd * delta_t) + cos(yaw));
    } else {
      px_p = p_x + v * delta_t * cos(yaw);
      py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    px_p += 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p += 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p  += nu_a * delta_t;

    yaw_p  += 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p += nu_yawdd * delta_t;

    while (yaw_p > M_PI) yaw_p -= 2.*M_PI;
    while (yaw_p < -M_PI) yaw_p += 2.*M_PI;

    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }

  x_.setZero();
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  P_.setZero();
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

    P_ += weights_(i) * x_diff * x_diff.transpose();
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  // stub: not yet implemented
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  // stub: not yet implemented
}
