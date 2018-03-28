#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.3;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.


  previous_time_step_ = 0.0;

  // initializing state dimension
  n_x_ = 5;

  // initializing augmented state dimension
  n_aug_ = 7;

  // initializing state vector
  x_ = VectorXd(n_x_);
  x_.fill(0.0);

  // initializing covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_.fill(0.0);

  // initializing sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);
  Xsig_pred_.fill(0.0);

  //initializing weights vector
  weights_ = VectorXd(2*n_aug_+1);

  // the current NIS for radar
  NIS_radar_ = 0.0;

  // the current NIS for laser
  NIS_laser_ = 0.0;

  // initializing lambda value
  lambda_ = 3 - n_aug_;

  is_initialized_ = false;

  x_ << 1, 1, 1, 1, 0.1;

  P_ << 0.15, 0   , 0, 0, 0,
        0,    0.15, 0, 0, 0,
        0,    0   , 1, 0, 0,
        0,    0   , 0, 1, 0,
        0,    0   , 0, 0, 1;
}


UKF::~UKF() {}


/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{

  double dt;

  if ((meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) ||
      (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_))
  {
    if (!is_initialized_)
    {

      // initial timestamp
      previous_time_step_ = meas_package.timestamp_;

      if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
      {
        x_(0) = meas_package.raw_measurements_(0);
        x_(1) = meas_package.raw_measurements_(1);
      }
      else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
      {
        float ro = meas_package.raw_measurements_(0);
        float phi = meas_package.raw_measurements_(1);
        float ro_dot = meas_package.raw_measurements_(2);
        x_(0) = ro     * cos(phi);
        x_(1) = ro     * sin(phi);
      }

      // done initializing, no need to predict or update
      is_initialized_ = true;

      return;
    }

    dt = (meas_package.timestamp_ - previous_time_step_) / 1000000.0;	//dt - expressed in seconds
    previous_time_step_ = meas_package.timestamp_;

    // ==========================
    //  PREDICTION STEP
    // ==========================
    Prediction(dt);


    // ==========================
    //  UPDATING STEP
    // ==========================
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      // ==============
      // Radar updates
      // ==============
      UpdateRadar(meas_package);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      // ==============
      // Laser updates
      // ==============
      UpdateLidar(meas_package);
    }
  }
}


/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t)
{

  MatrixXd X_sigma_pts;
  X_sigma_pts = MatrixXd(n_x_, 2*n_aug_+1);

  // ===========================
  //  Generate the Sigma Points
  // ===========================
  GenerateSigmaPoints(&X_sigma_pts);

  // =============================
  //  Predicting Sigma Points
  // =============================
  SigmaPointPrediction(&X_sigma_pts, delta_t);

  // ====================================================
  //  Predicting the State vector and State Co-variance
  // ====================================================
  PredictMeanAndCovariance();

}


/**
 * Generates the Sigma Points as the part of the Prediction step
 * @param Xsig_out pointer to matrix which will store the sigma points;
 */
void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out)
{
  MatrixXd Xsig_aug, P_aug, L;
  VectorXd x_aug, sigma_pts;

  n_aug_ = 7;
  // ===============================
  // Create Augmented Mean State
  // ===============================
  x_aug = VectorXd(n_aug_);
  x_aug.head(n_x_) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // =========================================
  // Create Augmented State Co-variance Matrix
  // =========================================
  P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5, 5) = P_;
  P_aug(5, 5) = std_a_*std_a_;
  P_aug(6, 6) = std_yawdd_*std_yawdd_;

  // =========================
  // Create square root matrix
  // =========================
  L = P_aug.llt().matrixL();

  Xsig_aug  = MatrixXd(n_aug_, 2*n_aug_+1);
  Xsig_aug.fill(0);
  Xsig_aug.col(0) = x_aug;


  // =========================
  // Generate the Sigma Points
  // =========================
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  *Xsig_out = Xsig_aug;
}


/**
   * Predicts the State vector at time k+1 based on the Generated Sigma Points
   * The function implements a non-linear Process model
   * @param Xsig_out : pointer to matrix which contains the generated sigma points;
   * @param delta_t : difference between the current time and the previosu time;
   */
void UKF::SigmaPointPrediction(MatrixXd* X_sigma_pts, double delta_t) {
  double px, py, v, yaw, yaw_d, nu_a, nu_yaw_d;

  //predicted state values
  double px_p, py_p, v_p, yaw_p, yaw_d_p;

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    //extract values
    px       = (*X_sigma_pts)(0, i);
    py       = (*X_sigma_pts)(1, i);
    v        = (*X_sigma_pts)(2, i);
    yaw      = (*X_sigma_pts)(3, i);
    yaw_d    = (*X_sigma_pts)(4, i);
    nu_a     = (*X_sigma_pts)(5, i);
    nu_yaw_d = (*X_sigma_pts)(6, i);

    //avoid division by zero
    if (fabs(yaw_d) > 0.001)
    {
      px_p = px + v / yaw_d * (sin(yaw + yaw_d * delta_t) - sin(yaw));
      py_p = py + v / yaw_d * (cos(yaw) - cos(yaw + yaw_d * delta_t));
    }
    else
    {
      px_p = px + v * delta_t * cos(yaw);
      py_p = py + v * delta_t * sin(yaw);
    }

    v_p     = v;
    yaw_p   = yaw + yaw_d * delta_t;
    yaw_d_p = yaw_d;

    //add noise
    px_p    = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p    = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p     = v_p + nu_a * delta_t;
    yaw_p   = yaw_p + 0.5 * nu_yaw_d * delta_t * delta_t;
    yaw_d_p = yaw_d_p + nu_yaw_d * delta_t;

    // write predicted sigma point
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yaw_d_p;
  }
}


/**
   * Predicts the State Mean Vector and the State Co-Variance Matrix
   */
void UKF::PredictMeanAndCovariance()
{

  VectorXd x_diff;
  x_diff = VectorXd(n_x_);

  // ====================
  //  Setting up weights
  // ====================
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i = 1; i < 2*n_aug_+1 ; i++)
  {
    weights_(i) = (0.5)/(lambda_+n_aug_);
  }


  x_.fill(0.0);
  // ==================================
  // Predicting the State Mean Vector
  // ==================================
  for (int i = 0; i < 2*n_aug_+1 ; i++)
  {
    x_ = x_ + weights_(i)*Xsig_pred_.col(i);
  }

  P_.fill(0.0);
  // ====================================
  // Predicting State Co-variance Matrix
  // ====================================
  for (int i = 1; i < 2*n_aug_+1 ; i++)
  {
    x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI)
    {
      x_diff(3)-=2.*M_PI;
    }
    while (x_diff(3)<- M_PI)
    {
      x_diff(3)+=2.*M_PI;
    }

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }

}


/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  VectorXd x_diff, z_diff, z_pred;
  MatrixXd S, R, Tc, K_gain;
  double p_x , p_y;
  //extract measurement as VectorXd
  VectorXd z = meas_package.raw_measurements_;

  //set measurement dimension, lidar can measure p_x and p_y
  int n_z = 2;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // ===============================================
  // Transform sigma points into measurement space
  // ===============================================
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // extract values
    p_x = Xsig_pred_(0, i);
    p_y = Xsig_pred_(1, i);

    // measurement model
    Zsig(0, i) = p_x;
    Zsig(1, i) = p_y;
  }

  //mean predicted measurement
  z_pred = VectorXd(n_z);
  z_pred.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    z_diff = Zsig.col(i) - z_pred;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  R = MatrixXd(n_z, n_z);

  R << std_laspx_*std_laspx_, 0                    ,
      0                     , std_laspy_*std_laspy_;

  S = S + R;

  /***********************
  * UKF Update for Lidar
  ************************/
  //create matrix for cross correlation Tc
  Tc = MatrixXd(n_x_, n_z);
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    z_diff = Zsig.col(i) - z_pred;

    // state difference
    x_diff = Xsig_pred_.col(i) - x_;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  K_gain = Tc * S.inverse();

  //residual
  z_diff = z - z_pred;

  //calculate NIS
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;

  // =======================================
  // Update state mean and covariance matrix
  // =======================================
  x_ = x_ + K_gain * z_diff;
  P_ = P_ - K_gain * S * K_gain.transpose();

}


/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  VectorXd z_pred, z_diff;
  MatrixXd Zsig, S, R, Tc, K_gain;
  double p_x, p_y, v, yaw, v1, v2;
  int n_z;

  n_z = 3;
  Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // ===============================================
  // Conversion from state space to measurement space
  // ===============================================
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    p_x = Xsig_pred_(0,i);
    p_y = Xsig_pred_(1,i);
    v   = Xsig_pred_(2,i);
    yaw = Xsig_pred_(3,i);

    v1 = cos(yaw)*v;
    v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  // ==========================
  // Mean predicted measurement
  // ==========================
  z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++)
  {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // ==============================================
  // Calculating the innovation covariance matrix S
  // ==============================================
  S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    //residual
    z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI)
    {
      z_diff(1)-=2.*M_PI;
    }
    while (z_diff(1)<-M_PI)
    {
      z_diff(1)+=2.*M_PI;
    }
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // =======================================
  // Add measurement noise covariance matrix
  // =======================================
  R = MatrixXd(n_z, n_z);
  R << std_radr_*std_radr_ ,                       0 ,                     0 ,
      0                    , std_radphi_*std_radphi_ ,                     0 ,
      0                    ,                       0 , std_radrd_*std_radrd_ ;

  S = S + R;


  /***********************
  * UKF Update for Radar
  ************************/
  Tc = MatrixXd(n_x_, n_z);
  // ==================================
  // Calculate cross correlation matrix
  // ==================================
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    //residual
    z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI)
    {
      z_diff(1)-=2.*M_PI;
    }
    while (z_diff(1)<-M_PI)
    {
      z_diff(1)+=2.*M_PI;
    }

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    //angle normalization
    while (x_diff(3)> M_PI)
    {
      x_diff(3)-=2.*M_PI;
    }
    while (x_diff(3)<-M_PI)
    {
      x_diff(3)+=2.*M_PI;
    }

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // ==================================
  // Calculating the Kalman gain K_gain
  // ==================================
  K_gain = Tc * S.inverse();

  VectorXd z = VectorXd(n_z);
  z = meas_package.raw_measurements_;
  //residual
  z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI)
  {
    z_diff(1)-=2.*M_PI;
  }
  while (z_diff(1)<-M_PI)
  {
    z_diff(1)+=2.*M_PI;
  }

  //calculate NIS
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;

  // =================================================
  // Updating state mean vector and covariance matrix
  // =================================================
  x_ = x_ + K_gain * z_diff;
  P_ = P_ - K_gain * S * K_gain.transpose();

}

