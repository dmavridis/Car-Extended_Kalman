#include "kalman_filter.h"

#include<iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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

// predict the state
	MatrixXd Ft_ = F_.transpose();

//	std::cout<<"F= "<<F_<<"\n";
	x_ = F_ * x_;
	P_ = F_ * P_ * Ft_ + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

// update the state by using Kalman Filter equations
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	MatrixXd Ht_ = H_.transpose();

	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd S = H_ * P_ * Ht_ + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht_;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	MatrixXd Ht_ = H_.transpose();
	VectorXd h1(3);

	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);

	float c = sqrt(px*px + py*py);
	if (c > 0.0001){
	h1 << c,
			atan2(py,px),
			(px*vx+py*vy)/c;
	} else{
		h1 << 0,0,0;
	}

	VectorXd z_pred = H_ * x_;
	VectorXd y = z - h1;


	MatrixXd S = H_ * P_ * Ht_ + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht_;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	P_ = (I - K * H_) * P_;


}
