#include "kf.h"
#include<iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

KalmanFilter::KalmanFilter() {

	F_ = MatrixXd(4, 4);
	F_ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	float dt = 500 / 1000000.0;
	F_(0, 2) = dt;
	F_(1, 3) = dt;

	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;

	Q_ = MatrixXd(4, 4);
	Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
		0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
		dt_3 / 2 * noise_ax, 0, dt_2* noise_ax, 0,
		0, dt_3 / 2 * noise_ay, 0, dt_2* noise_ay;

	R_ = MatrixXd(2, 2);
	R_ << 0.0225, 0,
		0, 0.0225;

	H_ = MatrixXd(2, 4);
	H_ << 1, 0, 0, 0,
		0, 1, 0, 0;

	// set the acceleration noise components
	noise_ax = 5;
	noise_ay = 5;

}

KalmanFilter::~KalmanFilter() {
}



void KalmanFilter::Predict(const int& T, const VectorXd& X, const MatrixXd& P) {

	
	x_ = VectorXd(4);
	x_ << X;

	p_ = MatrixXd(4, 4);
	p_ << P;

	/* cout <<  x_ << endl;
	 cout << p_ << endl;*/


	pre_x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	pre_P_ = F_ * p_ * Ft + Q_;

	cout << pre_x_ << endl;
	cout << pre_P_ << endl;

}

void KalmanFilter::Update1(const VectorXd& z, const MatrixXd& p_pre, const VectorXd& x_pre) {

	VectorXd z_pred = H_ * x_pre;

	VectorXd y = z - z_pred;

	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * p_pre * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = p_pre * Ht;
	MatrixXd K = PHt * Si;

	//new estimate

	x_ = x_pre+ (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);

	p_ = (I - K * H_) * p_pre;

	// p_c = pre_P_ - K * S * K.transpose();

	// p_r = K * (vs - v * v.transpose()) * K.transpose();



}





void  KalmanFilter::Update2(const double& bta0, const MatrixXd& vs, const VectorXd& v) {

	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * pre_P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = pre_P_ * Ht;
	MatrixXd K = PHt * Si;

	x_ = pre_x_ + K * v;

	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);

	// p_c = (I - K * H_) * pre_P_;
	p_c = pre_P_ - K * S * K.transpose();

	p_r = K * (vs - v * v.transpose()) * K.transpose();

	p_ = bta0 * pre_P_ + (1 - bta0) * p_c + p_r;

}