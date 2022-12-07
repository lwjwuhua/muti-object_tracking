#include "association.h"
#include "kf.h"
#include <iostream>
#include<cmath>
#include"track_list_manage.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


PDA::PDA() {

	/* 因为传感器与预测值的残差符合卡方分布，所以lamd为卡方自由度 */
	lamd = 16;
	/* lamd对应的门概率PG */
	PG = 0.9997;
	PD = 1;

	H_ = MatrixXd(2, 4);
	H_ << 1, 0, 0, 0,
		0, 1, 0, 0;

	F_ = MatrixXd(4, 4);
	F_ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	float dt = 200 / 1000000.0;
	F_(0, 2) = dt;
	F_(1, 3) = dt;



	R_ = MatrixXd(2, 2);
	R_ << 0.0225, 0,
		0, 0.0225;



}

PDA::~PDA() {

}




void PDA::pda_distance( const VectorXd& X, const MatrixXd& P, const VectorXd& measurement_pack) {



	x_ = VectorXd(4);
	x_ << X;

	P_ = MatrixXd(4, 4);
	P_ << P;

	VectorXd z_pred = H_ * x_;
	y = measurement_pack - z_pred;
	MatrixXd Ht = H_.transpose();
	VectorXd yt = y.transpose();
	S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();

	distance = y.transpose() * Si * y;
	// 	distance = abs(distance);
	e__ = exp((-0.5) * distance);

	// cout << "ej =" << e__ << endl;
//	cout << "dis = " << distance << endl;

	/* cout << "y = " << y << endl;
	 cout << "si = " << Si << endl;*/

}

bool PDA:: pda_gating(const double& gate) {

	if ( distance < 16) {
		
		cout << "========落入波门，关联成功=========" << endl;
		return true;
	}
	else
	{
		return false;
	}
}

void PDA::pda_probability(const int& m, double& e, const double& e_, const VectorXd& z_) {

	cout << "第" << m << "个量测点的概率" << endl;

	double det = S(0, 0) * S(1, 1) - S(0, 1) * S(1, 0);
	//det = abs(det);
	double det_ = (2 * 3.14 * S(0, 0)) * (2 * 3.14 * S(1, 1)) - (2 * 3.14 * S(0, 1)) * (2 * 3.14 * S(1, 0));
	//det_ = abs(det_);

//	cout << "det = " << det << endl;
	V = 3.14 * lamd * sqrt(det); // 相关波门体积

//	cout << "V = " << V << endl;

	b = (m / V) * sqrt(det_) * (1 - PD * PG) / PD;

	//	cout << "b = " << b << endl;

	beta = e_ / (b + e);
	cout << "beta k----- =" << beta << endl;
}


void PDA:: Association(const int& id ,const int& m_, const VectorXd& Meas, const VectorXd& trac, const MatrixXd& P ) {


//	cout << m_ << "测量 data association=======================" << endl;

	
	  pda_distance(trac, P, Meas);
	isassocia =  pda_gating(distance);
	 
	 
}

void PDA::Findlatest_element(const int& n, const int& n_, const MatrixXd& AM) {


	double min = AM(0, 0);
	 indx = 0;
	 indy = 0;
	for (size_t i_ = 0; i_ < n; i_++)
	{
		for (size_t j_ = 0; j_ < n_; j_++)
		{
			if (AM(i_,j_)!= 0x3f3f3f3f)
			{
				if (AM(i_, j_) < min)
				{
					min = AM(i_, j_);
					indx = i_;
					indy = j_;
				}

			}
			
		}

	}

}