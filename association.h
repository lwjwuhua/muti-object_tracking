#pragma once
#ifndef PDA_H_
#define PDA_H_
#include "Dense"
#include "measurement_package.h"
#include"track_list_manage.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;

class PDA {
public:



	/**
	 * Constructor
	 */
	PDA();

	/**
	 * Destructor
	 */
	virtual ~PDA();


	void pda_distance( const VectorXd& X, const MatrixXd& P, const VectorXd& measurement_pack);
	void pda_probability(const int& m, double& e, const double& e_, const VectorXd& z_);
	bool pda_gating (const double& gate);
	int lamd;
	float PG;
	float PD;
	double beta;
	double beta0;
	double V;
	VectorXd y;
	VectorXd x_;

	VectorXd X_asso;

	void Association(const int& m,  const int& m_, const VectorXd& M, const VectorXd& X , const MatrixXd& P );
	Track_manager trac_mang;
	bool isassocia;

	void Findlatest_element(const int& n, const int& n_, const MatrixXd& AM);
	int indx ;
	int indy ;


	// state covariance matrix
	MatrixXd P_;
	MatrixXd P_asso ;
	// state transistion matrix
	MatrixXd F_;

	// process covariance matrix
	MatrixXd Q_;

	// measurement matrix
	MatrixXd H_;

	// measurement covariance matrix
	MatrixXd R_;

	MatrixXd S;
	double distance;
	double distance_;
	double e__;
	double b;

private:
	bool is_initialized_;
};




#endif  // PDA_H_

