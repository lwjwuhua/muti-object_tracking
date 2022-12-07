#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Dense"

class MeasurementPackage {
public:

    enum SensorType {
        LASER, RADAR
    } sensor_type_;

    Eigen::VectorXd raw_measurements_;

    Eigen::VectorXd mea_;

    float a_;
    int64_t timestamp_;

};

class Track_list_manag
{
public:

    int id;
    int slot;
   Eigen :: VectorXd sig_track_state_list;
   Eigen :: MatrixXd sig_track_cov_list;
 
private:

};


#endif  // MEASUREMENT_PACKAGE_H_#pragma once
#pragma once
