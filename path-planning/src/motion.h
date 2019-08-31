#ifndef MOTION
#define MOTION

#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "helpers.h"
#include "constants.h"
#include "jmt.h"

using std::string;
using std::vector;
using Eigen::ArrayXd;
using Eigen::MatrixXd;
using namespace std;
using namespace Eigen;

double get_velocity(double pos_x1, double pos_x2, double pos_y1, double pos_y2, double t){
    double v_x = (pos_x1 - pos_x2)/ t;
    double v_y = (pos_y1 - pos_y2)/ t;

    return (v_x, v_y);
};

#endif