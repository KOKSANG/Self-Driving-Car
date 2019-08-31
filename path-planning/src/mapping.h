#ifndef MAPPING_H_
#define MAPPING_H_

#include <iostream>
#include <math.h>
#include <vector>
#include "spline.h"
#include "constants.h"

using std::vector;
using tk::spline;

class Mapping {
    public:
        vector<double> map_s;
        vector<double> map_x;
        vector<double> map_y;
        vector<double> map_dx;
        vector<double> map_dy;
        static spline spline_x;
        static spline spline_y;
        static spline spline_dx;
        static spline spline_dy;
        double interval;
        double track_length;

        Mapping(vector<double> waypoints_s, vector<double> waypoints_x, vector<double> waypoints_y,
            vector<double> waypoints_dx, vector<double> waypoints_dy, double separation,
            double length);
        virtual ~Mapping();

        vector<double> interpolate_points(int mode, vector<double> waypoints_s, vector<double> waypoints);
        vector<double> getXY(double s, double d);
        vector<double> getFrenet(double x, double y, double theta);

    private:
        double points;
    };

#endif