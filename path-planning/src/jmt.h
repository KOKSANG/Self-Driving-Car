#ifndef JMT_H
#define JMT_H

#include <vector>
#include "Eigen-3.3/Eigen/Dense"


using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

vector<double> JMT(vector<double> start, vector<double> end, double T){
    double a_0 = start[0];
    double a_1 = start[1];
    double a_2 = start[2]/2;
    
    MatrixXd A = MatrixXd(3, 3);
    A << pow(T, 3), pow(T, 4), pow(T, 5),
        3*pow(T, 2), 4*pow(T, 3), 5*pow(T, 4),
        6*T, 12*pow(T,2), 20*pow(T, 3);

    MatrixXd B = MatrixXd(3,1);
    B << end[0] - (start[0] + start[1]*T + .5*start[2]*pow(T, 2)),
        end[1] - (start[1] + start[2]*T),
        end[2] - start[2];
    
    MatrixXd Ai = A.inverse();
    MatrixXd delta = Ai*B;
    
    vector<double> results;
    results = {start[0], start[1], .5*start[2]};

    for(int i = 0; i < delta.size(); ++i) {
        results.push_back(delta.data()[i]);
    }
    
    return results;
}

vector<vector<double>> get_jmt(vector<double> initial, vector<double> final, double T){
    double si = initial[0];
    double si_d = initial[1];
    double si_dd = initial[2];
    double di = initial[3];
    double di_d = initial[4];
    double di_dd = initial[5];
    double sf = final[0];
    double sf_d = final[1];
    double sf_dd = final[2];
    double df = final[3];
    double df_d = final[4];
    double df_dd = final[5];

    vector<double> start_s = {si, si_d, si_dd};
    vector<double> end_s = {sf, sf_d, sf_dd};
    vector<double> start_d = {di, di_d, di_dd};
    vector<double> end_d = {df, df_d, df_dd};

    vector<double> jmt_s = JMT(start_s, end_s, T);
    vector<double> jmt_d = JMT(start_d, end_d, T);

    vector<vector<double>> output = {jmt_s, jmt_d};

    return output;
}

vector<double> polysolver(double x1, double x2, double x3, vector<double> alpha, double t){
    double t_2 = pow(t, 2);
    double t_3 = pow(t, 3);
    double t_4 = pow(t, 4);
    double t_5 = pow(t, 5);
    double xt = x1 + x2*t + 0.5*x3*t_2 + alpha[3]*t_3 + alpha[4]* t_4+ alpha[5]*t_5;
    double xt_d = x2 + x3*t + 3*alpha[3]*t_2 + 4*alpha[4]*t_3 + 5*alpha[5]*t_4;
    double xt_dd = x3 + 6*alpha[3]*t + 12*alpha[4]*t_2 + 20*alpha[5]*t_3;
    double xt_ddd = 6*alpha[3] + 24*alpha[4]*t + 60*alpha[5]*t_2;
    return {xt, xt_d, xt_dd, xt_ddd};
}

vector<vector<double>> generation_next_waypoints(vector<double> start, vector<double> target, vector<double> alpha_s, vector<double> alpha_d, double t){
    vector<double> next_s = polysolver(start[0], start[1], start[2], alpha_s, t);
    vector<double> next_d = polysolver(start[3], start[4], start[5], alpha_d, t);

    return {next_s, next_d};
}

#endif