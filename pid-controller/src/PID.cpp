#include "PID.h"

#include <iostream>
#include <cmath>
#include <vector>
#include <numeric>
#include <algorithm>
#include <uWS/uWS.h>

using std::vector;
using std::accumulate;
using std::numeric_limits;
using std::cout;
using std::endl;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(vector<double> tau, vector<double> gd, int timestep){
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  this->_cte = {0, 0, 0};
  this->_tau = tau;
  this->_gd = gd;
  this->counter = 1;
  this->twiddle_timestep = timestep;
  this->do_twiddle = false;
  this->total_error = 0.0;
  this->best_error = numeric_limits<float>::max();
}

void PID::UpdateError(double cte){
  /**
   * TODO: Update PID errors based on cte.
   */
  this->prev_cte = _cte[0];
  this->_cte[1] = cte - this->prev_cte;
  this->_cte[0] = cte;
  this->_cte[2] += cte;
}

void PID::TotalError(){
  /**
   * TODO: Calculate and return the total error
   */
  if ((this->counter % (2*this->twiddle_timestep)) == 0) this->do_twiddle = true;
  else this->do_twiddle = false;

  if (this->counter >= this->twiddle_timestep) this->total_error += pow(this->_cte[0], 2);
}

float PID::Steer(){
  float steer = -this->_tau[0]*this->_cte[0] - this->_tau[1]*this->_cte[1] - this->_tau[2]*this->_cte[2];
  if (steer > 1){
    steer = 1.0;
  }
  else if (steer < -1){
    steer = -1.0;
  }
  return steer;
}

float PID::Twiddle(double tol, double rate){
  if (this->do_twiddle == true){
    float total_error = this->total_error/ this->counter; 
    for (unsigned int i=0; i < _cte.size(); i++){
      // increase tau
      this->_tau[i] += this->_gd[i];
      // Check currently if total error is less than best error
      if (total_error < this->best_error){
        this->best_error = total_error;
        this->_gd[i] *= (1.0 + rate);
      }
      // else reset the tau and incre or decre the tau and gd depending on if total error is still greater than best error
      else {
        this->_tau[i] -= 2*this->_gd[i];
        if ((fabs(this->_cte[0]) > tol)){
          this->_gd[i] *= (1.0 + rate);
        }
        else {
          this->_tau[i] += this->_gd[i];
          this->_gd[i] *= (1.0 - rate);
        }
      }
    }

    this->counter = 0;
    this->total_error = 0;
  }
  
  float steer = Steer();
  return steer;
}