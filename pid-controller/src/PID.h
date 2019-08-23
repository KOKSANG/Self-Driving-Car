#ifndef PID_H
#define PID_H
#include <vector>
#include <uWS/uWS.h>

using std::vector;

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(vector<double> tau, vector<double> gd, int max);
  /**
   * PID Errors
   */
  vector<double> _cte;
  /**
   * PID Coefficients
   */
  vector<double> _tau;
  /**
   * Gradient Descent
   */
  vector<double> _gd;
  /**
   * Parameters
  */
  int counter;
  float best_error;

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  void TotalError();

  float Steer();

  float Twiddle(double tol, double rate);

 private:
  bool do_twiddle;
  int twiddle_timestep;
  double prev_cte;
  float total_error;

};

#endif  // PID_H