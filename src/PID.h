#ifndef PID_H
#define PID_H

#include <vector>
#include <iostream>

using namespace std;

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */
  std::vector<double> params;
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

/*
 * Twiddle
 * Some helper class for Gradient Descent tuning of PID parameters.
 *  Created on: Aug 12, 2017
 *      Author: Andy Scott
 */
class Twiddle {
public:
  int cycle_len;
  int count;
  bool init_state;
  int index;
  int state;
  bool err_count;
  double threshold;

  std::vector<double> dp;
  double cum_err;
  double best_err;

  /*
  * Constructor
  */
  Twiddle();

  /*
  * Destructor.
  */
  virtual ~Twiddle();

  /*
  * Initialize Twiddle.
  */
  void Init(int n, double Kp_d, double Ki_d, double Kd_d, double tol);

  /*
  * Update the PID param variables given cross track error and state of twiddle cycle.
  */
  void UpdateTwiddle(double cte, std::vector<double>& params);

  /*
  Sum diff parametrs vector and return true if less than threshold value.
  */
  double Threshold_test();

};

#endif /* PID_H */
