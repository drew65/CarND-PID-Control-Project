#include "PID.h"

#include <iostream>
#include <vector>
#include <numeric>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  //this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  params.push_back(Kp);
  params.push_back(Ki);
  params.push_back(Kd);
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::UpdateError(double cte) {
  d_error = (cte - p_error);
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  return -params[0] *p_error - params[2] * d_error - params[1] * i_error;
}

/*
 * Twiddle
 * Some helper functions for Gradient Descent tuning of PID parameters.
 *  Created on: Aug 12, 2017
 *      Author: Andy Scott
 */
Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

void Twiddle::Init(int n, double Kp_d, double Ki_d, double Kd_d, double tol=0.001) {
  dp.push_back(Kp_d);
  dp.push_back(Ki_d);
  dp.push_back(Kd_d);

  threshold = tol;
  cycle_len = n;
  count = 0;
  init_state = true;
  index = 0;
  state = 0;
  err_count = true;

  cum_err = 0.0;
  best_err = 0.0;

}

void Twiddle::UpdateTwiddle(double cte, std::vector<double>& params) {
  if (err_count) cum_err += cte*cte;
  count++;
  if (count == cycle_len) {
    count = 0;
    if (init_state == true) {
      init_state = false;
      best_err = cum_err/cycle_len;
      err_count = false;
      params[index] += dp[index];
    } else {
      switch (state) {
        case 0 : err_count = true;
                 state++;
                 break;
        case 1 : count = 0;
                 err_count = false;
                 if (cum_err/cycle_len < best_err) {
                   best_err = cum_err/cycle_len;
                   dp[index] *= 1.1;
                   state = 0;
                   index = (index+1) % 3;
                   params[index] += dp[index];
                 } else {
                   params[index] -= 2 * dp[index];
                   state++;
                 }
                 cum_err = 0.0;
                 break;
        case 2 : err_count = true;
                 state++;
        case 3 : count = 0;
                 err_count = false;
                 if (cum_err/cycle_len < best_err) {
                   best_err = cum_err/cycle_len;
                   dp[index] *= 1.1;
                   state = 0;
                   //index = (index+1) % 3;
                   //params[index] += dp[index];
                 } else {
                   params[index] += dp[index];
                   dp[index] *= 0.9;
                 }
                 index = (index+1) % 3;
                 params[index] += dp[index];
                 state = 0;
      }

    }

  }
}

double Twiddle::Threshold_test() {
   double sum = std::accumulate(dp.begin(), dp.end(), 0.0);
   //if (sum < threshold) return true;
   //else return false;
   return sum;
}
