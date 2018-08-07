#include "PID.h"
#include <iostream>
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/
static const double DP_DIFF  = 0.2;
static const int TWID_ERR_MIN = 75;
static const int TWID_RUN_LEN = 250;
static const double TWID_TOL = 0.00001;

PID::PID():twiddle(false) {}

PID::PID(bool twid):twiddle(twid) {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double Dpp, double Dpi, double Dpd) {
  K[0] = Kp;
  K[1] = Ki;
  K[2] = Kd;
  dp[0] = Dpp;
  dp[1] = Dpd;
  dp[2] = Dpi;
  i_error = 0.0;
  firstcte = true;
  count = 0;
  state = 0;
  twid_i = 0;
  best_err = numeric_limits<double>::max();
}

//returns true if need to reset
bool PID::UpdateError(double cte) {
  p_error = cte;
  if(firstcte)
  {
      prevCte = cte;
      firstcte = false;
  }
  d_error = cte - prevCte;
  prevCte = cte;
  i_error += cte;
  //cout << "p_err:" << p_error << ", d_err:" << d_error << ", i_err:" << i_error << endl;

  if(twiddle)
  {
    if(count > TWID_ERR_MIN)
      error = cte*cte;
    if(count > TWID_RUN_LEN)
    {
      TwiddleUpdate();
      count =0;
      cout << "K: " << K[0] << ", " << K[1] << ", " << K[2] << 
              "| dp: " << dp[0] << ", " << dp[1] << ", " << dp[2] << endl;

      return true;
    }
    else
      count++;
  }
  //cout << "Count: " << count << endl;
  return false;
}

double PID::TotalError() {
  return K[0] * p_error + K[1] * d_error + K[2] * i_error;
}

void PID::TwiddleUpdate() {
  if(state == 0)
  {
    K[twid_i] += dp[twid_i];
    state++;
    cout << "state0" << endl;
  }
  else if(state == 1)
  {
    if(error < best_err)
    {
      // Found Better Parameter, increase DP
      best_err = error;
      dp[twid_i] *= (1 + DP_DIFF);
      //Goto next parameter and reset state
      twid_i = (twid_i + 1) % 3;
      state = 0;
      cout << "state1: Found Better K" << endl;
      TwiddleTol();
    }
    else
    {
      K[twid_i] -= 2*dp[twid_i];
      cout << "state1: Try -2dp" << endl;
      state++;
    }
  }
  else if(state == 2)
  {
    if(error < best_err)
    {
      // Found Better Parameter, increase DP
      best_err = error;
      dp[twid_i] *= (1 + DP_DIFF);
      cout << "state2: Found Better K with -2dp" << endl;
      TwiddleTol();
    }
    else
    {
      K[twid_i] += dp[twid_i];
      dp[twid_i] *= (1 - DP_DIFF);
      cout << "state2: Go back to original K and reduce DP" << endl;
      TwiddleTol();
    }

    // Goto next parameter and reset state
    twid_i = (twid_i + 1) % 3;
    state = 0; 
  }

}

void PID::TwiddleTol(){
  // Turns twiddle off onces sum(dp) < twiddle tolerance
  if((dp[0] + dp[1] + d[2]) < TWID_TOL)
    twiddle = false;
}

