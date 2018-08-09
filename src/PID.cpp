#include "PID.h"
#include <iostream>
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/
static const double DP_DIFF  = 0.1;
static const int TWID_ERR_MIN = 0;
static const int TWID_RUN_LEN = 2500;
static const double TWID_TOL = 0.00001;

PID::PID():twiddle(false) {}

PID::PID(bool twid):twiddle(twid) {}

PID::~PID() {}

void PID::Init(double Kp, double Kd, double Ki, double Dpp, double Dpd, double Dpi) {
  K[0] = Kp;
  K[1] = Kd;
  K[2] = Ki;
  dp[0] = Dpp;
  dp[1] = Dpd;
  dp[2] = Dpi;

  //reset all variables
  i_error = 0.0;
  firstcte = true;
  count = 0;
  state = 0;
  twid_i = 0;
  best_err = numeric_limits<double>::max();
  runs =0 ;
  if(twiddle)
    TwiddleUpdate();
}

//returns true if need to reset
bool PID::UpdateError(double cte) {
  p_error = cte;
  if(firstcte)
  {
      prevCte = cte;
      i_error = 0;
      firstcte = false;
  }
  d_error = cte - prevCte;
  prevCte = cte;
  i_error += cte;
  //cout << "p_err:" << p_error << ", d_err:" << d_error << ", i_err:" << i_error << endl;

  if(twiddle)
  {
    if(count > TWID_ERR_MIN)
      error += cte*cte;
    //Restart simulation and do next twiddle parameters.
    if(count > TWID_RUN_LEN || error > best_err)
    {
      cout << "K: " << K[0] << ", " << K[1] << ", " << K[2] << 
              ", dp: " << dp[0] << ", " << dp[1] << ", " << dp[2] << endl;
      cout << "Count: " << count << "/" << TWID_RUN_LEN <<
              ", Best Error: " << best_err <<
              ", Error: " << error << endl << endl;
      TwiddleUpdate();

      //Reset the PID state/variables
      count =0;
      error = 0.0;
      firstcte = true;

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
  if(state == 1)
  {
    if(error < best_err)
    {
      // Found Better Parameter, increase DP
      best_err = error;
      dp[twid_i] *= (1 + DP_DIFF);
      //Goto next parameter and reset state
      cout << "state1[" << twid_i << "]: Found Better K with +dp" << endl;
      twid_i = (twid_i + 1) % 3;
      state = 0;
      TwiddleTol();
    }
    else
    {
      //Try Changing parameter in negative direction
      K[twid_i] -= 2*dp[twid_i];
      //cout << "state1: Try -2dp" << endl;
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
      cout << "state2[" << twid_i << "]: Found Better K with -2dp" << endl;
    }
    else
    {
      K[twid_i] += dp[twid_i];
      dp[twid_i] *= (1 - DP_DIFF);
      cout << "state2[" << twid_i << "]: Go back to original K and reduce DP" << endl;
    }

    // Goto next parameter and reset state
    TwiddleTol();
    twid_i = (twid_i + 1) % 3;
    state = 0; 
  }
  // Try K[i] = dp[i]
  if(state == 0)
  {
    K[twid_i] += dp[twid_i];
    state++;
  }

}

void PID::TwiddleTol(){
  // Turns twiddle off onces sum(dp) < twiddle tolerance
#if 0
  if((dp[0] + dp[1] + d[2]) < TWID_TOL)
    twiddle = false;
#endif
  cout << "Runs: " << ++runs << endl;;
}

