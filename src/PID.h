#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double prevCte;

  /*
  * Coefficients
  */ 
  double K[3];

  /*
  * Constructor
  */
  PID();
  PID(bool twid);

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, double Dpp, double Dpi, double Dpd);
  bool firstcte;

  /*
  * Update the PID error variables given cross track error.
  */
  bool UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Twiddle stuff
  */
  int count;
  bool twiddle;
  bool firstTwiddle;
  double dp[3];
  int twid_i;
  double error;
  int state; // 0 p+=dp, 1 p-=2dp
  double best_err;
  void TwiddleUpdate();
};

#endif /* PID_H */
