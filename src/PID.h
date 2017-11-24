#ifndef PID_H
#define PID_H

class PID {
  double previous_cte;
private:

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte, double speed);
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
  * Calculate the total PID error.
  */
  double TotalError();

  double GetAngle(double cte, double speed);
};

#endif /* PID_H */
