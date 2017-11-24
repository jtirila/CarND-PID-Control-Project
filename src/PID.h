#ifndef PID_H
#define PID_H

class PID {
  double previous_cte;
private:

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte, double speed);

  /*
  * Errors
  */
  double p_error;
  double d_error;
  double i_error;

  /*
  * Coefficients
  */
  // Kek
  double Kp;
  double Ki;
  double Kd;
public:


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
