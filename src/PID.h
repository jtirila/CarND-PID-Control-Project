#ifndef PID_H
#define PID_H

class PID {
  double previous_cte;

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

  double dKp;
  double dKi;
  double dKd;

private:

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte, double speed);
  void ChangeParam(int change_type, int paramIdx);

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

  double TotalD() const;


  /*
  * Calculate the total PID error.
  */
  double TotalError() const;

  void IncreaseParam(int paramIdx);
  void DecreaseParam(int paramIdx);

  double GetAngle(double cte, double speed);
};

#endif /* PID_H */
