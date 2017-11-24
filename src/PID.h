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
  double total_absolute_error;

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
  void ChangeParamDiff(int changeType, int paramIdx);

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

  void ChangeParam(int change_type, int paramIdx);
  void ResetError();
  void AccelerateParamTrials(int paramIdx);
  void DecelerateParamTrials(int paramIdx);
  void PrintParamValues();

  double GetAngle(double cte, double speed);
};

#endif /* PID_H */
