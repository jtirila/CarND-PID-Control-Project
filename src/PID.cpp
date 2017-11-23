#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() : total_int(0.0), previous_cte(0.0), is_initialized(false) {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(double cte) {
}

double PID::TotalError() {
}

double PID::GetAngle(double cte, double speed) {
  // FIXME: a rather heavy procedure to keep track of whether first round or not
  double y_diff;
  y_diff = (cte - previous_cte);
  previous_cte = cte;
  is_initialized = true;
  previous_cte = cte;
  cout << "y_diff: " << y_diff << " total_int: " << total_int << "\n";


  total_int += cte * speed;

  return -Kp * cte - Kd * y_diff - Ki * total_int;
}

