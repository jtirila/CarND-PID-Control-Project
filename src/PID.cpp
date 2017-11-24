#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() : previous_cte(0.0), p_error(0.0), d_error(0.0), i_error(0.0) {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(double cte, double speed) {
  p_error = cte;
  d_error = cte - previous_cte;
  i_error += cte * speed;
  previous_cte = cte;
  cout << "p_error: " << p_error << " d_error: " << d_error << " i_error: " << i_error << "\n";
}

double PID::TotalError() {
}

double PID::GetAngle(double cte, double speed) {
  cout << "in get_angle: cte: " << cte << "\n";
  UpdateError(cte, speed);
  return -Kp * p_error -Kd * d_error -Ki * i_error;
}

