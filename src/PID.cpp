#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() : previous_cte(0.0), p_error(0.0), d_error(0.0), i_error(0.0), dKp(0.1), dKi(0.1), dKd(0.1) {}

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

double PID::TotalD() const {
  return dKp + dKi + dKd;
}

double PID::TotalError() const {
  return i_error;
}

void PID::ChangeParam(int change_type, int paramIdx) {
  if(paramIdx == 0) {
    if (change_type == 0)
      Kp += dKp;
    else if (change_type == 1)
      Kp -= dKp;
  } else if(paramIdx == 1) {
    if (change_type == 0)
      Ki += dKi;
    else if (change_type == 1)
      Ki -= dKi;
  } else if(paramIdx == 2) {
    if (change_type == 0)
      Kd += dKd;
    else if (change_type == 1)
      Kd -= dKd;
  }
}

void PID::IncreaseParam(int paramIdx){
  ChangeParam(0, paramIdx);
}

void PID::DecreaseParam(int paramIdx){
  ChangeParam(1, paramIdx);
}

double PID::GetAngle(double cte, double speed) {
  cout << "in get_angle: cte: " << cte << "\n";
  UpdateError(cte, speed);
  return -Kp * p_error -Kd * d_error -Ki * i_error;
}

