#include "PID.h"
#include <iostream>
#include <cmath>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() : previous_cte(0.0), p_error(0.0), d_error(0.0), i_error(0.0), dKp(0.02), dKi(5e-06), dKd(0.3), total_absolute_error(0.0) {}

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
  total_absolute_error += fabs(cte);
  // cout << "p_error: " << p_error << " d_error: " << d_error << " i_error: " << i_error << "\n";
}

double PID::TotalD() const {
  return dKp + dKi + dKd;
}

double PID::TotalError() const {
  return total_absolute_error;
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

void PID::ResetError() {
  total_absolute_error = 0.0;
}

void PID::PrintParamValues() {
  cout << "Kp: " << Kp << " Ki: " << Ki << " Kd: " << Kd << " dKp: " << dKp << " dKi: " << dKi << " dKd: " << dKd << "\n";
}


void PID::ChangeParamDiff(int change_type, int paramIdx){
  double increase_coeff = 1.2;
  double decrease_coeff = 0.8;

  if(paramIdx == 0) {
    if (change_type == 0)
      dKp *= increase_coeff;
    else if (change_type == 1)
      dKp *= decrease_coeff;
  } else if(paramIdx == 1) {
    if (change_type == 0)
      dKi *= increase_coeff;
    else if (change_type == 1)
      dKi *= decrease_coeff;
  } else if(paramIdx == 2) {
    if (change_type == 0)
      dKd *= increase_coeff;
    else if (change_type == 1)
      dKd *=  decrease_coeff;
  }

}
void PID::AccelerateParamTrials(int paramIdx){
  ChangeParamDiff(0, paramIdx);
}

void PID::DecelerateParamTrials(int paramIdx){
  ChangeParamDiff(1, paramIdx);
}

double PID::GetAngle(double cte, double speed) {
  UpdateError(cte, speed);
  return -Kp * p_error -Kd * d_error -Ki * i_error;
}

