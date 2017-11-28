#include "PID.h"
#include <ctime>
#include <chrono>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    
    p_error = 0;
    i_error = 0;
    d_error = 0;
    
    t_old = std::chrono::system_clock::now();
    has_d_old_error = false;
}

void PID::UpdateError(double cte) {
    if (!has_d_old_error) {
        d_old_error = cte;
        has_d_old_error = true;
    }
  
    t = std::chrono::system_clock::now();
    
    // Calculate time elapsed
    std::chrono::duration<double> dt = t - t_old;
    
    // Update errors
    p_error = cte;
    i_error += cte;
    d_error = (cte - d_old_error) / dt.count();
    
    t_old = t;
    d_old_error = cte;
}

double PID::TotalError() {
  return -Kp * p_error - Ki * i_error - Kd * d_error;
}