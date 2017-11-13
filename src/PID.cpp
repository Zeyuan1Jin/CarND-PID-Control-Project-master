#include <iostream>
#include <stdlib.h>
#include "PID.h"

#define WINDOW_SIZE 20


using namespace std;


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    p_error = 0;
    i_error = 0;
    d_error = 0;
}

void PID::UpdateError(double cte, double dt) {
    d_error = (cte - p_error) / dt;
    p_error = cte;
    i_error += cte * dt;
}

double PID::TotalError(double speed) {
    //PID parameters corrected for different speed
    // Reference: https://github.com/NikolasEnt/PID-controller
    return (Kp - 0.0032 * speed) * p_error + Ki * i_error + (Kd + 0.0002 * speed) * d_error;
}

