/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <math.h>
#include <iostream>
#include <vector>

using namespace std;

PID::PID() {  }

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi,
               double output_lim_mini) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  Kpi = Kpi;
  Kii = Kii;
  Kdi = Kdi;
  output_lim_maxi = output_lim_maxi;
  output_lim_mini = output_lim_mini;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   **/
  d_error = cte - p_error;
  i_error += cte;
  p_error = cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   * The code should return a value in the interval [output_lim_mini,
   * output_lim_maxi]
   */
  double control;
  control = -Kpi * p_error - Kii * i_error - Kdi * d_error;
  if (control < output_lim_mini) {
    control = output_lim_mini;
  } else if (control > output_lim_maxi) {
    control = output_lim_maxi;
  }
  return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
  /**
   * TODO: Update the delta time with new value
   */
  this->dt = new_delta_time;
}