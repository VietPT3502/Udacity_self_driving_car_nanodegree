/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>
#include <limits.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  output_lim_max = output_lim_maxi;
  output_lim_min = output_lim_mini;
  pro_error = 0.0;
  dif_error = 0.0;
  int_error = 0.0;
  prev_error = 0.0;

  p_array[0] = Kpi;
  p_array[1] = Kdi;
  p_array[2] = Kii;


}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  
   std::cout << "update error" << std::endl;
   prev_error = pro_error;
   pro_error = cte;
   if(pro_error == 0) return;
   if (delta_time > 0)
      dif_error = (pro_error - prev_error) / delta_time;
   else
      dif_error = 0;
   int_error += cte * delta_time;

}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
  
   double total_error = (p_array[0] * pro_error) + (p_array[1] * dif_error) +  (p_array[2] * int_error);
         
         

  return max(min(total_error, output_lim_max), output_lim_min);
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  delta_time = new_delta_time;

  return delta_time;
}