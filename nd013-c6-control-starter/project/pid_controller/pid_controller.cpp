/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
   PID::output_lim_max = output_lim_max;
   PID::output_lim_min = output_lim_min;
   PID::prev_cte = 0;
   PID::cte = 0;
   PID::int_cte = 0;
   PID::dif_cte = 0;
   PID::sum_cte = 0;
   PID::theta = 0.3;
   p_array[0] = 0.0;
   p_array[1] = 0.0;
   p_array[2] = 0.0;
   dp_array[0] = 1.0;
   dp_array[1] = 1.0;
   dp_array[2] = 1.0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  PID::cte = cte;
  PID::dif_cte = (PID::cte - PID::prev_cte) * PID::delta_t;
  PID::int_cte += cte * PID::delta_t;
  PID::prev_cte = PID::cte;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
   double best_error = 9999;
   double total_error;
   while(dp_array[0] + dp_array[1] + dp_array[2] > PID::theta)
   {
      for(int i =0; i < 3; i++)
      {
         p_array[i] += dp_array[i];
         total_error = -(p_array[0] * PID::cte + p_array[1] * PID::dif_cte + p_array[2] * PID::int_cte);
         if(total_error < best_error)
         {
            best_error = total_error;
            dp_array[i]  *= 1.1;
         }
         else
         {
            p_array[i] -= 2*dp_array[i];
            total_error = -(p_array[0] * PID::cte + p_array[1] * PID::dif_cte + p_array[2] * PID::int_cte);
            if(total_error < best_error)
            {
               best_error = total_error;
               dp_array[i]  *= 1.1;
            }
            else
            {
               p_array[i] += dp_array[i];
               dp_array[i] *= 0.9;
            }
         }
      }
   }
   double control = min(total_error, best_error);
   return max(min(control, output_lim_max), output_lim_min);

   return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  PID::delta_t = new_delta_time;
}