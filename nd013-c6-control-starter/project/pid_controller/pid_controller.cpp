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
   this->output_lim_max = output_lim_maxi;
   this->output_lim_min = output_lim_mini;
   this->prev_cte = 0;
   this->cte = 0;
   this->int_cte = 0;
   this->dif_cte = 0;
   this->sum_cte = 0;
   this->theta = 0.3;
   this->p_array[0] = Kpi;
   this->p_array[1] = Kii;
   this->p_array[2] = Kdi;
   this->dp_array[0] = 1.0;
   this->dp_array[1] = 1.0;
   this->dp_array[2] = 1.0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  this->cte = cte;
  this->dif_cte = (this->cte - this->prev_cte) / this->delta_t;
  this->int_cte += cte * this->delta_t;
  this->prev_cte = this->cte;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
   double best_error = 9999;
   double total_error;
   while(this->dp_array[0] + this->dp_array[1] + this->dp_array[2] > this->theta)
   {
      for(int i =0; i < 3; i++)
      {
         this->p_array[i] += this->dp_array[i];
         total_error = -(this->p_array[0] * this->cte + this->p_array[1] * this->dif_cte + this->p_array[2] * this->int_cte);
         std::cout << "error" << total_error << std::endl; 
         if(total_error < best_error)
         {
            best_error = total_error;
            this->dp_array[i]  *= 1.1;
         }
         else
         {
            this->p_array[i] -= 2*this->dp_array[i];
            total_error = -(this->p_array[0] * this->cte + this->p_array[1] * this->dif_cte + this->p_array[2] * this->int_cte);
            if(total_error < best_error)
            {
               best_error = total_error;
               this->dp_array[i]  *= 1.1;
            }
            else
            {
               this->p_array[i] += this->dp_array[i];
               this->dp_array[i] *= 0.9;
            }
         }
      }
   }
   double control = min(total_error, best_error);
   return max(min(control, output_lim_max), output_lim_min);

   return control;
}

void PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  this->delta_t = new_delta_time;
}