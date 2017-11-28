# Self Driving Car Engineer Nanodegree Term2: PID Controller project 

## Basic Project Info

The task in this project is to implement a [PID controller](https://en.wikipedia.org/wiki/PID_controller) in C++. The 
controller is to maneuver a car in the Udacity simulator. The info the code receives from the simulator 
contains the following elements on each iteration:  
* The speed of the car
* The steering angle 
  - Note: this is not the same as the car's orientation
* The _cross track error_, that is, the distance from the reference path (center of lane)   

Using this info, the code sends adjusted steering angles back to the simulator. Optionally, one 
can also modify the speed of the vehicle as desired, using another PID controller or some other mechanism.  

### Success criteria

As per the project rubric, the project is accepted when the code is clean enough and implements the 
basic flow of a PID controller in C++ for the steering angle.  

---

# Getting Started Using the Controller 

The dependencies and hints for setting up the environment can be found at the 
[original Udacity repository for this project](https://github.com/udacity/CarND-PID-Control-Project). 

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

# Discussion

## Introduction

This part discusses the underlying PID controller framework and the way I implemented it. 

I will just briefly discuss the basic principle of PID control here. For further information, see
e.g. Udacity's learning material or the [Wikipedia oarticle on PID controllers](https://en.wikipedia.org/wiki/PID_controller).

The main metric underlying all the computations in PID control is, in a vehicle context, the `Cross Track Error` (CTE) - that is, 
the perpendicular distance form the desired path. The steering actions are based on a weighted sum of three different 
statistics based on CTE, discussed below.

### The Three Terms

The PID controller steering angle is computed as
```
A = K_p * CTE + K_d * diff(CTE) + K_i * int(CTE)
``` 
where `K_p`, `K_d` and `K_i` are the weights of the proportional, differential and intergral terms, respectively. 

In the discrete case, the differential term `diff(CTE)` can be calculated as the difference between the current and previous 
`CTE`. 

The integral term `int(CTE)` is just the integral (or in the discrete case, sum) of all the previous `CTE`'s. 

Now, in principle, some normalizations might make sense, and some are also applied here: in a case where the speed of the vehicle varies greatly but 
the measurements arrive at approximately constant 
intervals, it may be beneficial in the integration phase to multiply 
each instantaneous `CTE` by the speed of the vehicle to obtain an integral of the `CTE` that does not 
depend too much on speed. For example, if one multiplies the vehicle's speed by 10, we would 
only receive 1/10 of the previous measurements so multiplying by 10 would normalize the `CTE` integral. 
This normalization is used in this project. 

Also, if the intervals between the measurements varied, a raw difference between two consecutive 
would vary depending on the length of the interval so division could be useful. However, in this particular 
case the measurement interval is approximately constant so this latter normalization is ignored. 
   
### My Implementation

The essence of my implementation are the following functions:  
```
double PID::GetAngle(double cte, double speed) {
  UpdateError(cte, speed);
  return -Kp * p_error -Kd * d_error -Ki * i_error;
}
```
and 
```
void PID::UpdateError(double cte, double speed) {
  p_error = cte;
  d_error = cte - previous_cte;
  i_error += cte * speed;
  previous_cte = cte;
  total_absolute_error += fabs(cte);
}
```

That update the "error" terms `CTE` `diff(CTE)` `int(CTE)` and then compute the weighted sum 
of these terms. 


#### Some PID Controller Characteristics in the Context of This Project

This section discusses some phenomena in the specific context of this project simulator and the track where the 
controller is used. 

##### Something on the Proportional Term 
##### A Note on the Differential Term

##### A Note on the Integral Term

#### Tuning the Weights

My approach to finding suitable parameters was two-fold: 
1. First I experimented manually so that I obtained a nice initial set of weights to keep the vehicle on track 
1. Subsequently, I implemented an online version of the twiddle algorithm that keeps tuning the weights up and down, 
discarding changes that increase total (absolute) error and keep changes that decrease this error, accelerating 
further changes into the same direction. This online version of the algorithm requires quite a bit of dynamic 
processing and is a bit hard to reason about, but here is the code anyway: 

```
if(pid.TotalD() > tolerance) {
  if (this_twiddle_iteration_time < -0.5) {
    twiddle_start_time = std::chrono::high_resolution_clock::now();
    this_twiddle_iteration_time = 0.0;


    if (twiddle_iteration_ind == -1) {
      // Do nothing
    } else {
      if (twiddle_iteration_ind > 0) {
        std::cout << "Total error: " << pid.TotalError() << " best error: " << best_total_error << "\n";
        if (pid.TotalError() < best_total_error) {
          std::cout << "Yes we were able to improve! Accelerating the changes for this parameter!\n";
          best_total_error = pid.TotalError();
          int previous_param_to_modify = (param_to_modify + 2) % 3;
          if(previous_param_to_modify < 0)
            previous_param_to_modify += 3;
          pid.AccelerateParamTrials(previous_param_to_modify);
          increase_decrease_index = 0;
        } else {
          std::cout << "Not able to improve.\n";
          int previous_param_to_modify = (param_to_modify +2) % 3;
          if (increase_decrease_index == 1) {
            std::cout << "That was the decrease iteration so resetting to previous value and moving on.\n";
            pid.ChangeParam(0, previous_param_to_modify);
            pid.DecelerateParamTrials(previous_param_to_modify);
            increase_decrease_index = 0;
          } else {
            std::cout
                << "That was the increase iteration so making a further attempt by decreasing the param value.\n";
            pid.ChangeParam(1, previous_param_to_modify);
            param_to_modify = previous_param_to_modify;
            increase_decrease_index = 1;
          }
        }
      } else {
        best_total_error = pid.TotalError();
      }
      pid.ResetError();
      pid.ChangeParam(increase_decrease_index, param_to_modify);
      param_to_modify = (param_to_modify + 1) % 3;
    }
    twiddle_iteration_ind++;
    std::cout << "starting new twiddle iteration (round " << twiddle_iteration_ind << "), new param values: \n";
    pid.PrintParamValues();
  } else if (this_twiddle_iteration_time > 57) {
    this_twiddle_iteration_time = -1;
  } else {
    auto t_now = std::chrono::high_resolution_clock::now();
    this_twiddle_iteration_time = std::chrono::duration<double, std::milli>(
        t_now - twiddle_start_time).count() / 1000.0;

  }
} else {
  std::cout << "The twiddle algorithm has converged, the final values are: \n";
  pid.PrintParamValues();
}

``` 

The twiddle iteration length 57 (seconds) is just determined by observing the approximate time it takes for the vehicle to drive 
a full lap around the track, and no shorter period is used because the characteristics of the track 
vary greatly at its different parts and with shorted twiddle intervals, this would result in spurious 
differenced in the total error integrals. 


## Conclusion


