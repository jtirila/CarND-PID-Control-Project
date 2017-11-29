# Self Driving Car Engineer Nanodegree Term2: PID Controller project 

By **J-M Tirilä**.

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

Alternatively, if you want to run the online twiddle version of the algorithm, the last step can be replaced 
with `./pid_learn`.

# My Implementation 

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

##### The Effect of the Proportional Term 

The proportional term always adjust the steering angle to the right direction, but has the characteristic that the 
corrections will overshoot the desired track because at the time when the steering angle is 
set to zero, the vehicle is typically approaching the desired path at an angle 
and hence overshoots the target. Only after this overshot is the course corrected towards the center again, 
yet again overshooting to the other side. This is clearly manifested in the following video clip (click to see on YouTube): 

[![D Control](https://img.youtube.com/vi/eY74W7dqnmM/0.jpg)](https://www.youtube.com/watch?v=eY74W7dqnmM)

##### The Effect of the Differential Term

The differential term will also capture _some_ aspects of correctibe behavior correctly. To be specific, when ther `CTE` 
is changing into one direction or the other, the vehicle will steer into the opposite direction for a brief while.  
However, as soon as the vehicle reaches a direction perpendicular to the desired one, the difference will be zero and 
the corrective behavior will stop. Hence, the differential term alone will typically never really _reduce_ the error, it will 
just prevent it from growing larger very fast (although it may keep getting larger and larger as the diff term is not 
concerned with the `CTE` as such, just its change rate). This slowly growing `CTE` behavior of the differential 
term can be again be seen on the video below (click for YouTube): 

[![D Control](https://img.youtube.com/vi/peCkIgqx6V4/0.jpg)](https://www.youtube.com/watch?v=peCkIgqx6V4)

The purpose of the differential term is really to counter the overshoot behavior of the proportional term: where the 
proportional term would keep the car moving across the desired path, the differential term will 
detect the decreasing CTE and adjust the steering in to the opposite direction. 

However, in the case of systematic drift into one direction or the other, even the differential term will be 
insufficient. In this project, there is no drift as such but the curves cause similar behavior: the proportional 
and differential term are slow to react to the effects of a curve. 

##### The Effect of the Integral Term

The intergral term serves as kind of a memory effect: if the vehicle keeps being off into one or the other direction
from the center of the track, the integral term will eventually pick this up and the steering will be adjusted 
towards the center lane. However, the integral term alone is a very poor steering guide as can be seen in the video 
below (click for YouTube):  
[![D Control](https://img.youtube.com/vi/B8wNKa_xYM0/0.jpg)](https://www.youtube.com/watch?v=B8wNKa_xYM0)

As is apparent, the integral term will overshoot the trajectory just like the proportional term, and even more so: 
where the proportional term immediately changes sign after crossing the path, the integral will only slowly 
manifest similar behavior, and will even keep turning into the wrong direction until finally the terms of opposite sign 
will outgrow the previous integral value. 

However, when combined to the proportional and differential term with suitably chosen weight parameters, 
the integral term will serve as a tuning factor that increasingly accelerates corrective behavior when there is 
long term drift, e.g. in longer curves. 

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

The typical (relevant) output lines from the beginning of a learning run look like this:

```
42["steer",{"steering_angle":0.0692990693866529,"throttle":0.4}]
Total error: 929.866 best error: 1018.45
Yes we were able to improve! Accelerating the changes for this parameter!
starting new twiddle iteration (round 4), new param values: 
Kp: 0.139469 Ki: 2.49661e-05 Kd: 4.3 dKp: 0.024 dKi: 6e-06 dKd: 0.36
CTE: 0.2475 Speed: 41.4062 Steering Value: 0.231396
--
--
42["steer",{"steering_angle":-0.78565514021861,"throttle":0.4}]
Total error: 751.797 best error: 929.866
Yes we were able to improve! Accelerating the changes for this parameter!
starting new twiddle iteration (round 5), new param values: 
Kp: 0.139469 Ki: 3.09661e-05 Kd: 4.3 dKp: 0.0288 dKi: 6e-06 dKd: 0.36
CTE: 1.7204 Speed: 43.0471 Steering Value: -0.727705
--
--
Total error: 792.439 best error: 751.797
Not able to improve.
That was the increase iteration so making a further attempt by decreasing the param value.
starting new twiddle iteration (round 6), new param values: 
Kp: 0.139469 Ki: 1.89661e-05 Kd: 4.3 dKp: 0.0288 dKi: 6e-06 dKd: 0.36
CTE: 0.4051 Speed: 43.0268 Steering Value: 0.0167006
```

And finally, the output will just indicate that the algorithm has converged (as per the tolerance value): 

```
The twiddle algorithm has converged, the final values are: 
Kp: 0.168278 Ki: 2.49661e-05 Kd: 4.66 dKp: 8.97229e-06 dKi: 9.96921e-10 dKd: 8.97229e-05
```

## Output

Below (click image for YouTube) is a video of one full lap of the car driving around the track using the 
twiddle-optimized parameters from above. The video was recorded on a smartphone because a screen recording on the same 
computer had a negative effect on 
steering performance. 

## Conclusion

The PID implementation as such was not very complicated. However, to reason about a suitable set of parameters was 
non-trivial and it took some time to find ones that kept the vehicle on track. 

One could argue that an optimization algorithm could be used from the beginning to iteratively look for better and 
better weights. However, the instrumentation of such a procedure would require either a lot of manual work or 
a programmatic way to restart the simulator. I did not have the time to investigate these options so my twiddle had 
to had a reasonable starting point so it was able to successfully drive around the track for longer periods, keeping
track of the total error. 

An interesting phenomenon is the fragility of the algorithm. For example, when trying to record a successful 
round around the track using the twiddle-optimized parameters, the screen recording affected the driving 
to an extent that the result was unusable. I resorted to using an external device to record the video. It would 
be interesting to investigate the causes for this phenomenon, and possible mitigations. 


