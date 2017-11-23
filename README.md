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

## PID Controllers: Pros and Cons

### My Implementation

TODO: add all of the important pieces of code here

#### Some PID controller characteristics in the context of this project

This section discusses some phenomena in the specific context of this project simulator and the track where the 
controller is used. 

##### A Note on the Differential Term

##### A Note on the Integral Term

## Conclusion


