# CarND-Controls-PID

Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.

    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Running the PID Controller and Twiddle process

To run the PID with my chosen parameters:

```
./pid
```

To run the PID with custom parameters, Kp, Kd, Ki 

```
./pid <Kp> <Kd> <Ki>
```

To run the twiddle algorithm with predefined starting K and dp values:

```
./pid 1
```

## Write-up

To choose the K parameters, a combination of manual tuning and twiddle was used. Initial experiments with the twiddle algorithm proved that the dp values played a huge role into the end results. Incorrect dp values could cause the twiddle algorithm to settle on local minimas. For example, if a large dp value was chosen for Ki, generally a small value, the twiddle algorithm would take considerably more iterations to converge relative to the other parameters causing the Ki parameter to be excluded from simulation runs during the period where Kp and Kd starts to converge.  

Initial manual experimentation developed a good sense for how the parameters affected the car's behavior and a good starting point for twiddle. The proportional term determines how fast the car can respond to curves. If  Kp is too low, it won't be able to steer fast enough and if it's too high it will cause the car to oscillate to unstability. The integral term determines movement towards the set point and also removes any error at steady state. If Ki is too low, it cannot respond fast enough to remove oscillations and if it's too low it'll cause unstable oscillations. The differential term smooths the return movement to set point. If the Kd is too low, it'll increase the oscillation frequency back to the set point and if it's too high it might take too long to return to the set point. After several iterations of playing around with the parameters, I chose dp values of 0.1, 0.5, and 0.001 for Kp, Kd, and Ki respectively. 

After several twiddle runs of various sizes, I decided to empirically determine the K parameters because the twiddle algorithm was over optimizing. Since twiddle optimizes to error, it starts to increase Kd to reduce the oscillation period, which in turn does minimizes error. This has the side effect of causing jerky motions which would be unpleasant to the passenger. The values I chose are as follows.

| Parameter | Value      |
| --------- |:----------:|
| Kp        | 0.199      |
| Kd        | 2.25395    |
| Ki        | 0.00771561 |


