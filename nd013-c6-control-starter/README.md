# Control and Trajectory Tracking for Autonomous Vehicle

# Proportional-Integral-Derivative (PID)

In this project, you will apply the skills you have acquired in this course to design a PID controller to perform vehicle trajectory tracking. Given a trajectory as an array of locations, and a simulation environment, you will design and code a PID controller and test its efficiency on the CARLA simulator used in the industry.

### Installation

Run the following commands to install the starter code in the Udacity Workspace:

Clone the <a href="https://github.com/udacity/nd013-c6-control-starter/tree/master" target="_blank">repository</a>:

`git clone https://github.com/udacity/nd013-c6-control-starter.git`

## Run Carla Simulator

Open new window

* `su - student`
// Will say permission denied, ignore and continue
* `cd /opt/carla-simulator/`
* `SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl`

## Compile and Run the Controller

Open new window

* `cd nd013-c6-control-starter/project`
* `./install-ubuntu.sh`
* `cd pid_controller/`
* `rm -rf rpclib`
* `git clone https://github.com/rpclib/rpclib.git`
* `cmake .`
* `make` (This last command compiles your c++ code, run it after every change in your code)

## Testing

To test your installation run the following commands.

* `cd nd013-c6-control-starter/project`
* `./run_main_pid.sh`
This will silently fail `ctrl + C` to stop
* `./run_main_pid.sh` (again)
Go to desktop mode to see CARLA

If error bind is already in use, or address already being used

* `ps -aux | grep carla`
* `kill id`

## How to implements

First i calculate distance between car coordinate and each point in best spiral to get which points is closest to the car

Second, i calculate the angle error using car coordinate and spiral point found above and calculate using atan2 function

Third, i calculate the throttle error using velocity of the car and velocity of spiral point found above

Then i implements pid class:
* init function to update kpi kdi kii and limit
* Update function that update cte, previous cte, differential cte and integral cte
* Total error function that calculate using pid formular

## Result

My result is in the plot .png but because of eviroments problem, my motion planning is not well so at the moments planning fail, the pid cant help it.


## Improve

Using twiddle algorithm can help tune the PID parameters
We can use bicycle model to get better because yaw of the car is not what desired steering, yaw of the front wheel is the desired steering.






