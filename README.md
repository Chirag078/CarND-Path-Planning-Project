# CarND-Path-Planning-Project

### Self-Driving Car Engineer Nanodegree Program

--- 

---

## Goals :- 
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

---
---

## Here is the data provided from the Simulator to the C++ Program

###  Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH
---

### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

---

### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

---

### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

---
## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

---
---
## Solution Approch :
## Prediction : 

- Based on Sensor Fusion data we can predict the Environment around the our Car

- so here we will decide the other cars lane and speed by using the input from the sensor fusion module
here i have taken the below steps :
	1. i have checked the lane of the car based on d parameter 
	2. car speed is calculated based on x and y velocity component.
 ---

## Behavior Planning :
	
- if the Car is in same lane and distance is < 30m then consider car is ahead.
- if lane is not same then 
	1. check for 30 front and back both side for left lane
	2. check for 30 front and back both side for left lane 
	     and decide weather we can change the lane or not
		 
- Check if car is ahead 
  1. Check for the left lane whether it is safe to change lane to the left ?
	 if it is safe then change lane from middle lane to left
  2. Check for the right lane whether it is safe to change lane to the right ?
	 if it is safe then change lane from middle lane to right
  3. If No way to go then reduce the speed
- if there is no vehicle in front of our car with in 30m
  1. check our lane
      - if we are in left lane : change the lane from left to middle if lane change is safe
	  - if we are in left lane : change the lane from left to middle if lane change is safe
  2. increase the Speed Difference if it is less then Max Limit 	

---

## Trajectory Generation :

- calculation of the trajectory is based on the speed and lane output from the behavior of car coordinates and privious path points.

- Initialize the spline calculation by considering below points:
	1. Use Cars 2 Points from previous Trajectory or use Car Position if there is not previous trajectory 
	2. consider 3 Point from the Future Trajectory  
	3. Transfer this points coordinate to local car coordinate  
	
- the pass trajectory points are copied to the new trajectory.
- the rest of the points are calculated by evaluating the spline and transforming the output coordinates to not local coordinates 
Worth noticing the change in the velocity of the car.
- increase/decrease speed on every trajectory points instead of doing it for the complete- trajectory.
   
---

---

## Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

   
---

---

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

---
---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

---
---

## Code Style

[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
