# SelfDrivingCarND_PathPlanner

The goal for the 1st project for the Self driving car nanodegree, Term 3, was to develop a path planner for a car driving in a highway simulator, with the goal to drive safely, comfortly and legaly.
For each cycle of the simulator, the planner has the aim to compute the min cost path with a limited jerk and without overcome speed limits. Furthermore, the main focus of the driving was to avoid collision with other cars in the road.

## Repository structure
The repository is made of 3 folders and some files:
1.  /src folder: includes all the src files required for executable build and to execute the path planner;
2.	/executables folder: contains the compiled files
3.  /data: includes the file representing the description of the track in waypoints
4.  CMakeLists.txt and cmakepatch.txt are the configuration files used to make the system executable on windows. 
5.  install-mac.sh and install-ubuntu.sh: scripts for mac and ubuntu provided by udacity in order to make the configuration of the OS faster (e.g. uWebSockets installation) 

## 1. Planner Definition and Development

The project was mainly involved in the definition of the logic of the path planner. This superviser has the aim to:
1.  Compute the next best state (amd transaction) in the global state machine;
2.  Compute the target goal depending on the best next state computed;
3.  Compute a smooth and min jerk path using the knowledje about the previous path and the trajectory constraint.

### Planner Skeleton

The skeleton of the Path planner project is made by two main parts: 
1. *main.cpp* which include all the logic for the simulator connection as well as the management of the trajectory computation; 
2. PathPlanner.h and PathPlanner.cpp which includes the core logic of the path planning itself.

The main function has the aim to manage the entire connection with the simulator. In the loop of the main function is also managed the communication with the simulator for the trajectories to be shownd.
The PathPlanner includes all the logic for the next state cost computation, goals definition and trajectory genearation


### Planner initialization

The path planner, that is the core of the implementation, is initialized in the main.cpp file setting some environment variables and target planning parameters.

```cpp
PathPlanner planner = PathPlanner(LANE_WIDTH, NUM_LANES, HORIZON, UPDATE_TIMESTAMP);
planner.InitalizeMap(map_file_, max_s);
```

Main parameters initialized are the lane width (4.0 mt) and the number of lanes (3).
Therfore, the *InitializeMap* method is called. This function has the aim to load all the waypoints in the provided file, compute the best fitting function using splines, and then the list of waypoints is refined in order to have a smooter path definition.

```cpp
//Initialize the roadMap object with with the waypoints form the csv file
this->road_map = RoadMap(map_waypoints_x,map_waypoints_y,map_waypoints_s);
this->road_map.InitSpline();
this->road_map.RefineSpline();
```

Finally, in oder to handle the "circularity" of the map, a transaction point is added:

```cpp
// Add one more point to the map in order to ajust transiction between the end and the start of the track
map_waypoints_x.push_back(map_waypoints_x[0]);
map_waypoints_y.push_back(map_waypoints_y[0]);
map_waypoints_s.push_back(MAX_TRACK_S);
map_waypoints_dx.push_back(map_waypoints_dx[0]);
map_waypoints_dy.push_back(map_waypoints_dy[0]);	
```
### Path planning

The path planning starts with the decision if to compute a new trajectory based on the previus path provided by the simulator. In order to reduce computation timings, was decided to reduce the computation of a new trajectory only in the first message received by the simulator and only if the *previous_path* vector provided by the simulator are small enough


```cpp
// In case this is the first time a path is developed, compute the trajectory as the best state was KL

if(previous_path_x.size() == 0){
		telemetry t {D2lane(j[1]["d"]),j[1]["s"],j[1]["speed"]};
		target_path = ComputeKLPath(t);
}
// If the prev_path is long enough, resent it as it is
else if(previous_path_x.size() >= 125)
{
    target_path = this->car.current_path;
		target_path.X = previous_path_x;
		target_path.Y = previous_path_y;
}
// In case the prev_path was shorter than a fixed value, compute a new trajectory piece to be appended to the previous one
// 125 is the lenght of the new path computed 
else
{ ...
```

A new path is computed only if the *previous_path* size is smaller than the number of trajectory waypoints that the further logic will compute ( *PATH_HORIZON -> 2.5s* and *PATH_WP_PERIOD -> 0.02s*).

In case a new trajectory is computed, the planner will merge these new waypoints to the still remaining path.


```cpp
...
vector<double> path_x = previous_path_x;
vector<double> path_y = previous_path_y;

// Append the new computed trajectory, to the previous_trajectory points
for (int i=0; i<target_path.X.size(); i++)
{
    path_x.push_back(target_path.X[i]);
    path_y.push_back(target_path.Y[i]);
}
target_path.X = path_x;
target_path.Y = path_y;
...
```

#### Smooth trajectory integration

The integration of the previous_path trajectory and the new part of the trajectory computed was one hard issue. Without using some smooth logic, the car was figure out to overcome the max acceleration. 

In order to avoid this kind of unwanted behaviour, the new part of trajectory was computed starting from the "end state" of the previus path. The last path is one of the attributes of the *MainVehicle* object, and information regargin the end_s,end_speed and end_d of the path can be retreived from this object.


```cpp
...
// The telemetry object is used to simplfy the code
telemetry t {D2lane(this->car.current_path.D_end[0]),this->car.current_path.S_end[0],
			 this->car.current_path.S_end[1]};

// Compute the min cost state
string action = ComputeMinCostState(t);
...
```

#### Best State computation

The state machine implemented includes 4 states:

1. Start State;
2. Keep Lane state (KL);
3. Change Lane Left state (CLL);
4. Change Lane Right state (CLR);

![alt tag](https://github.com/ciabo14/SelfDrivingCarND_PathPlanner/blob/master/images/stateMachine.PNG)

The transaction between the states are regulated by the cost function computation. The cost function is vary simple and basically depens on the distance between between the car, and the vehicles in the other lanes. The idea is that the distance with the other cars is stricly dependent on the speed of the main car and of the other cars. The highest is the distance, the faster I can drive; the smaller is the distance, the slower I can drive. 
For the lane where the car is placed, the cost function only depends from the distance from the car in front.
```cpp
...
if (forward_dist != 0.0)
{
    cost = KL_COST_FACTOR / forward_dist;
}
return cost;
...
```
On the other hand, for other lanes, we use for cost function a combination of both the distance with the car in front and with the car behind. This let us to prevent change of line when a car behind is too close. 
Note that the cost function for the change lane takes into consideration also the speed of the car behind. Infact, in case the car behind is driving slower than our car and is far at least as the distance defined by the *BACKWARD_COLLISION_BUFFER*, the cost component relative to this other vehicle is not considered.

```cpp
...
// Compute the cost for the CL state using the relative coefficient
if (forward_dist != 0.0 && backward_dist != 0.0)
{	
	cost = CL_COST_FACTOR_F / forward_dist;
	// Does not consider the back vehicle in case it is going slower and is far more than the 
	// BACKWARD_COLLISION_BUFFER distance
	if(b_v > car.car_speed || backward_dist < BACKWARD_COLLISION_BUFFER)
		cost += CL_COST_FACTOR_R / backward_dist;
}
return cost;
...
```

State with smaller cost is returned and goal state defined accordingly.

#### Target goal definition

Goal definition is the core of the planning and is made by the logic which, starting from the min cost state, compute the target position, speed and acceleration in Frenet coordinates. 

The goal definition includes the application of all the constraints used to avoid collision, speed limits overcome and improvvise and not desired accelerations. 

##### Constraints

First constraint implemented is the limit speed saturation. Every time a goal state is compueted, the target speed is compared to a *MAX_SPEED* and *MIN_SPEED* values. In case the speed computed is higher, the target speed is saturated to these 2 values. 

Therefore, if the car in front is proceeding lower then our car, and the KL is the best action, the car speed need to be reduced in order to guaranteed a fixed forward collision buffer, reaching the same speed of the car in front at maximum at the distance of the buffer.

```cpp
...
// Limit car end_speed up and down in order to avoid to over come the road speed limit or to decelerate too much
if (end_speed > MAX_SPEED)
    end_speed = MAX_SPEED;

if (end_speed < MIN_SPEED)
    end_speed = MIN_SPEED;

...
double target_speed = start_speed + speed_adj;

// Adjust computed target speed decreasing it in order to reach the same speed of the car in front at the distance of 
if(target_speed > vehicle_s_speed){
    if(rough_front_dist > FORWARD_COLLISION_BUFFER*3)
	return target_speed;
    else if(rough_front_dist >= FORWARD_COLLISION_BUFFER)
    {
	return vehicle_s_speed + (target_speed-vehicle_s_speed)* ((rough_front_dist-FORWARD_COLLISION_BUFFER)/(3*FORWARD_COLLISION_BUFFER));
    }
    else if(rough_front_dist < FORWARD_COLLISION_BUFFER){
	return MIN_SPEED + (vehicle_s_speed - MIN_SPEED)* ((rough_front_dist)/(FORWARD_COLLISION_BUFFER));
    }
}
return target_speed;
...
```

Moreover, in order to avoid jurky behaviour, also acceleration is limited. In this case, instead of limiting the acceleration, we directly reduce the maximum speed variance between the start_speed and the end_speed for the goal states using an experimental coefficient.

```cpp
...
// Compute speed adjustment starting from the current distance from the vehicle in front and the 
// MIN/MAX car accelerations
double speed_adj  = KL_ACC_COEFF * rough_front_dist;

speed_adj = speed_adj > MAX_SPEED_VAR ? MAX_SPEED_VAR : speed_adj;
speed_adj = speed_adj < MIN_SPEED_VAR ? MIN_SPEED_VAR : speed_adj;
...
```
These countermeasure appears to be very strong both in avoid collision approaching a vehicle in front, both when a car enters the lane the car is driving with small buffer distance.

#### Trajectory compuation

Once the goal status is computed, the min jerk trajectory is computed using the polinomial derivatives logic. 
The *ComputeMinimumJerk* and *ComputeMinimumJerkPath*, computes polinomio coefficients and Frenet coordinates that are then converted in global map coordinates and sent to the simulator to be applied.


## 2. Further impementations

**Other vehicles behavioud prediction**: in order to make the min cost state computation finer, one way will be to include a logic to predict the behaviour of other cars from the sensor fusion data. Knowing which is the most probable maneuver from cars in the other lanes, could be helpful to prevent collision and to takes decision toword min cost (or faster reaching of the target).

**Finest car speed adjustment for collision avoidance**: one way to made the collision avoidance smarter could be to compute the speed the car should have at the end of the computed trajectory, in order to arrive to have the same speed of the car in front with a fixed buffer distance.

**Curve speed management**: reducing a little bit the speed in the curves could help to reduce the jerk of the trajectory;

**Lane change refinement**: when a lane change is decided, could maybe be better to increase from the start the speed if the car was following the car in front for collision avoidance.

## 3. Dependencies

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

## 4. Basic Build Instructions
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## 5. Environment details
The project was developed in the Eclipse Keples environment, under the linux distribution Ubuntu 17.04.

## 6. Simulator data 

The simulator provide some data about the car localization and the environment itself. Below the list of information provided by the simulator:

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## 7. Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.
