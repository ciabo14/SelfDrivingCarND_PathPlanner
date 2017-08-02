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

#### Best State computation



#### Target goal definition

#### Trajectory compuation

##### Constraints


## 2. Dependencies

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

## 3. Basic Build Instructions
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## 4. Environment details
The project was developed in the Eclipse Keples environment, under the linux distribution Ubuntu 17.04.

## 5. Simulator data 

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

## 6. Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## 6. Further impementations (WIP)
