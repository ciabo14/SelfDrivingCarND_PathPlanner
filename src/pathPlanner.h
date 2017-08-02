#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <vector>
#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include "mainVehicle.h"
#include "roadMap.h"

// MAX_SPEED and MIN_SPEED are both in m/s and are defined in order to avoid speed limit overcame
#define MAX_SPEED 20
#define MIN_SPEED 10

#define KL_ACC_COEFF 4.0
#define KL_DIST_THR 30.0

#define MIN_TRACKING_CHANGE    -4.0
#define MAX_TRACKING_CHANGE     4.0

#define PATH_PLAN_SECONDS       2.5
#define PATH_PLAN_INCREMENT     0.02

#define MAX_TRACK_S             6945.554

#define LANE_CHANGE_COST_SIDE_F 0.9
#define LANE_CHANGE_COST_SIDE_R 0.5
#define LANE_CHANGE_COST_AHEAD  1.0

#define FORWARD_COLLISION_BUFFER 4.0

using namespace std;

struct telemetry 
{
    int    lane;
    double car_s;
    double car_speed;
};

class PathPlanner{
    
    public:

    PathPlanner(double lw, int nol, int trj_len, int prev_trj_len);
    virtual ~PathPlanner();

    MainVehicle car;
    double laneWidth;
    int numberOfLanes; 
    int horizon; 
    int update_timestamp;
    RoadMap road_map;
    vector<double> currentX;
    vector<double> currentY;

    vector<double> currentD;
    vector<double> currentS;
    
    void InitalizeMap(string map_file, double max_s);
    void ComputeNextTrajectory(string s);
    void UpdateMainVehicle(double x, double y, double s, double d, double yaw, double speed, 
                           vector<vector<double>> sensor_fusion);
    void ComputeSensorFusion();

    /**********************************
    * Methods for best trajectory identification
    **********************************/
    
    Path ComputeStupidTrajectory(vector<double> previous_path_x, vector<double> previous_path_y,
								 double car_s, double car_d, double yaw);
    string ComputeMinCostState(telemetry t);
    double KLStateCost(telemetry t);
    double CLLStateCost(telemetry t);
    double CLRStateCost(telemetry t);

    Path ComputeKLPath(telemetry t);
    Path ComputeCLLPath(telemetry t);
    Path ComputeCLRPath(telemetry t);

    void DistanceToCarsInLane(int lane, OtherVehicle *f_v, OtherVehicle *b_v,
									   double *f_d, double *b_d);
    void ComputeMinimumJerkPath(Path *p);
    vector<double> ComputeMinimumJerk(vector<double> start, vector<double> end);
    double Lane2D(int lane);
    int D2lane(double d);
    double AdjustDistance2Speed(double start_speed, double rough_front_dist, 
								OtherVehicle forward_vehicle);
    double ComputeCarSpeed(OtherVehicle v);

    double distance(double x1, double y1, double x2, double y2);
    int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);
    int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
    vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
    vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

};
#endif /* PATHPLANNER_H_ */