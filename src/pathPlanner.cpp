#include <vector>
#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <map>
#include "json.hpp"
#include "pathPlanner.h"
#include "mainVehicle.h"
#include "roadMap.h"
#include "path.h"
#include <utility>
#include "Eigen-3.3/Eigen/Dense"

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

using json = nlohmann::json;

using namespace std;
using namespace Eigen;

PathPlanner::PathPlanner(double lw, int nol, int horizon, int update_timestamp) 
{

    this->laneWidth = lw;
    this->numberOfLanes = nol;
	
	// Used initially for the 
    this->horizon = horizon;
    this->update_timestamp = update_timestamp;
    
	car = MainVehicle(-1, .0, .0, .0, .0, .0, .0);
}

PathPlanner::~PathPlanner() {}

/**************************
 * Initialize the road_map object of the path planner instance, starting from waypoints defined in the file named map_file
 **************************/
void PathPlanner::InitalizeMap(string map_file, double max_s)
{

    cout << "----------INITIALITATION OF THE MAP----------"<<endl;
    
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Load map values for waypoint's x,y,s and d normalized normal vectors from file 
	ifstream in_map_(map_file.c_str(), ifstream::in);
	string line;
	while (getline(in_map_, line)) {
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}
	
	// Add one more point to the map in order to ajust transiction between the end and the start of the track
	map_waypoints_x.push_back(map_waypoints_x[0]);
	map_waypoints_y.push_back(map_waypoints_y[0]);
	map_waypoints_s.push_back((double)MAX_TRACK_S);
	map_waypoints_dx.push_back(map_waypoints_dx[0]);
	map_waypoints_dy.push_back(map_waypoints_dy[0]);	
	
	//Initialize the roadMap object with with the waypoints form the csv file
	this->road_map = RoadMap(map_waypoints_x,map_waypoints_y,map_waypoints_s);
	this->road_map.InitSpline();
    this->road_map.RefineSpline();
}

/**************************
* Compute next trajectory to be applyed from the data coming from the simulator. The function:
* 1. Check the min cost next state (action) between keep lane (KL), change lane left (CLL) and change lane right (CLR);
* 2. Compute the target speed and acceleration to be applied;
* 3. Compute the min jerk trajectory.
***************************/
void PathPlanner::ComputeNextTrajectory(string s)
{
    
    auto j = json::parse(s);
    //Update current car status (x,y,s,d,yaw and speed) and the data coming from sensor fusion
    auto sensor_fusion = j[1]["sensor_fusion"];
    UpdateMainVehicle(j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"], j[1]["yaw"], j[1]["speed"],
                      sensor_fusion);

    // Previous path data given to the Planner
    vector<double> previous_path_x = j[1]["previous_path_x"];
    vector<double> previous_path_y = j[1]["previous_path_y"];
    // Previous path's end s and d values 
    double end_path_s = j[1]["end_path_s"];
    double end_path_d = j[1]["end_path_d"];

	Path target_path;
	//target_path = IncludePrevPathInfo(vector<double> previous_path_x, vector<double> previous_path_y)

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
	{
		// The telemetry object is used to simplfy the code
		telemetry t {D2lane(this->car.current_path.D_end[0]),this->car.current_path.S_end[0],
					 this->car.current_path.S_end[1]};
		
		// Compute the min cost state
		string action = ComputeMinCostState(t);
		// Compute the target to accomplish the min cost state
		if (action == "KL")
			target_path = ComputeKLPath(t);
		if (action == "CLL")
			target_path = ComputeCLLPath(t);
		if (action == "CLR")
			target_path = ComputeCLRPath(t);

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
	}

	// assign the current computed path to the car current_path
	car.current_path = target_path;
}

/**************************
 * Update the main car vehicle using data coming from the simulator   
 **************************/
void PathPlanner::UpdateMainVehicle(double x, double y, double s, double d, double yaw, double speed, 
                                    vector<vector<double>> sensor_fusion)
									{
    
    // Update main car's localization Data
    car.UpdateMainVehicleStatus(x, y, s, d, yaw, speed);
    car.ComputeCurrentLane(this->laneWidth);
    // Update sensor fusion data
    car.UpdateSensorFusion(sensor_fusion);

}

/***************************
* Compute the state with min cost between KL, CLL and CLR 
***************************/
string PathPlanner::ComputeMinCostState(telemetry t)
{
	// Compute the cost for the KL state
	double KL_cost  = KLStateCost(t);
	double CLL_cost = 1e9;
	double CLR_cost = 1e9;
	// Compute t cost of the CLL and CLR states only if not in the border lanes (1 and 3 respectively)
	if(t.lane != 1)
        CLL_cost  = CLStateCost(t, t.lane-1);
	if(t.lane != 3)
    	CLR_cost = CLStateCost(t, t.lane+1);
	
    cout << "costs: KL->" << KL_cost << " - CLL->" << CLL_cost << " - CLR->" << CLR_cost << endl;

	if(KL_cost <= CLL_cost && KL_cost <= CLR_cost)
		return "KL";
	else if(CLL_cost <= CLR_cost)
		return "CLL";
	return "CLR";
}

/***************************
* Compute the Keelp Lane state cost from current position
***************************/
double PathPlanner::KLStateCost(telemetry t)
{
	double cost = 100000;

	OtherVehicle forward_vehicle, backword_vehicle;
	double forward_dist, backword_dist;
	// Compute current distance to vehicle in front and behind in the current vehicle lane
	DistanceToCarsInLane(t.lane, &forward_vehicle, &backword_vehicle,
						 &forward_dist, &backword_dist);

	//cout << "Current Lane Distances: " << forward_dist  << backword_dist << endl;
    // Compute the cost for the KL state using the relative coefficient
	if (forward_dist != 0.0)
    {
        cost = KL_COST_FACTOR / forward_dist;
    }
    return cost;
}
/***************************
* Compute the Change Lane Left/Right cost from current position 
***************************/
double PathPlanner::CLStateCost(telemetry t, int lane)
{	
	double cost = 100000;
	OtherVehicle forward_vehicle, backward_vehicle;
	double forward_dist, backward_dist;
	// Compute current distance to vehicle in front and behind in the lane at left of the vehicle
	DistanceToCarsInLane(lane, &forward_vehicle, &backward_vehicle,
						&forward_dist, &backward_dist);

	// Compute the cost for the CL state using the relative coefficient
	if (forward_dist != 0.0 && backward_dist != 0.0)
	{	
		cost = CL_COST_FACTOR_F / forward_dist + CL_COST_FACTOR_R / backward_dist;
	}
	return cost;
}

/**************************
* Compute a new waypoints list for the keep lane state.
**************************/
Path PathPlanner::ComputeKLPath(telemetry t)
{
	cout << "COMPUTE KL PATH" << endl;
	Path next_path;
	OtherVehicle forward_vehicle, backward_vehicle;
	double forward_dist, backward_dist;
	
	// Use the position of the sensor fusion data to compute the distance of the cars in the same lane
	DistanceToCarsInLane(t.lane, &forward_vehicle, &backward_vehicle,
						 &forward_dist, &backward_dist);

    double start_speed = t.car_speed;

	// Limit car start speed in order to avoid to overcome the road speed limit
    if (start_speed > MAX_SPEED)
        start_speed = MAX_SPEED;
	
	// Compute the end speed accordingly to the current start_speed and to the vehicle in front/rear in the target lane
    double end_speed = AdjustDistance2Speed(start_speed, forward_dist, forward_vehicle);;

	// Limit car end_speed up and down in order to avoid to over come the road speed limit or to decelerate too much
    if (end_speed > MAX_SPEED)
        end_speed = MAX_SPEED;

    if (end_speed < MIN_SPEED)
        end_speed = MIN_SPEED;
	
	// Compute the end s_position accordingly to the physical equation
	double s_end = t.car_s + PATH_HORIZON * 0.5 * (start_speed + end_speed);
	
	// Initialize new_path
	next_path = Path({t.car_s, start_speed, 0}, 
				{s_end, end_speed, 0}, 
				{Lane2D(t.lane), 0, 0},
				{Lane2D(t.lane), 0, 0});
	// Compute min jerk trajectory
	ComputeMinimumJerkPath(&next_path);
	return next_path;
}

/**************************
* Compute a new waypoints list for the change lane left state.
**************************/
Path PathPlanner::ComputeCLLPath(telemetry t)
{
	cout << "COMPUTE CLL PATH" << endl;
	double s_end = t.car_s + PATH_HORIZON * t.car_speed;
	
	// Initialize new path considering as target speed the same speed the car currently have (no acceleration/deceleration in changin lane)
	Path next_path = Path({t.car_s, t.car_speed, 0}, 
					 {s_end, t.car_speed, 0}, 
					 {Lane2D(t.lane), 0, 0},
					 {Lane2D(t.lane-1), 0, 0});

	// Compute min jerk trajectory
	ComputeMinimumJerkPath(&next_path);
	return next_path;
}

/**************************
* Compute a new waypoints list for the change lane right state.
**************************/
Path PathPlanner::ComputeCLRPath(telemetry t)
{
	cout << "COMPUTE CLR PATH" << endl;
	double s_end = t.car_s + PATH_HORIZON * t.car_speed;

	// Initialize new path considering as target speed the same speed the car currently have (no acceleration/deceleration in changin lane)
	Path next_path = Path({t.car_s, t.car_speed, 0}, 
					 	  {s_end, t.car_speed, 0}, 
					 	  {Lane2D(t.lane), 0, 0},
					 	  {Lane2D(t.lane+1), 0, 0});

	// Compute min jerk trajectory
	ComputeMinimumJerkPath(&next_path);
	return next_path;
}

/**************************
* Compute the target speed starting from the start_speed, the distance to the front vehicle and the acc. parameters
**************************/
double PathPlanner::AdjustDistance2Speed(double start_speed, double rough_front_dist, 
										 OtherVehicle forward_vehicle)
{
	// Compute car speed for the forward vehicle
	double vehicle_s_speed =  ComputeCarSpeed(forward_vehicle);
	
	// Compute speed adjustment starting from the current distance from the vehicle in front and the 
	// MIN/MAX car accelerations
	double speed_adj  = KL_ACC_COEFF * rough_front_dist;
	
	speed_adj = speed_adj > MAX_SPEED_VAR ? MAX_SPEED_VAR : speed_adj;
	speed_adj = speed_adj < MIN_SPEED_VAR ? MIN_SPEED_VAR : speed_adj;
	
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
}

/**************************
* Compute an OtherVehicle speed starting from the data received from the sensor fusion
**************************/
double PathPlanner::ComputeCarSpeed(OtherVehicle v)
{
	// Compute car heading and road heading in order to adjust speed relatively to the current driving of the car
	double car_heading  = rad2deg(atan(v.vy/v.vx));
	
	
	int waypoint0 = ClosestWaypoint(v.x,v.y,road_map.fined_x,road_map.fined_y);
	int waypoint1 = NextWaypoint(v.x,v.y,car_heading,road_map.fined_x,road_map.fined_y);
	double lane_heading = rad2deg(atan((road_map.fined_y[waypoint1] - road_map.fined_y[waypoint0]) / 
						  (road_map.fined_x[waypoint1] - road_map.fined_x[waypoint0])));
	double delta_theta = car_heading - lane_heading;
	
	// Compute v_s and v_d speed 
	
	double mag_v = sqrt(pow(v.vx,2) + pow(v.vy,2));
	double v_s = mag_v * cos(delta_theta);
	double v_d = mag_v * sin(delta_theta);
	
	//return v_s;
	return mag_v;
}



/**************************
* Compute the distance to the front and behind cars in a specified lane respect to the current car position
**************************/
void PathPlanner::DistanceToCarsInLane(int lane, OtherVehicle *f_v, OtherVehicle *b_v,
									   double *f_d, double *b_d)
{
    double closest_front = 100000;
	double closest_behind = 100000;
	
	for (std::map<int,OtherVehicle>::iterator it=this->car.close_vehicles.begin(); it!=this->car.close_vehicles.end(); ++it)
    {
		OtherVehicle close_v = (OtherVehicle)it->second;
		// Choose only close vehicle in the desired lane
		if (D2lane(close_v.d) == lane)
        {	
			// Compute the smallest front distance
            double diff = close_v.s - this->car.s;
            if (diff > 0.0 && diff < closest_front){
				*f_v = close_v;
				closest_front = diff;
			}
            // Compute the smallest behind distance
			if (diff < 0.0 && -diff < closest_behind){
				*b_v = close_v;
				closest_behind = -diff;
			}
        }
    }
	*f_d = closest_front;
	*b_d = closest_behind;
}

/**************************
* Compute the d value respect to a specified lane
**************************/
double PathPlanner::Lane2D(int lane)
{
	double d = (double)(((lane - 1) * this->laneWidth)+(this->laneWidth/2)); 
	if (lane == 1)
		d += 0.1;
	if (lane == 3)
		d -= 0.1;
    return d;
}

/**************************
* Compute the lane of a specified d value
**************************/
int PathPlanner::D2lane(double d)
{	
	int lane = ceil(d/this->laneWidth);
	
	if(lane >0 & lane <4)
		return lane;
	return 0;
}

/**************************
* Compute minimum jerk path in Frenet coordinates
**************************/
void PathPlanner::ComputeMinimumJerkPath(Path *p)
{
    // Generate minimum jerk path in Frenet coordinates
    vector<double> next_s_vals = ComputeMinimumJerk(p->S_start, p->S_end);
    vector<double> next_d_vals = ComputeMinimumJerk(p->D_start, p->D_end);
    // Convert Frenet coordinates to map coordinates
    vector<double> next_x_vals = {};
    vector<double> next_y_vals = {};
    for (int i=0; i<next_s_vals.size(); i++)
    {
        vector<double> xy = getXY(fmod(next_s_vals[i], MAX_TRACK_S),
                                  next_d_vals[i],
                                  this->road_map.fined_s,
                                  this->road_map.fined_x,
                                  this->road_map.fined_y);
        next_x_vals.push_back(xy[0]);
        next_y_vals.push_back(xy[1]);
    }
	p->X = next_x_vals;
    p->Y = next_y_vals;
    p->last_s = next_s_vals[next_s_vals.size()-1];
    p->last_d = next_d_vals[next_d_vals.size()-1];

}

/**************************
* Execute Min jerk minimization
**************************/
vector<double> PathPlanner::ComputeMinimumJerk(vector<double> start, vector<double> end)
{
    MatrixXd A = MatrixXd(3,3);
    VectorXd b = VectorXd(3);
    VectorXd x = VectorXd(3);

    double t  = PATH_HORIZON;
    double t2 = t * t;
    double t3 = t * t2;
    double t4 = t * t3;
    double t5 = t * t4;

    A <<   t3,    t4,    t5,
         3*t2,  4*t3,  5*t4,
         6*t,  12*t2, 20*t3;

    b << end[0] - (start[0] + start[1] * t + 0.5 * start[2] * t2),
         end[1] - (start[1] + start[2] * t),
         end[2] - start[2];

    x = A.inverse() * b;

    double a0 = start[0];
    double a1 = start[1];
    double a2 = start[2] / 2.0;
    double a3 = x[0];
    double a4 = x[1];
    double a5 = x[2];

    vector<double> result;
    for (double t=PATH_WP_PERIOD; t<PATH_HORIZON+0.001; t+=PATH_WP_PERIOD) 
    {
        double t2 = t * t;
        double t3 = t * t2;
        double t4 = t * t3;
        double t5 = t * t4;
        double r = a0 + a1*t + a2*t2 + a3*t3 + a4*t4 + a5*t5;
        result.push_back(r);
    }
    return result;
}


/**************************
 * Function used to compute a "stupid" trajectory by which the car only follows the lane where it is   
 **************************/
Path PathPlanner::ComputeStupidTrajectory(vector<double> previous_path_x, vector<double> previous_path_y,
										  double car_s, double car_d, double yaw)
{
	Path p;
	int prev_path_size = previous_path_x.size();
	vector<double> car_last_sd{car_s, car_d};
	
	if(prev_path_size > 0){
		
		cout << "Smaller than horizon - timestamp: " << previous_path_x.size()<< endl;
		int prev_trj_used = min((int)(previous_path_x.size()),this->update_timestamp);

		for(int i = 0; i < prev_trj_used; i++){
			p.X.push_back(previous_path_x[i]);
			p.Y.push_back(previous_path_y[i]);
		}

		car_last_sd = getFrenet(previous_path_x[prev_trj_used], previous_path_y[prev_trj_used],
								yaw, this->road_map.fined_x,this->road_map.fined_y);

		cout << car_s << "-" << car_last_sd[0] << endl;
	}

	double dist_inc = 0.30;
	for(int i = 0; i < this->horizon - p.X.size(); i++)
		{	
			vector<double> X_Y = getXY(car_last_sd[0]+(dist_inc*i), +2, road_map.fined_s, road_map.fined_x, road_map.fined_y);
			p.X.push_back(X_Y[0]);
			p.Y.push_back(X_Y[1]);
		}
	
	if (prev_path_size != 0) {
		for (int i = 1 ; i < p.X.size() - prev_path_size; ++i){
			p.X[i ] = p.X[i - 1] + (p.X[i + 1] - p.X[ i ]);
			p.Y[i ] = p.Y[i - 1] + (p.Y[i + 1] - p.Y[i ]);
		}
		
	}
	
	return p;
}

double PathPlanner::distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int PathPlanner::ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int PathPlanner::NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> PathPlanner::getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> PathPlanner::getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};
}

