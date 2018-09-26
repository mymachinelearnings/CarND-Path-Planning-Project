#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
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

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
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
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
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

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

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

  int my_car_lane = 1; //2nd lane

  double ref_vel = 0; //mph


  h.onMessage([&ref_vel, &my_car_lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;

	  //Having these declarations here is working fine
//	  int lane = 1; //2nd lane
//	  double ref_vel = 49.5; //mph
	  if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
  		  //cout << "got telimetry with vel, lane as " << ref_vel << ", " << my_car_lane << endl;

        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	//prev_size is the number of points left out by the car yet. This is given by the simulator
          	int prev_size = previous_path_x.size();


          	/*
          	 * COLLISSION AVOIDANCE
          	 * Determine the cars in front of you in the same lane
          	 * If the distance is less than 30m, then decrease the speed to a certain value initially
          	 *
          	 */


          	if(prev_size > 0) {
          		car_s = end_path_s;
          	}
          	/////////////////// PREDICTION START ///////////////////////
          	/*
          	 * These variables are used to identify if car is in the current, left or right lanes
          	 * within 30 m which is considered to be a safe distance
          	 */
          	bool car_ahead = false;
          	bool car_left = false;
          	bool car_right = false;
          	bool quick_break = false;

          	// Loop through the sensor fusion data
          	// format :: [id, x, y, vx, vy, s, d]

          	for(int i=0; i<sensor_fusion.size(); i++) {

          		//Evaluate where the car is
          		int other_car_lane;

          		float d = sensor_fusion[i][6];

          		//This determines the lane of the ith car from sensor fusion data
          		if(d > 0 && d < 4) {
          			other_car_lane = 0;
          		} else if (d > 4 && d < 8) {
          			other_car_lane = 1;
          		} else if (d > 8 && d < 12) {
          			other_car_lane = 2;
          		} else {
          			continue;
          		}

          		//Each lane size is 4 m, we are in middle lane
          		// so the d should be > 4(-2) or < 8(+2) to be in our lane. +2 is for the middle of the lane
				double other_car_vx = sensor_fusion[i][3];
				double other_car_vy = sensor_fusion[i][4];
				double other_car_vel = sqrt((other_car_vx*other_car_vx) + (other_car_vy*other_car_vy));

				double other_car_s = sensor_fusion[i][5];

				//Predict the next car's next instance
				other_car_s += ((double)prev_size * 0.02 * other_car_vel);

				//Check if ith car is in which lane, and calculate the dist between my car and that car and set appropriate lane car variable
				//If other car is very near, then decrease speed much faster
				if(other_car_lane == my_car_lane) {
					car_ahead = car_ahead || (other_car_s - car_s > 0 && other_car_s - car_s < 30);
					if(other_car_s - car_s < 10) quick_break = true;
				} else if(other_car_lane == my_car_lane - 1) {
					car_left = car_left || (other_car_s - car_s > -20 && other_car_s - car_s < 30);
					if(other_car_s - car_s < 10) quick_break = true;
				} else if(other_car_lane == my_car_lane + 1) {
					car_right = car_right || (other_car_s - car_s > -20 && other_car_s - car_s < 30);
					if(other_car_s - car_s < 10) quick_break = true;
				}
          	}
          	/////////////////// PREDICTION END ///////////////////////

      		//////////////////  BEHAVIOUR PLANNING START //////////////////



				/*
				 * Now that we know where the cars are
				 * evaluate the final lane of the car based on the flags determined
				 *
				 * if there's a car ahead
				 * 		check if LEFT LANE is free and change lane, else
				 * 		check if RIGHT LANE is free and change lane, else
				 * 		slow down
				 *
				 * 	Always be on middle lane to be faster and at the same time giving space to overtaking vehicles on the left lane
				 * 	so, if the vehicle is not in MIDDLE LANE,
				 * 		check if vehicle is in LEFT LANE, and if middle lane is free, move to middle lane
				 * 		OR
				 * 		check if vehicle is in RIGHT LANE, and if middle lane is free, move to middle lane
				 *
				 *
				 */
          		double MAX_VELOCITY = 49.5;
          		double MAX_ACC = 0.224;
				if(car_ahead) {

					if(my_car_lane > 0 && !car_left) {
						cout << "	<<<<<< Turn LEFT  <<<<<< from Lane " << my_car_lane << " to Lane ";
						my_car_lane--;
						cout << my_car_lane << endl;
					} else if(my_car_lane < 2 && !car_right) {
						cout << "	>>>>>> Turn RIGHT >>>>>> from Lane " << my_car_lane << " to Lane ";
						my_car_lane ++;
						cout << my_car_lane << endl;
					} else {
						if(quick_break) {
							//cout << "Car too close, Slow down fast!!!";
							ref_vel -= 3*MAX_ACC;
						} else {
							ref_vel -= MAX_ACC;
						}
					}

				} else {
					if(my_car_lane != 1) {
						if ((my_car_lane == 0 && !car_right) || (my_car_lane == 2 && !car_left)) {
							cout << "	|||||| Its better to be in MIDDLE lane!" << endl;
							my_car_lane = 1;
						}
					}

					if(ref_vel < MAX_VELOCITY) {
						//If no car ahead, accelerate faster to reach max velocity
						if((my_car_lane == 1 && !car_ahead) || (my_car_lane == 0 && !car_left) || (my_car_lane == 2 && !car_right)) {
							ref_vel += 2*MAX_ACC;
						} else {
							ref_vel += MAX_ACC;
							//cout << "Too Slow, increasing the speed! " << ref_vel <<  "\n";
						}

					}
				}

	      		//////////////////  BEHAVIOUR PLANNING END //////////////////


         	/*
          	 * Here's the process
          	 * First, get the prev points and the one before that
          	 * If there are no prev points - meaning if this is the first or 2nd moment in time, then generate them using math
          	 * prev x is same as current car x, the one before that is x' = x - dt. cos
          	 * prev y is same as current car x, the one before that is y' = y - dt. sin
          	 *
          	 * call these points A (the one before previous), B(previous)
          	 * call B as reference since that's your most recent point
          	 * add them to ptsx, ptsy
          	 *
          	 * Now, inorder to generate a smooth line starting from A, imagine 3 anchor points way ahead - like 30, 60, 90 m respectively
          	 * call these C, D, E respectively
          	 *
          	 * Now, your task is to generate a trajectory along these points and use spline to get all the points at every 20 ms span
          	 *
          	 *
          	 */

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	vector<double> ptsx;
          	vector<double> ptsy;

          	double ref_x;
			double ref_y;
			double ref_yaw;

			double prev_ref_x;
          	double prev_ref_y;

          	if(prev_size < 2) {

          		ref_x = car_x;
          		ref_y = car_y;

          		prev_ref_x = ref_x - cos(car_yaw);
          		prev_ref_y = ref_y - sin(car_yaw);

          		ref_yaw = deg2rad(car_yaw);

          	} else {

          		ref_x = previous_path_x[prev_size - 1];
          		ref_y = previous_path_y[prev_size - 1];

          		prev_ref_x = previous_path_x[prev_size - 2];
          		prev_ref_y = previous_path_y[prev_size - 2];

          		ref_yaw = atan2(ref_y - prev_ref_y, ref_x-prev_ref_x);

          	}

          	ptsx.push_back(prev_ref_x);
			ptsx.push_back(ref_x);

			ptsy.push_back(prev_ref_y);
			ptsy.push_back(ref_y);

			//Now generating the next 3 points

			vector<double> next_anchor_0 = getXY(car_s + 30, (4*my_car_lane + 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_anchor_1 = getXY(car_s + 60, (4*my_car_lane + 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_anchor_2 = getXY(car_s + 90, (4*my_car_lane + 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);

			ptsx.push_back(next_anchor_0[0]);
			ptsx.push_back(next_anchor_1[0]);
			ptsx.push_back(next_anchor_2[0]);

			ptsy.push_back(next_anchor_0[1]);
			ptsy.push_back(next_anchor_1[1]);
			ptsy.push_back(next_anchor_2[1]);

			/*
			 * Now that we have all 5 points - prev 2 and next 3
			 * lets move the frame of reference so that the origin is at point B
			 * the amount you want to shift is basically by point B, so subtract each point by point B to get the shift
			 * once you have the shift, since the frame of reference also should be turned, the effective pints will be
			 * x' = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw)
			 * y' = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw)
			 */

			for(int i=0; i<5; i++) {
				double shift_x = ptsx[i] - ref_x;
				double shift_y = ptsy[i] - ref_y;

				//Overriding with the shifted params
				ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
				ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);

			}

			//creating a line through these points

			tk::spline s;
			s.set_points(ptsx, ptsy);
			//cout << "Spline set!" << endl;

			/*
			 * Once the frame of reference and angle are transformed,
			 * 1. Add the left over points to next_x_vals, next_y_vals
			 * 2. Interpolate the points on the spline generated to get the remaining points(50 - prev_size) to make the total as 50 points for every 0.02 sec
			 * 0.02 is the time the simulator gets the input from the code, 2.24 is the conversion factor from mph to kmph
			 *
			 */

			for(int i=0; i < prev_size; i++) {
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}
			//cout << "Prev left over points added!" << next_x_vals.size() << endl;

			/*
			 * Now that we've added the left over points, add the rest to make total points as 50
			 * For this, we interpolate the spline and get those points
			 * First, get the x values so that spline can give corresponding y values
			 *
			 * Given a triangle with s on x axis, d as hypotenuse,
			 * find the number of points that would be traversed within it in every 0.02 sec timeframe
			 * n * 0.02 * vel(in kmph) = d
			 *
			 */

			double target_x = 30.0;
			double target_y = s(target_x);
			double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

			double x_add_on = 0;

			for(int i=1; i<= 50 - prev_size; i++) {

				double N = (target_dist / (.02 * ref_vel / 2.24));
				double x_point = x_add_on + (target_x/N);
				double y_point = s(x_point);

				x_add_on = x_point;
				double x_temp = x_point;
				double y_temp = y_point;

				// Rotate back to normal after rotating it earlier

				//cout << "points before transformation :: " << x_point << y_point << endl;
				/*
				 * Rotating back to original frame of reference
				 * This involves angle rotation followed by shift addition
				 * Major change here as during shifting, i was using the temp variable above instead of the actual variable with which it was shifter earlier
				 *
				 */

				x_point = (x_temp * cos(ref_yaw) - y_temp*sin(ref_yaw));
				y_point = (x_temp * sin(ref_yaw) + y_temp*cos(ref_yaw));

				x_point += ref_x;
				y_point += ref_y;



				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);
				//cout << "points after transformation :: " << x_point << y_point << endl;


			}

//          	cout << "next_x_vals - size ::  " << next_x_vals.size() << ", ";
//          	cout << "next_y_vals - size ::  " << next_y_vals.size() << "\n";
//          	for(int i=0; i<next_x_vals.size(); i++) {
//          		cout << next_x_vals[i] << " - " << next_y_vals[i] << ", ";
//
//          	}

          	/*
          	 * Predict the car s and d co-ordinates based on math models
          	 * Convert them to x and y co-ordinates
          	 * This implementation is basic to check if the car is keeping the lane, but this doesn't give smooth transitions
          	 * For smoother transitions, you'll need jerk minimization techniques(polynomial fit, or spline)

          	double dist_inc = 0.3; //0.5 m per sec corresponds to 50 kmph
          	for(int i=0; i< 50; i++) {

          		double next_s = car_s + (i+1) * dist_inc;
          		//Lane is 4 m wide, and we are in middle lane, so we are 1.5 times the lane width which means 6 m. Let's be in the 2nd lane always
          		double next_d = 6;

          		vector<double> next_xys = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          		next_x_vals.push_back(next_xys[0]);
          		next_y_vals.push_back(next_xys[1]);
          	}

          	cout << "next_x_vals - size ::  " << next_x_vals.size() << ", ";
          	cout << "next_y_vals - size ::  " << next_y_vals.size() << "\n";
          	for(int i=0; i<next_x_vals.size(); i++) {
          		cout << next_x_vals[i] << " - " << next_y_vals[i] << ", ";

          	}

          	cout << "\n\n";
			*/



          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	json msgJson;
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
