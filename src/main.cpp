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
#include "helpers.h"

using namespace std;

// for convenience
using json = nlohmann::json;
using std::string;
using std::vector;

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
  
  // 3 LANES HIGHWAY DEFINITION:
  
  //      |   ↓   |   ↓   |   ↓   ||   ↑   |   ↑   |   ↑   |
  //      |   3 oncoming lanes    ||   0   |   1   |   2   |   --> lane no.
  //      |	  |       |       ||	   |       |       |
  //      |	  |       |       |0m	   4m	   8m     12m  --> distances to waypoints in the middle of the double-yellow dividing line in the center of the highway
  //                               
	
  int lane = 1; //start in lane 1
  //bool change_lines = false	
  int lane_change_wp = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&lane_change_wp](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
		
//  ******************************************************************
//  1. READ IN SIMULATOR DATA
//  ******************************************************************
		
          //Main car's localization Data

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
		
            //vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
	    auto sensor_fusion = j[1]["sensor_fusion"];
		
	    // content Sensor Fusion Data: a list of all other car's attributes on the same side of the road. (No Noise)
	    //**************************************************************************************************
	    // sensor_fusion [i]:
	    // [0] --> car's unique ID
	    // [1] --> car's x position in map
	    // [2] --> car's y position in map
	    // [3] --> car's x velocity [m/s]
	    // [4] --> car's y velocity [m/s]
	    // [5] --> car's s position in Frenet coordinates
	    // [6] --> car's d position in Frenet coordinates
	
            // target reference velocity
	    double ref_vel = 49.5; //mph

	    // number of points in previous path
            int prev_size = previous_path_x.size();

            int next_wp = -1;
		
	    // reference x,y, yaw states
	    // either we will reference the starting point as where the car is or at the previous paths end point
            double ref_x = car_x;		//starting point where the car is
            double ref_y = car_y; 		//starting point where the car is
            double ref_yaw = deg2rad(car_yaw); 	//start orientation 

	      // if previous size is almost empty, use the car as starting reference
              if(prev_size < 2)
              {
                next_wp = NextWaypoint(ref_x, ref_y, ref_yaw, map_waypoints_x,map_waypoints_y,map_waypoints_dx,map_waypoints_dy);
              }
	      // else, use last two points of previous path for generating starting reference
              else
              {
	        ref_x = previous_path_x[prev_size-1];  			// x value of last point of previous path
		double ref_x_prev = previous_path_x[prev_size-2];  	// x value of penultimate point of previous path
		ref_y = previous_path_y[prev_size-1];			// y value of last point of previous path
		double ref_y_prev = previous_path_y[prev_size-2];	// y value of penultimate point of previous path
		ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);	// orientation
		      
		next_wp = NextWaypoint(ref_x,ref_y,ref_yaw,map_waypoints_x,map_waypoints_y,map_waypoints_dx,map_waypoints_dy);
		car_s = end_path_s;
		car_speed = (sqrt((ref_x-ref_x_prev)*(ref_x-ref_x_prev)+(ref_y-ref_y_prev)*(ref_y-ref_y_prev))/.02)*2.24;
              }
		
// ***********************************************************
// OTHER CAR IN MY LANE - ADAPT EGO CAR SPEED
// ***********************************************************
		
              //find ref_v to use
              double minDistance = 6945.554; // minDistance = minimal distance between ego car and an other car in my lane in front of me (initial value = length of the track)
              bool change_lanes = false;
              for(int i = 0; i < sensor_fusion.size(); i++)
              {
          	float d = sensor_fusion[i][6];
          	if(d<(2+4*lane+2)&&d>(2+4*lane-2)) //check if there is a car between both lane lines of my lane
          	{
          	  //if there is a car in my lane, check its velocity and position
		  double vx = sensor_fusion[i][3];		//car's x velocity [m/s]
          	  double vy = sensor_fusion[i][4];		//car's y velocity [m/s]
          	  double check_speed = sqrt(vx*vx+vy*vy); 	//car's velocity [m/s]
			
          	  double check_car_s = sensor_fusion[i][5]; 	//car's s position in Frenet coordinates
          	  check_car_s+=((double)prev_size*.02*check_speed); //if using previous points can project s value out
			
          	  //check s values greater than mine and s gap
          	  if((check_car_s>car_s)&&((check_car_s-car_s)<30)&&((check_car_s-car_s)<minDistance)) //check if there is a car inbetween 30m in front of me
          	  {  
          	    minDistance = (check_car_s - car_s); // minDistance = minimal distance between me and an other car in my lane in front of me

          	    if((check_car_s-car_s) > 20)
          	    {
         	      //match that cars speed
          	      ref_vel = check_speed*2.24;  //instead of 2.237
          	      change_lanes = true;
          	    }
          	    else
          	    {
		      //go slightly slower than the cars speed
          	      ref_vel = check_speed*2.24-5;
          	      change_lanes = true;
          	    }
          	  }
          	}
              }
		
// ***********************************************************
// OTHER CAR IN MY LANE - IF POSSIBLE EXECUTE LANE CHANGE
// ***********************************************************
	
              //try to change lanes if too close to car in front
              if(change_lanes && ((next_wp-lane_change_wp)%map_waypoints_x.size() > 2))
              {
                bool changed_lanes = false;
		      
		// * * * * * * * * * * * * * * * * *
          	// IF POSSIBLE CHANGE TO LEFT LANE
		// * * * * * * * * * * * * * * * * *
		      
          	if(lane != 0 && !changed_lanes)  //if ego is not on lane no. 0 and changed_lanes = false...
          	{
          	  bool lane_safe = true;
          	  for(int i = 0; i < sensor_fusion.size(); i++)
          	  {
          	    //car is in left lane
          	    float d = sensor_fusion[i][6];
          	    if(d<(2+4*(lane-1)+2)&&d>(2+4*(lane-1)-2)) //check if there is a car between both lane lines of the lane one left to mine
          	    {
          	      double vx = sensor_fusion[i][3];
          	      double vy = sensor_fusion[i][4];
          	      double check_speed = sqrt(vx*vx+vy*vy);

  		      double check_car_s = sensor_fusion[i][5];
          	      check_car_s+=((double)prev_size*.02*check_speed);
          	      double dist_s = check_car_s-car_s;
          	        if(dist_s < 20 && dist_s > -20) // if there is a car one lane to the left within the range of 20m behind and in front of my car...
          		{
          		  lane_safe = false;	// .... lane change is unsafe
          		}
        	    }
		  }
          	  if(lane_safe)
          	  {
          	    changed_lanes = true;
		    lane -= 1;
          	    lane_change_wp = next_wp;
          	  }
          	}
		      
		// * * * * * * * * * * * * * * * * *
          	// IF POSSIBLE CHANGE TO RIGHT LANE
		// * * * * * * * * * * * * * * * * *
		      
          	if(lane != 2 && !changed_lanes)  //if ego car is not on lane no. 2 and changed_lanes = false...
          	{
          	  bool lane_safe = true;
          	  for(int i = 0; i < sensor_fusion.size(); i++)
          	  {
          	    //car is in right lane
          	    float d = sensor_fusion[i][6];
          	    if(d<(2+4*(lane+1)+2)&&d>(2+4*(lane+1)-2))) //check if there is a car between both lane lines of the lane one left to mine
          	      { 
          		double vx = sensor_fusion[i][3];
          		double vy = sensor_fusion[i][4];
          		double check_speed = sqrt(vx*vx+vy*vy);

          		double check_car_s = sensor_fusion[i][5];
          		check_car_s+=((double)prev_size*.02*check_speed);
          		double dist_s = check_car_s-car_s;
          		if(dist_s < 20 && dist_s > -10) // if there is a car one lane to the left within the range of 20m behind and in front of my car...
          		{
          		  lane_safe = false;	// .... lane change is unsafe
          		}
          	      }
          	  }
          	  if(lane_safe)
          	  {
          	    changed_lanes = true;
          	    lane += 1;
          	    lane_change_wp = next_wp;
          	  }
          	}
              }
		
//  ******************************************************************
//  Trajectory
//  ******************************************************************		
		
              // Creating a list of widely spaced (x,y) waypoints, evenly spaced at 30m
	      // later we will interpolate these waypoints with a spline and fill it in with more points that control speed

              vector<double> ptsx;
              vector<double> ptsy;

	      // if previous size is almost empty, use the car as starting reference
 	      if(prev_size < 2)
              {
          	//Use two points that make the path tangent to the car
		double prev_car_x = car_x - cos(car_yaw);
          	double prev_car_y = car_y - sin(car_yaw);

          	ptsx.push_back(prev_car_x);
          	ptsx.push_back(car_x);

          	ptsy.push_back(prev_car_y);
          	ptsy.push_back(car_y);
              }
	      // use the last couple of points in the previous path that the car was following as starting reference
              else
              {
          	//Redefine reference state as previous path end point
		ref_x = previous_path_x[prev_size-1];
		ref_y = previous_path_y[prev_size-1];
			
		double ref_x_prev = previous_path_x[prev_size-2];
		double ref_y_prev = previous_path_y[prev_size-2];
		// ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
		      
		// Use two points that make the path tangent to the previous path's end point
		ptsx.push_back(ref_x_prev);
          	ptsx.push_back(ref_x);

          	ptsy.push_back(ref_y_prev);
          	ptsy.push_back(ref_y);
              }
	      //In Frenet add evenly 30m spaced anchor points ahead of the starting reference
              vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
              vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
              vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

              ptsx.push_back(next_wp0[0]);
              ptsx.push_back(next_wp1[0]);
              ptsx.push_back(next_wp2[0]);  // the vector includes 5 x points: the two previous point + 3 locations in 30, 60 and 90m

              ptsy.push_back(next_wp0[1]);
              ptsy.push_back(next_wp1[1]);
              ptsy.push_back(next_wp2[1]);  // the vector includes 5 y points: the two previous point + 3 locations in 30, 60 and 90m

              // transformation to local car's coordinates
              for (int i = 0; i < ptsx.size(); i++ )
              {
          	//shift car reference angle to 0 degrees 
          	double shift_x = ptsx[i]-ref_x;
          	double shift_y = ptsy[i]-ref_y;

		ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
		ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
              }
              // create a spline
              tk::spline s;

              // set (x,y) points (anchor points) to the spline
              s.set_points(ptsx,ptsy);
		
	      // Defining the acutal (x,y) points that the path planner is going to be using 
              vector<double> next_x_vals;
              vector<double> next_y_vals;

	      // Start with all of the previous path points from last time
              for(int i = 0; i < previous_path_x.size(); i++)
              {
          	next_x_vals.push_back(previous_path_x[i]);
          	next_y_vals.push_back(previous_path_y[i]);
              }
	      // Calculate how to break up spline points so that we travel at our desired reference velocity
              double target_x = 30.0;
              double target_y = s(target_x);
              double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
          	
              double x_add_on = 0;
		
	      // Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
	      for (int i = 1; i <= 50-previous_path_x.size(); i++)
	      {
		if(ref_vel > car_speed)
		{
		  car_speed+=.224;  //acceleration of 5 m/s²
		}
		      
		else if(ref_vel < car_speed)
		{
		  car_speed-=.224;  // deceleration of 5 m/s²
		}
		
		double N = (target_dist/(.02*car_speed/2.24)); // ref_vel[mph] dividing by 2.24 --> m/s
		double x_point = x_add_on+(target_x)/N;
		double y_point = s(x_point);

		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;

		// rotate back to normal after rotating it earlier  **local coordinates back to global coordinates**
		x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
		y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));

		x_point += ref_x;
		y_point += ref_y;


		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);
	      }
          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          	
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        } // end "telemetriy" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
