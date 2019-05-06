#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  int lane = 1; // 0, 1, 2
  float ref_vel = 0; //initial mph

  h.onMessage([&lane, &ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

	  // START
	  int prev_size = previous_path_x.size();

	  if(prev_size > 0) {
	    car_s = end_path_s;
	  }

	  // Use sensor fusion to find if there are any cars close by
	  bool tooCloseFront = false;
	  bool tooCloseLeft  = false;
	  bool tooCloseRight = false;
	  for(int idx=0; idx < sensor_fusion.size(); idx++) {

	    // check the lane of the car
	    float d = sensor_fusion[idx][6];
	    int   check_lane = -1;
	    if (d > 0 && d <4) {
	      check_lane = 0;
	    } else if (d > 4 && d < 8) {
	      check_lane = 1;
	    } else if (d < 12) {
	      check_lane = 2;
	    } else {
	      continue;
	    }

	    // Get car position from speed
	    double vx = sensor_fusion[idx][3];
	    double vy = sensor_fusion[idx][4];
	    double check_speed = sqrt(vx*vx + vy*vy);
	    double check_car_s = sensor_fusion[idx][5];
	    check_car_s += (double)prev_size * 0.02*check_speed;

	    
	    if(lane == check_lane) {
	      //check if there is a car which is close from front
	      tooCloseFront |= (check_car_s > car_s) && (check_car_s-car_s < 30);
	    } else if ((lane - check_lane) == 1) {
	      // check if there is a car which is close in left lane
	      tooCloseLeft |= ((car_s - 30) < check_car_s) && ((car_s + 30) > check_car_s);
	    } else {
	      // check if there is a car which is close in the right lane
	      tooCloseRight |= ((car_s - 30) < check_car_s) && ((car_s + 30) > check_car_s);
	    }
	  }


	  // Finite State machine
	  double del_vel = 0;
	  if(tooCloseFront) {
	    if ( !tooCloseLeft && (lane > 0) ) {
	      lane--; // change to left lane
 	    } else if (!tooCloseRight && (lane < 2) ) {
	      lane++; // change to right lane
	    } else {
	      del_vel -= 0.224; // slow down
	    }
	  } else {
	    if (lane != 1) {
	      if (!tooCloseRight && (lane==0) ) {
		lane++; // come to center
	      } else if (!tooCloseLeft && (lane==2)) {
		lane--; // come to center
	      } else {
		// do nothing
	      }
	    }
	    if (ref_vel < 49.5) {
	      del_vel += 0.224; // increase speed
	    }
	  }

          /**
           * Define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
	  
	  
	  // List of widely spaced waypoints, evenly spaced at 30m
	  vector<double> ptsx;
	  vector<double> ptsy;

	  // reference x,y, yaw states
	  double ref_x = car_x;
	  double ref_y = car_y;
	  double ref_yaw = deg2rad(car_yaw);

	  if (prev_size < 2) {
	    // If no previous points - use the car
	    ptsx.push_back(car_x - cos(car_yaw));
	    ptsx.push_back(car_x);
	    ptsy.push_back(car_y - sin(car_yaw));
	    ptsy.push_back(car_y);
	  } else {
	    // Use previos points
	    ref_x = previous_path_x[prev_size-1];
	    ref_y = previous_path_y[prev_size-1];	    
	    
	    double ref_x_prev = previous_path_x[prev_size-2];
	    double ref_y_prev = previous_path_y[prev_size-2];

	    // redefine the reference x,y,yaw
	    ref_yaw = atan2(ref_y - ref_y_prev, ref_x-ref_x_prev);

	    // use the two points
	    ptsx.push_back(ref_x_prev);
	    ptsx.push_back(ref_x);
	    ptsy.push_back(ref_y_prev);
	    ptsy.push_back(ref_y);
	  }

	  vector<double> next_wp0 = getXY(car_s+30, 2+(4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
	  vector<double> next_wp1 = getXY(car_s+60, 2+(4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
	  vector<double> next_wp2 = getXY(car_s+90, 2+(4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

	  ptsx.push_back(next_wp0[0]);
	  ptsx.push_back(next_wp1[0]);
	  ptsx.push_back(next_wp2[0]);

	  ptsy.push_back(next_wp0[1]);
	  ptsy.push_back(next_wp1[1]);
	  ptsy.push_back(next_wp2[1]);

	  // shift to local vehicle co-ordinates
	  for (int idx=0; idx < ptsx.size(); idx++) {
	    double shift_x = ptsx[idx]-ref_x;
	    double shift_y = ptsy[idx]-ref_y;
	    ptsx[idx] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
	    ptsy[idx] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
	  }

	  // create a spline
	  tk::spline spl;
	  spl.set_points(ptsx, ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;
	  // copy previous values to next values
	  for(int idx=0; idx < previous_path_x.size(); idx++) {
	    next_x_vals.push_back(previous_path_x[idx]);
	    next_y_vals.push_back(previous_path_y[idx]);
	  }

	  // Calculae spline points
	  double target_x = 30.0;
	  double target_y = spl(target_x);
	  double target_dist = sqrt( (target_x*target_x) + (target_y*target_y));
	  double x_add_on = 0;

          
	  // fill up the path planner
	  for(int idx =1; idx <= 50-previous_path_x.size(); idx++) {
	    ref_vel += del_vel;
	    if (ref_vel > 49.5) {
	      ref_vel = 49.5; // maximum speed
	    } else if (ref_vel < 0.224) {
	      ref_vel = 0.224;
	    } else {
	      // do nothing
	    }
	    double N = target_dist/(0.02*ref_vel/2.24);
	    double x_point = x_add_on + (target_x)/N;
	    double y_point = spl(x_point);
	    x_add_on = x_point;

	    double x_ref = x_point;
	    double y_ref = y_point;

	    x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw) + ref_x;
	    y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw) + ref_y;

	    next_x_vals.push_back(x_point);
	    next_y_vals.push_back(y_point);
	  }
	  
	  // END
	  /*double dist_inc = 0.5;
	  for(int idx =0; idx <50; idx++) {
	    double next_s = car_s + (idx+1)*dist_inc;
	    double next_d = 6;
	    auto xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	    next_x_vals.push_back(xy[0]);
	    next_y_vals.push_back(xy[1]);
	    }*/
	  json msgJson;	  
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
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
