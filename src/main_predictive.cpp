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


/* 
 * 
 */
int d_to_lane(double d, double lane_width = 4) {
  int lane = 0;
  
  if (d < 0) {
    return lane;
  }
  
  while (d >= lane_width) {
    d -= lane_width;
    lane++;
  }
  return lane;
}



int lane = 1;
double ref_vel = 0.0;
double max_acceleration = .242;
double max_decceleration = .242;
double max_vel = 49.85 - max_acceleration; // Avoid the velocity to go above 50.0. Therefor the maximum + acceleration has to be less then 50.
double planning_distance = 50.0;
double decision_breakpoint = 40.0;
int best_lane = 1;
double lane_change_safty_range = 20;


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
  //std::cout << "2" << std::endl;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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
          double car_curr_s = car_s; // car_s will be overwritten to the latest planned point, so i store it here.

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;


          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
           int prev_size = previous_path_x.size();
          
          
          // Check for vehicle in front of us for the position were we will be in the end of the current planning 
          // current planning are <=50 waypoints * 20ms so <1 sec in future
          if (prev_size > 0) {
          	car_s = end_path_s;
          }
          
          
          // Variable to decide if we have to think about a lane-change or not
          // To close means a lane-change is required or deceleration
          // should_change_lane is for the predictive part and indicates it could be good/not to change lane
          bool too_close = false;
          bool should_change_lane = false;
          
            
          // Cost compare to get cheapest lane
          double left_lane_cost = 0.0;
          double middle_lane_cost = 0.0;
          double right_lane_cost = 0.0;
          
          // Safty first. Check if the lane is safe to go
          bool left_lane_save = true;
          bool middle_lane_save = true;
          bool right_lane_save = true;
          
          // Get the velocity and distance to the closest/slowest car in our lane in front of us
          double car_ahead_velocity = std::numeric_limits<double>::max();
          double car_ahead_s_delta = std::numeric_limits<double>::max();
          
          // Check all sensor-fusion data and look for cars near to us
          for (int i = 0; i < sensor_fusion.size(); i++) {
          	
            float d = sensor_fusion[i][6]; // Get d value which indicates on which lane the object is
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt((vx * vx) + (vy * vy));
            double check_car_s = sensor_fusion[i][5];
            
            // If car is on same lane as my car
            if (d_to_lane(car_d) == d_to_lane(d)) {
              
              // Predicitve part of car is far enough aways from us
              // If it is farer away than 30-s we can use the predication to check for the best-lane option
              // If it is closer than 30-s don't check for optimal paths
              if ((check_car_s  > car_curr_s) && (check_car_s - car_curr_s < decision_breakpoint) && (check_car_s - car_curr_s > 30)) {
              	should_change_lane = true;
              }

              // If it is closer than 30-s don't check for optimal paths instead check for any possible path
              // If a path has not be found before and we get to close -> safty first
              // Critical distance were the predictive-part does not longer have priority -> safty first
              if ((check_car_s > car_curr_s) && (check_car_s - car_curr_s < 30)) { // TODO: WAS 30
                
                // We should try a lane-change
                too_close = true;
                // Save speed if the car is the slowest. This will be used for velocity adjustment
                if (check_speed < car_ahead_velocity) {
                  car_ahead_velocity = check_speed;
                }
                // Check if it is closer than a car scanned before
                if (check_car_s - car_curr_s < car_ahead_s_delta) {
                	car_ahead_s_delta = check_car_s - car_curr_s;
                }
              }             
            }
            
            
            // Calculcate the cost for each lane, the costs bases on distance to car and amount of cars.
            if ((check_car_s > car_curr_s) && (check_car_s - car_curr_s) < planning_distance) {
              switch(d_to_lane(d)) {
                case 0: left_lane_cost   += pow(planning_distance - (check_car_s - car_s), 1.2); break;
                case 1: middle_lane_cost += pow(planning_distance - (check_car_s - car_s), 1.2); break;
                case 2: right_lane_cost  += pow(planning_distance - (check_car_s - car_s), 1.2); break;
              }
            }
            
            // Dynamic safty distance calculates the required gap on a other lane for a safe lane-change. For this 
            // solution the delta of current-velocity to maximum velocity is considered because we need a bigger gap on the lane 
            // to change because a driving car there could have a much higher speed when we drive much slower than allowed.
            // More over a constant (in this case 15) is added which describes the velocity independend gap for a safe lane change.
            double dynamic_safty_distance = ((fabs(ref_vel - max_vel) / max_vel) * lane_change_safty_range) + 15;
            // Avoid collisions
            // Get cars which are close to me. Car is not allowed to drive <dynamic_safty_distance> behind/before me on the different lane
            if ((check_car_s > car_curr_s - dynamic_safty_distance) && (check_car_s < car_s + dynamic_safty_distance)) {
            	// If a car is found in the range -/+ dynamic-distance-s of me, mark that lane as unsafe.
              	switch(d_to_lane(d)) {
                  case 0: left_lane_save = false; break;
                  case 1: middle_lane_save = false; break;
                  case 2: right_lane_save = false; break;
                }
            }
          }
          
          
          bool switch_lane_possible = false;
          int plannedLane = -1;

          // Only evaluate a new best-lane if the is any costs on the current lane.
          if ((lane == 0 && left_lane_cost   != 0 || 
               lane == 1 && middle_lane_cost != 0 || 
               lane == 2 && right_lane_cost  != 0) && should_change_lane) { 
            best_lane = left_lane_cost < middle_lane_cost // Is
              ? left_lane_cost <= right_lane_cost 
                ? 0  // Left lane
                : 2  // Right lane
              : middle_lane_cost <= right_lane_cost // Fix bug which cause the car to go from left-lane to right always if the left-lane was too ful.
                ? 1  // Middle lane
                : 2; // Right lane lane
                  
          // Check lane change if current lane is not the best option
          if (best_lane != d_to_lane(car_d)) {
            // Check if the better lane is on the right/left side based on the current lane       
            if (best_lane < d_to_lane(car_d)) {
              // Better lane is on the left
              if (d_to_lane(car_d) == 1) {
                // middle-lane -> left-lane
                if (left_lane_save) {
                  // Left-lane has enough space
                  plannedLane = 0;
                  switch_lane_possible = true;
                }
              } else {
                // right-lane -> middle-lane
                if (middle_lane_save) {
                  // middle-lane has enough space
                  plannedLane = 1;
                  switch_lane_possible = true;
                }
              }
            } else {
              // Better lane is on the right
              if (d_to_lane(car_d) == 1) {
                // middle-lane -> right-lane
                if (right_lane_save) {
                  // right-lane has enough space
                  plannedLane = 2;
                  switch_lane_possible = true;
                }
              } else {
                // left-lane -> middle-lane
                if (middle_lane_save) {
                  // middle-lane has enough space
                  plannedLane = 1;
                  switch_lane_possible = true;
                }
              }
            }
          }
        }
  
        // Do not change lane if plannedLane is same as before or
        // the current velocity may risk a crash when doing a lane-change
        if (switch_lane_possible) {
          lane = plannedLane;
        } else if (too_close) {
          if (d_to_lane(car_d) == 1 && (left_lane_save || right_lane_save)) {
            if (left_lane_save) lane = 0;
            if (right_lane_save) lane = 2;
          } else if ((d_to_lane(car_d) == 0 || d_to_lane(car_d) == 2) && middle_lane_save) {
            lane = 1;
          } else {
            if (ref_vel > (car_ahead_velocity * 2.24)) {
              ref_vel -= max_decceleration;
            }
          }
        } else {
          if (ref_vel <= max_vel) {
            ref_vel += max_acceleration;
          }			
        }
      
          
          
        std::cout << left_lane_cost << " \t " << middle_lane_cost << " \t " << right_lane_cost << " | " << left_lane_save << " \t " << middle_lane_save << " \t " << right_lane_save << " - Best: " << best_lane << std::endl;
        
        // Variable which will be used for the spline-function (smooth curve)
        vector<double> ptsx;
        vector<double> ptsy;
        
        // Reference-values to calculate the cars position from global (map coordinates) to local-coordinates
        double ref_x = car_x;
        double ref_y = car_y;
        double ref_yaw = deg2rad(car_yaw);
        
        // Check if there are path-points left from the previous iterations
        if (prev_size < 2) {
          // If there are no planning from previous iterations available
          
          // calculate the position where the vehicle probably comes from
        	double prev_car_x = car_x - cos(car_yaw);
        	double prev_car_y = car_y - sin(car_yaw);
          
          // Add the pseudo-previous and current location to vectors for the spline-function         
          ptsx.push_back(prev_car_x);
          ptsx.push_back(car_x);
          ptsy.push_back(prev_car_y);
          ptsy.push_back(car_y);
        	
        } else {
          
          // Get the last position
        	ref_x = previous_path_x[prev_size - 1];
        	ref_y = previous_path_y[prev_size - 1];
          
          // Get the second last position
          double ref_x_prev = previous_path_x[prev_size -2];
          double ref_y_prev = previous_path_y[prev_size -2];

          // Caluclate the heading based on the second-last and last position
          ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
          
          // Save the last and second last position in vector which will be used for spline
          ptsx.push_back(ref_x_prev);
          ptsx.push_back(ref_x);
          ptsy.push_back(ref_y_prev);
          ptsy.push_back(ref_y);
        }
        
        
        // Get wide but even spaced way-points for spline generation (inspired by Udacity Q|A video)
        vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        vector<double> next_wp1 = getXY(car_s + 60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        vector<double> next_wp2 = getXY(car_s + 90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        
        // Add the x,y coordinates of the previously generated waypoints to the respective vectors for spline
        ptsx.push_back(next_wp0[0]);
        ptsx.push_back(next_wp1[0]);
        ptsx.push_back(next_wp2[0]);
        
        ptsy.push_back(next_wp0[1]);
        ptsy.push_back(next_wp1[1]);
        ptsy.push_back(next_wp2[1]);
        
        // Calculate the coordinates from world to car-positions. 
        // As described in the Udacity Q|A video "Makes the math much more easier"
        for(int i = 0; i < ptsx.size(); i++) {
          // Store the shift in coordinates for x, y
        	double shift_x = ptsx[i] - ref_x;
        	double shift_y = ptsy[i] - ref_y;
          
          // Take in account the current yaw-rate for position-transformation to local coordinates
          ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
          ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
        }
        
        // Create spline using "spline.h" provided by Tino Kluge: https://kluge.in-chemnitz.de/opensource/spline/
        tk::spline s;
        s.set_points(ptsx, ptsy);
        
        // Create position-vectors for x-,y-coordinates which will be passed to the simulator 
        vector<double> next_x_vals;
        vector<double> next_y_vals;
        
        // Add all previously planned coordinates which were not driven yet.
        for (int i = 0; i < prev_size; i++) {
        	next_x_vals.push_back(previous_path_x[i]);
        	next_y_vals.push_back(previous_path_y[i]);
        }
        
        // The following calculation is mostly inspired by the Q/A-video of udacity.
        // Get y coordinate based on x coordinate from previously created spline
        double target_x = 30.0;
        double target_y = s(target_x);
        // Measure the distance to drive, this is required to ensure we do not accelerate faster than allowed (code comes below)
        double target_dist = sqrt(target_x*target_x + target_y*target_y);
        
        // Store the last x-point to drive through for next calculation
        double x_add_on = 0;
        
        // I've decided to generate 30 instead of 50 next coordinates. (50 was in the Q|A video of udacity)
        // Within this i can faster react on short-term-changes in the traffic
        for (int i = 1; i < 30 - prev_size; i++) {

          // Ensure we do not accelerate more than allowed.
          // .02 because the simulator drives 1 coordinate every 20ms (0.020 seconds). /2.24 to convert mph to mps
        	double N = (target_dist / (.02 * ref_vel / 2.24));
          double x_point = x_add_on + (target_x) / N;
          double y_point = s(x_point);
          
          // Save last x_point
          x_add_on = x_point;
          
          double x_ref = x_point;
          double y_ref = y_point;
          
          // Calculate values to go back to map-coordinate
          x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
          y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
          
          // Add values required to go from local to map-coordinates
          x_point += ref_x;
          y_point += ref_y;

          
          // Store values in vectors which will be send to the simulator
          next_x_vals.push_back(x_point);
          next_y_vals.push_back(y_point);
        }
          
          

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
    //std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    //std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    //std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}