#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

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
  
  int lane = 1;
  double ref_velocity = 0.0;
  int state = 0;
  int prep_count = 0;
  int prep_target_lane = 1;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,
               &lane, &ref_velocity, &state, &prep_count, &prep_target_lane]
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
          double original_car_s = car_s;
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

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          int previous_path_size = previous_path_x.size();

          // ************************ //
          // ** DECIDE ON BEHAVIOR ** //
          // ************************ //
          if (previous_path_size > 0) {
            car_s = end_path_s;
          }

          bool consider_switch = false;
          bool slow_down = false;
          int switch_lane = lane;
          int adj_car_idx;
          double max_velocity = 49.5;

          // lane data 
          vector<double> lane_speeds = {0, 0, 0};
          vector<vector<int>> lane_cars = {{}, {}, {}};
          vector<int> lane_nearby_cars = {0, 0, 0}; // store cars that are pretty near us behind, or all cars ahead
          
          // blocking car
          int block_car_idx;
          double block_car_speed = 49.5;
          double min_block_car_dist = 10000.0;

          for (int i=0; i < sensor_fusion.size(); ++i) {

            double obj_speed = getCarSpeed(sensor_fusion[i]);
            double obj_s = sensor_fusion[i][5];
            double obj_d = sensor_fusion[i][6];
            int obj_lane = getCarLane(obj_d); // returns 99 if not in the 3 lanes

            // add car data to appropriate lane
            if (obj_lane < 3) {
              lane_speeds[obj_lane] += obj_speed;
              lane_cars[obj_lane].push_back(i);
              if ((obj_s - car_s > - 20) && (obj_s - car_s < 80)) {
                lane_nearby_cars[obj_lane] += 1;
              }
            }

            // check if car is in your lane and too close
            if (obj_lane == lane) {
              double obj_end_path_s = obj_s + (double)previous_path_size * 0.02 * obj_speed;
              double dist_to_obj = car_s - obj_end_path_s;

              if (obj_end_path_s >= car_s) {
                // record that this is the closest car in front of us
                if (dist_to_obj < min_block_car_dist) {
                  min_block_car_dist = dist_to_obj;
                  block_car_speed = obj_speed;
                  block_car_idx = i;
                }
                
                // if we are too close, just slow down before deciding anything
                if (obj_end_path_s - car_s < 25) {
                  slow_down = true;
                // otherwise if b/w 25-35, then consider switching lanes
                } else if (obj_end_path_s - car_s < 35) {
                  consider_switch = true;
                }
              }
            }
          }

          // get speeds and value (inverse of cost)
          vector<double> lane_value = {0,0,0};
          for (int i=0; i < 3; i++) {
            if (lane_speeds[i] > 0) {
              lane_speeds[i] = lane_speeds[i] / (double)lane_cars[i].size();
            } else {
              lane_speeds[i] = 49.5;
            }
            // dock some points depending on the crowd of the lane
            lane_value[i] = lane_speeds[i] - 5 * (double)lane_nearby_cars[i];
          }
          
          // only consider switch if you're cleanly in a lane (not in the middle)
          // this is to help for cases where we are mid-transition but the path isn't complete yet
          bool in_lane_center = false;
          if ((car_d > lane * 4 + 1) && (car_d < lane * 4 + 3)) {
            in_lane_center = true;
          }
          
          // check if you're prepping for lane change by slowing down
          // if prepping, don't allow us to consider which lane to change
          bool not_prepping = true;
          if ((state == 3) && (prep_count < 20)) {
            not_prepping = false;
          }
          
          // ********************* //
          // ** DECIDE ON STATE ** //
          // ********************* //
          // -1 = slow down and stay in this lane
          // 0 = forward tracking
          // 1 = immediate switch
          // 2 = car is 10m vicinity, car has space in front, car vs guy in front of u > 20
          // 3 = plan a switch car is pretty ahead of u, slown down to lane change
          // 99 = just stay in this lane and track the car in front of you

          if (consider_switch && in_lane_center && not_prepping && (ref_velocity > 30)) {
            
            // STEP 1: Which lanes to consider, based on speed
            switch_lane = 1;
            if (lane == 1) {
              if (lane_nearby_cars[0]== 0) {
                switch_lane = 0;
              } else if (lane_nearby_cars[2] == 0) {
                switch_lane = 2;
              } else if (lane_value[0] > lane_value[2]) {
                switch_lane = 0;
              } else {
                switch_lane = 2;
              }
            } 

            // STEP 2: Check cars in that lane to decide what we can do
            bool immediate_switch = true;
            double adj_car_speed = 49.5;
            double adj_car_dist = 10000.0;
            double adj_car_front_space = 0.0;
            double adj_car_back_space = 0.0;

            for (int j=0; j < lane_cars[switch_lane].size(); ++j) {
              int obj_idx = lane_cars[switch_lane][j];
              double obj_speed = getCarSpeed(sensor_fusion[obj_idx]);
              double obj_s = sensor_fusion[obj_idx][5];
              double obj_end_path_s = obj_s + (double)previous_path_size * 0.02 * obj_speed;

              // if obj between -30 behind us and +30 in front of us, hard to switch
              if ((car_s - 30 < obj_end_path_s) && (obj_end_path_s < car_s + 30)) {
                immediate_switch = false;
              } 

              // determine if this is the nearest adj car
              double distance_to_us = obj_end_path_s - car_s;
              if ((distance_to_us > -10) && (fabs(distance_to_us) < adj_car_dist)) {
                adj_car_idx = obj_idx;
                adj_car_dist = fabs(distance_to_us);
                adj_car_speed = obj_speed;
                adj_car_front_space = spaceAroundCar(sensor_fusion, lane_cars[switch_lane], adj_car_idx, 'f');
                adj_car_back_space = spaceAroundCar(sensor_fusion, lane_cars[switch_lane], adj_car_idx, 'b');
              }
            }
            
            double current_dist_adj_car = fabs(original_car_s - (double)sensor_fusion[adj_car_idx][5]);
            if (immediate_switch && (ref_velocity > 35)) {
              if (state != 2) {
                std::cout<<"Make immediate switch to lane "<<switch_lane<<std::endl;
              }
              state = 2;
              prep_count = 0;
              // technically you can set state = 1 here and bypass the checking
              // but if we set state 2 we can make a couple of trajectories & check if they're viable
              // this is safer
            } else if ((lane_nearby_cars[switch_lane] < 2) && (current_dist_adj_car <= 10) && (ref_velocity > adj_car_speed + 5)) {
              // test some trajectories
              state = 2;
              prep_count = 0;
            } else if ((current_dist_adj_car <= 40) && (adj_car_back_space >= 50) && (adj_car_speed > block_car_speed + 5)) {
              if (state != 3) {
                std::cout<<"Slow down to try to go behind dude in lane "<<switch_lane<<std::endl;
              }
              state = 3;
              prep_target_lane = switch_lane; // store the prep target lane variable
              prep_count = 1;
            } else {
              if (state != 99) {
                std::cout<<"Trail the car in front of us"<<std::endl;
              }
              state = 99;
              prep_count = 0;
            }
          } else if (slow_down) {
            if (state != 99) {
              std::cout<<"Too close to car ahead... slowing down..."<<std::endl;
            }
            state = 99;
            prep_count = 0;
          } else if (not_prepping == false) {
            // if you've been prepping, try to switch to the lane you were prepping for
            state = 2; 
            switch_lane = prep_target_lane; // use the stored value of this
            prep_count += 1;
          } else {
            //std::cout<<"I am happy in my lane!"<<std::endl;
            state = 0; // move forward
            prep_count = 0;
          }
         

          // ***************** //
          // ** SET UP PATH ** //
          // ***************** //
          double pos_x;
          double pos_y;
          double angle;
          vector<double> spline_ref_x;
          vector<double> spline_ref_y;
          
          // set the most recent point
          double prior_car_x;
          double prior_car_y;

          if (previous_path_size < 2) {
            pos_x = car_x;
            pos_y = car_y;
            angle = deg2rad(car_yaw);
            prior_car_x = car_x - cos(car_yaw);
            prior_car_y = car_y - sin(car_yaw);   
          } else {
            // since we have enough data, lets use that instead
            pos_x = previous_path_x[previous_path_size-1];
            pos_y = previous_path_y[previous_path_size-1];
            prior_car_x = previous_path_x[previous_path_size-2];
            prior_car_y = previous_path_y[previous_path_size-2];
            angle = atan2(pos_y - prior_car_y, pos_x - prior_car_x);
          }
          // add some reference points to begin with
          spline_ref_x.push_back(prior_car_x);
          spline_ref_x.push_back(pos_x);
          spline_ref_y.push_back(prior_car_y);
          spline_ref_y.push_back(pos_y);

          vector<vector<double>> trajectory_vals;
          vector<double> target_s_points = {30, 60, 90};


          // **************************************** //
          // ** GENERATE TRAJECTORY BASED ON STATE ** //
          // **************************************** //
          
          if (state != 2) {
            trajectory_vals = generateNextVals(car_s, pos_x, pos_y, angle, state,
                                               ref_velocity, block_car_speed, // only used if state =99
                                               target_s_points, {lane, lane, lane},
                                               previous_path_x, previous_path_y,
                                               spline_ref_x, spline_ref_y,
                                               map_waypoints_s, map_waypoints_x, map_waypoints_y);
          } else {
            // try a couple of trajectories
            bool is_viable;
            vector<vector<double>> option_vals;
            vector<vector<double>> target_s_options = {
              {30, 60, 90},
              {5, 35, 65, 95},
              {10, 40, 70, 100},
              {15, 45, 75, 105},
            };
            vector<vector<int>> target_lane_options = {
              {switch_lane, switch_lane, switch_lane},
              {lane, switch_lane, switch_lane, switch_lane},
              {lane, switch_lane, switch_lane, switch_lane},
              {lane, switch_lane, switch_lane, switch_lane},
            };

            for (int i=0; i<target_s_options.size(); ++i) {
              option_vals = generateNextVals(car_s, pos_x, pos_y, angle, state,
                                             ref_velocity, block_car_speed,
                                             target_s_options[i], target_lane_options[i], // use ith option
                                             previous_path_x, previous_path_y,
                                             spline_ref_x, spline_ref_y,
                                             map_waypoints_s, map_waypoints_x, map_waypoints_y);
              is_viable = viablePath(option_vals, block_car_idx, lane_cars[switch_lane],
                                     sensor_fusion, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              if (is_viable) {
                std::cout<<"Switching to lane "<<switch_lane<<" with option "<<i<<std::endl;
                trajectory_vals = option_vals;
                lane = switch_lane; // only update lane if this works
                break;
              }
            }

            if (lane != switch_lane) {
              std::cout<<"None of the options worked... just track car"<<std::endl;
              trajectory_vals = generateNextVals(car_s, pos_x, pos_y, angle, 99,
                                               ref_velocity, block_car_speed, // only used if state =99
                                               target_s_points, {lane, lane, lane},
                                               previous_path_x, previous_path_y,
                                               spline_ref_x, spline_ref_y,
                                               map_waypoints_s, map_waypoints_x, map_waypoints_y);
              if (not_prepping == false) {
                // if we we routed here b/c we were prepping & wanted to check if we can switch now,
                // reset the state back to prepping i.e. 3
                state = 3;
              }
            }
          }

          // ************ //
          // * END HERE * //
          // ************ //

          msgJson["next_x"] = trajectory_vals[0];
          msgJson["next_y"] = trajectory_vals[1];

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