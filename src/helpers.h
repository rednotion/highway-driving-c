#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include "spline.h"

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
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

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
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

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

// Return car lane
int getCarLane(double d) {
  if (d <= 4) {
    return 0;
  } else if (d <= 8) {
    return 1;
  } else if (d <= 12) {
    return 2;
  } else {
    return 99;
  }
}

// get speed
double getCarSpeed(vector<double> sensor_fusion) {
  double obj_vx = sensor_fusion[3];
  double obj_vy = sensor_fusion[4];
  double obj_speed = sqrt(obj_vx*obj_vx + obj_vy*obj_vy);
  return obj_speed;
}

// get space around car by comparing all the objects in its lane
// direction = 'f' means forward space (compare to cars in front)
// direction = 'b' means backward space
// track_car_idx is the car you want as the reference point
double spaceAroundCar(vector<vector<double>> sensor_fusion, vector<int> lane_cars, 
                      int track_car_idx, char direction) {
  double space = 10000.0;
  double track_car_s = sensor_fusion[track_car_idx][5];
  if (direction == 'f') {
    // space in front of this car
    for (int i=0; i < lane_cars.size(); ++i) {
      double s = sensor_fusion[lane_cars[i]][5];
      double dist = s - track_car_s;
      if ((s > track_car_s) && (dist < space)) {
        space = dist;
      }
    }
  } else {
    // space behind this car
    for (int i=0; i < lane_cars.size(); ++i) {
      double s = sensor_fusion[lane_cars[i]][5];
      double dist = track_car_s - s;
      if ((track_car_s > s) && (dist < space)) {
        space = dist;
      }
    }
  }
  return space;
}

// check viable path by estimating the path of every other object and checking for distance
bool viablePath(vector<vector<double>> trajectory_vals,
                int block_car_idx, vector<int> lane_obj_idxs,
                vector<vector<double>> sensor_fusion,
                const vector<double> &maps_s, const vector<double> &maps_x,
                const vector<double> &maps_y) {

  vector<double> next_x = trajectory_vals[0];
  vector<double> next_y = trajectory_vals[1];
  
  // proxy angle by using last minus first value
  int end_int = next_y.size() - 1;
  double angle = atan2(next_y[end_int] - next_y[0], next_x[end_int] - next_x[0]);
  
  vector<int> objs_to_test = lane_obj_idxs;
  objs_to_test.push_back(block_car_idx);
  
  for (int j=0; j < objs_to_test.size(); ++j) {
    double obj_s = sensor_fusion[objs_to_test[j]][5];
    double obj_d = sensor_fusion[objs_to_test[j]][6];
    double obj_speed = getCarSpeed(sensor_fusion[objs_to_test[j]]);
    
    for (int i=0; i < next_x.size(); ++i) {
      obj_s += 0.02 * obj_speed;
      vector<double> obj_xy = getXY(obj_s, obj_d, maps_s, maps_x, maps_y);
      double vertical_dist = fabs(obj_xy[0] - next_x[i]);
      double horizontal_dist = fabs(obj_xy[1] - next_y[i]);
      double pythagoras = pow(vertical_dist*vertical_dist + horizontal_dist*horizontal_dist, 0.5);
      // check if diagonal distance b/w our path & other car is less than 10m
      if (pythagoras < 6) {
        return false;
      }
      // if the other lane obj ends up being just 20m ahead of us, don't bother man
      // j != lane_obj_idxs.size() --> not the blocking car
      vector<double> next_sd  = getFrenet(next_x[i], next_y[i], angle, maps_x, maps_y);
      double s_dist = obj_s - next_sd[0];
      if ((j != lane_obj_idxs.size()) && (s_dist > 0) && (s_dist < 20)) {
        std::cout<<"......car too close to switch"<<std::endl;
        return false;
      }
    }
  }
  return true;
}

// generate path
vector<vector<double>> generateNextVals(double car_s, double pos_x, double pos_y, double angle,
                                        int state, 
                                        double &ref_velocity, double target_velocity,
                                        vector<double> target_s_points, 
                                        vector<int> target_lanes,
                                        //int target_lane,
                                        vector<double> previous_path_x, vector<double> previous_path_y,
                                        vector<double> spline_ref_x, vector<double> spline_ref_y,
                                        const vector<double> &maps_s, const vector<double> &maps_x, 
                                        const vector<double> &maps_y) {

  // what we want to collect
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // now add some extra points based on input target_s_points (how fast u wanna change lane)
  for (int i=0; i < target_s_points.size(); i++) {
    vector<double> next_ref_point = getXY(car_s+target_s_points[i], (2+4*target_lanes[i]), maps_s, maps_x, maps_y);
    spline_ref_x.push_back(next_ref_point[0]);
    spline_ref_y.push_back(next_ref_point[1]);
  }

  // shift the point relative to current car pos
  for (int i=0; i < spline_ref_x.size(); i++) {
    double shift_x = spline_ref_x[i] - pos_x;
    double shift_y = spline_ref_y[i] - pos_y;
    spline_ref_x[i] = (shift_x * cos(0 - angle) - shift_y * sin(0 - angle));
    spline_ref_y[i] = (shift_x * sin(0 - angle) + shift_y * cos(0 - angle));
  }

  // spline this mf
  tk::spline spline_machine;
  spline_machine.set_points(spline_ref_x, spline_ref_y);

  // to smoothen out the transition, add points from the previous path first
  for (int i=0; i < previous_path_x.size(); i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // calculate how to break up spline points to travel at our desired ref velocity
  double target_x = target_s_points[0];
  double target_y = spline_machine(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);
  double current_x = 0;  // imagine travelling from origin of the triangle
  int n_added = 0;
  
  //while (current_x < target_x) {
  while (n_added < 50 - previous_path_x.size()) {
    // fix velocity
    double delta = (ref_velocity < 35) ? 0.16 : 0.224;

    // for keep in lane, immediate switch, test switches, speed up if possible
    if ((state == 0) | (state == 1) | (state == 2)) {
      if (ref_velocity < 49.3) {
        ref_velocity += delta;
      }
    // for prepare to switch
    } else if (state == 3) {
      if (ref_velocity > 35) {
        ref_velocity -= delta;
      }
    // for follow
    } else if (state == 99) {
      // only if we're more than 5kmph different then we try to adjust
      if (fabs(ref_velocity - target_velocity) > 5) {
        if (ref_velocity < target_velocity) {
          ref_velocity += delta;
        } else {
          ref_velocity -= delta;
        }
      }
    } else if (state == -1) {
      // just slow down, no limit
      if (ref_velocity > 0.5) {
        ref_velocity -= delta;
      }
    }

    // curving if we do lane change, so dampen the speed a little for more pieces
    if ((state == 1) | (state == 2)) {
      ref_velocity -= 0.05;
    }

    // then decide on how far to push x_local
    double n_pieces = target_dist / (0.02 * ref_velocity / 2.24);
    double x_local = current_x + (target_x / n_pieces);
    double y_local = spline_machine(x_local);

    // reset our index
    current_x = x_local;

    // rotate back to global coordinates to push to the next_vals
    // step 1: rotate
    double x_point = (x_local * cos(angle) - y_local * sin(angle));
    double y_point = (x_local * sin(angle) + y_local * cos(angle));
    // step 2: shift
    x_point += pos_x;
    y_point += pos_y; 

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
    
    n_added += 1;
  }
  
  return {next_x_vals, next_y_vals};
}

#endif  // HELPERS_H