# Model Documentation

## Overview

## 1. Behavior Choice
### Overview of states
- 0: Keep in lane and try to **maintain** target max_velocity of 49.5
- 1: Make an immediate switch in 30m
- 2: Try a few trajectories to make a lane switch
- 3: Prepare for lane switch by slowing down
- 99: Track the car in front of you but **slow down**

### High-level decisions
First, we need to check for some **data points** before deciding on our behavior/state:
1. If there is a car within **20m** ahead of us, set `slow_down = true`. Else, if we are between **20-35m**, set `consider_switch = True`.
2. If our car's current position/`end_path_s` is **not within** 2m of the target `lane` center, we set `in_lane_center = False`.
3. If the global variable `state==3` from the previous loop (i.e. we were slowing down to try and change lane), we set `not_prepping = False`. We also implement a 'limit' on this, by tracking the number of times we are continuously deciding on `state==3` with the global variable `prep_count`

Then, we can decide on the first series of states using the series of if-else combinations:
```c++
if (slow_down) {
  state = 99;
} else if ((not_prepping == false) && (prep_count < 20)) { 
  state = 2;
  switch_lane = prep_target_lane;
  prep_count += 1
} else if (consider_switch && in_lane_center && not_prepping && (ref_velocity > 30)) {
  // CONSIDER SWITCH MODULE (see below)
} else {
  state = 0
}
```

### Consider Switch Module
The consider switch module requires 4 booleans to be set to true:
- `consider_switch`: We are within 35m of the car in front of us, necessitating a lane change
- `in_lane_center`: We are not mid-switch, which would prevent random swerving when we are in the midst of changing lanes, as the path length is usually shorter than what is needed for a complete lane change
- `not_prepping`: If we are "prepping" for lane change, we can immediately switch to `state = 2` and set the `switch_lane` to the previous decision
- `ref_velocity > 30`: This is to prevent cases of a lane change when the car is too slow, which might be dangerous and lead to collisions.

Then, we go through a few steps:
1. Decide on a switch-lane: If we are in lane 0 or 2, then `switch_lane = 1`. If we are in the middle aisle, pick the lane with (a) no cars; (b) no cars in the near vicinity; or (c) whose `lane_value` is greater. `lane_value` is the average speed of cars in the lane, minus the number of cars in the near vicinity. 
2. Extrapolate the path of all the cars in the `switch_lane` to check:
  - If any of them will be within (-30, +30)m of our end path. If so, `immediate_switch = False` as we are likely to collide.
  - Track the nearest car to us that is within (-10, inf) and record its index, and the space available in front and behind that car. 

Then, we can use a series of if-else statements to refine the state
```c++
if (immediate_switch && (ref_velocity > 35)) {
  state = 1 // or state = 2 to generate trajectories anyway
} else if ((lane_nearby_cars[switch_lane] < 2) && (current_dist_adj_car <= 10) && (ref_velocity > adj_car_speed + 5)) {
  // 1. <2 cars nearby in the switch lane
  // 2. current distance (not end_path distance) is around ~10m, so we could potentially overtake the car
  // 3. we are travelling faster than the car already, otherwise will be hard to switch
  state = 2
} else if ((current_dist_adj_car <= 40) && (adj_car_back_space >= 50) && (adj_car_speed > block_car_speed + 5)) {
  // 1. current distance is around ~40m so it's not too far to slow down to
  // 2. there's sufficient space behind the car to go behind it
  // 3. its worth switching into that lane if that car is going faster than the car blocking us
  state = 3;
  not_prepping = false;
  prep_count += 1;
  prep_target_lane = switch_lane; // store our goal lane in the global variable for the next iteration
} else {
  state = 99
}
```

## 3. Trajectory Generation
There is a general trajectory generation function called `generateNextVals` which will generate the next x and y points for the program. 
```c++
generateNextVals(car_s, pos_x, pos_y, angle, state,
                 ref_velocity, block_car_speed, // only used if state =99
                 target_s_points, target_lane_points,
                 previous_path_x, previous_path_y,
                 spline_ref_x, spline_ref_y,
                 map_waypoints_s, map_waypoints_x, map_waypoints_y);
```
The more important variables are:
- `state`: This will be used within the function to determine how the velocity changes when generating the path (e.g. slow down, speed up, aim for some target velocity)
- `block_car_speed`: This will be used as the reference velocity to slow down/target to, if the `state = 99`
- `target_s_points`: This is a vector<double> of the s points we want to generate our spline on e.g. *{30, 60, 90}*
- `target_lane_points`: This is a vector<int> of the lanes we want to generate our spline on. It will be converted to `d` values e.g. *{1,1,1}*

### Generating options for lane switching
For `state == 2`, the program will try a couple of combinations of `target_s_points` and `target_lane_points` to try and find a viable one. If it fails, it'll try the next option. If there is no viable paths after trying all options, it will move to `state == 99`.

The options tried (in order) are:
| Type | target_s_points | target_lane_points |
|------|-----------------|--------------------|
| Immediate switch | {30, 60, 90} | {switch_lane, switch_lane, switch_lane} |
| Switch in 5m | {5, 35, 65, 95 } | {lane, switch_lane, switch_lane, switch_lane} |
| Switch in 5m | {10, 40, 70, 100 } | {lane, switch_lane, switch_lane, switch_lane} |
| Switch in 5m | {15, 45, 75, 105 } | {lane, switch_lane, switch_lane, switch_lane} |

The test for the "viability" of the path is located in `isViablePath`, which checks for:
1. Collisions with car ahead of us (in current lane) & cars in switch lane: Where difference is `s` value is < 10
2. If upon switching into the target lane, there is a car ahead of us within 20m, as it might (a) cause possible collision if we don't slow down fast enough; (b) not be worth it to switch to just get stuck behind another car.

## Further improvements
- When we are in the middle lane, we pre-select a lane choice and only try one option. But we could also try the second option, if the first one fails.
- Consider switching 2 lanes (e.g. switching to the middle lane might not be worth it, but it gives us access to the further lane which might be relatively empty)
