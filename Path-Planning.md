
---

** Path Planning Project **

The goal of this project is to build a path planner that creates smooth, safe trajectories for the car to follow. The highway track has other vehicles, all going different speeds, but approximately obeying the 50 MPH speed limit.

[//]: # (Image References)
[image1]: ./examples/SlowDown.png
[image2]: ./examples/LaneChange.png
[image3]: ./examples/FourMile.png
[video1]: ,/examples/PathPlanningRun.mp4

## [Rubric Points](https://review.udacity.com/#!/rubrics/1020/view)
All the code for this project has been derived from the example code in the course and is in this directory.
[Here](https://github.com/gvp-study/CarND-Path-Planning-Project.git)

# Path Planning
## Implementing the Path Planning Module

I implemented the path planning module based on the lessons and looking at links to the implementations suggested by Aaron Brown and David Silver.

 (https://medium.com/self-driving-cars/five-different-udacity-student-controllers-4b13cc8f2d0f)

The path planning system receives input about the real time state of a self driving car in a simulator. The car drives on a six lane road with 3 lanes going each way. The system also receives the state of the 12 other cars going in the same direction in real time.

The self driving car looks at the state of the cars in its current lane and tries to maintain a steady speed below the speed limit. When it finds another car in its lane which is going slower it slows down to avoid colliding with it. The car then tries to find a lane to switch to safely. After the destination lane is obtained, the self driving car then computes a smooth and safe path from its current state to the destination lane.

The path planning takes into consideration, the limits of the vehicle velocity and acceleration limits by using by using a third order spline to ensure a smooth transition between current and future way points. The details of the implementation is given below.

## Map and Waypoints
The track the car navigates through is represented by a series of waypoints recorded in a map file called highway_map.csv. The waypoints in the map are represented by a five element vector (x, y, s, dx, dy). The first two elements (x,y) represents the coordinates of the waypoint in the global coordinate system. s indicates the distance from a starting point in Frenet coordinates. The final two elements (dx, dy) is the tangent vector of the road at that waypoint. Given a point in the track in Frenet coordinates.The helper function getXY() converts any point in Frenet coordinates to the (x, y) coordinates of that point.

At a macro level the path planning problem is to find a set of (s, d) points in time on the 3 lane road to safely drive through.

## Splines
The simulator takes a series of 50 points fed to it in the (next_x_vals, next_y_vals) array. After each cycle, it then returns a list with the points that it already traversed through removed from the list. This returned list form the previous_points.

The code is structured such that a set of (next_x_vals, next_y_vals) are the same as the previous points except that the used up points are replenished with the new points added to the end. The number of the points fed to the simulator is kept constant at 50 points.

To compute these new points, we use a spline through a set of waypoints set in the future. These waypoints are set a distances of approximately 30m, 60m and 90m in front of the vehicle. These points in Frenet coordinates are converted into (x, y) coordinates and fed to a spline function which computes the spline that smoothly goes through those points.

The code that populates the end of the previous points list is shown below. Note that the horizon is set at 30 meters from the last point in the previous list. This distance is then divided into N points based the distance travelled in a cycle time of 20 msec. At a speed of 50mph, N typically is around 70 and the distance increment is .45 meters.

```
// Calculate how to break up spline points so that we travel at our desired reference velocity.
double target_x = 30.0;
double target_y = s(target_x);
double target_dist = hypot(target_x, target_y);

double x_add_on = 0;
double N = (target_dist / (0.02*ref_vel/2.24));
double incr_x = (target_x)/N;

// Fill up the rest of the path planner after the previous points. Here we will always output 50 points.
for(int i = 0; i <= 50-previous_path_x.size(); i++)
  {
    double x_point = x_add_on+incr_x;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // Rotate back to normal after rotating it earlier.
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);

  }
```
## Acceleration and Velocity Limits
The project enforces an acceleration limit of 10m/s/s and a velocity limit of 50mph. Given that the feedback loop is 0.02 seconds, this limits the change in velocity between every cycle to .2m/s. To make sure the velocity limit is not exceeded, the reference velocity is explicitly set to 49.5mph.

## Sensor Fusion and Car Behavior
The telemetry data also provides us with the details of all the 12 other cars in the circuit. They are contained in the sensor fusion packets with the following data (id, x, y, xdot, ydot, s, d). We can easily loop over all the sensor fusion data and find where each car is located relative to our self driving car.

### Collision Avoidance
The sensor fusion data can be used to decide whether a car is in front of us going in the same lane. If there is one, we can decide if it is going slower than the speed limit. If the car in front of the current lane is slower than us, we can slow down gradually till we match the speed of the car in front and avoid a collision. To do this, we first set the current_close_free variable based on the d value of any neighboring car being inside the current lane d limits as shown below. This is based on the fact that the lanes are 4 meter wide and the d is measured from the middle of the 2 lane highway.
```
bool current_lane_free = !(d < (2+4*lane+2) && d > (2+4*lane-2));
```
Once we confirm that the neighboring car is in the same lane when current_lane_free is false, we check the neighboring car's position by checking its s value is within the a 30m window as shown below. If this is true, we set the too_close boolean to true.

```
((check_car_s > car_s) && ((check_car_s-car_s) < 30))
```
The complete section of the code to do this shown below.

```
auto sensor_fusion = j[1]["sensor_fusion"];

int prev_size = previous_path_x.size();

if(prev_size > 0)
{
    car_s = end_path_s;
}
// Find if lane is occupied and decide on lane to change to.
bool too_close = false;
bool left_free = false;
bool right_free = false;
int current_lane = lane;
int left_lane = lane - 1;
int right_lane = lane + 1;
// Go through all other cars. Find ref_v to use.
bool right_lane_free = true;
for(int i = 0; i < sensor_fusion.size(); i++)
{
    // Car is in my lane.
    float d = sensor_fusion[i][6];
    bool current_lane_free = !(d < (2+4*lane+2) && d > (2+4*lane-2));

    if(!current_lane_free)
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = hypot(vx, vy);
      double check_car_s = sensor_fusion[i][5];

      check_car_s += ((double)prev_size*0.02*check_speed);
      // Check s values greater than mine and s gap
      if((check_car_s > car_s) && ((check_car_s-car_s) < 30))
      {
        // Do some logic here, lower reference velocity so we dont crash into the car
        // also flag to try to change lanes.
        too_close = true;
      }
    }
  }
```
### Slowing down and Speeding up
If we detect there is a car too_close to us, we decrease the reference velocity by 0.224 for that cycle. This decrement amounts to a slowdown of based on the deceleration of 5m/s/s which is less than the maximum of 10m/s/s. If we do not see a car blocking our lane, we accelerate by the same rate of 5m/sec till it hits the speed limit. The rapid update of 50 cycles/second makes this slowdown and speedup very smooth and remain within the jerk limits.
```
if(too_close)
{
    ref_vel -= 0.224;
}
else if(ref_vel < speed_limit)
{
    ref_vel += 0.224;
}
```
### Changing Lanes
If there is a car in the same lane ahead of us and is going slower, we can slow down to match the speed of that car. But we can also decide to change lanes if it is safe. To do this safely, we need to check if there are cars in the left or right lanes and decide if it is safe to change lanes if there is a lane free on the left or right of the current lane. The code to do this is shown below.
```
// Check if there is a lane free to switch to.
if(too_close)
{
    bool left_lane_free = true;
    bool right_lane_free = true;
    // Loop over all neighboring cars.
    for(int i = 0; i < sensor_fusion.size(); i++)
    {
      // Car is in left lane.
      float d = sensor_fusion[i][6];
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = hypot(vx, vy);
      double check_car_s = sensor_fusion[i][5];

      check_car_s += ((double)prev_size*0.02*check_speed);
      // Check s values greater than mine and s gap
      if((check_car_s > car_s) && ((check_car_s-car_s) < 30))
      {
        left_lane_free &= (lane >= 0 && !(d < (2+4*left_lane+2) && d > (2+4*left_lane-2)));
        right_lane_free &= (lane < 2 && !(d < (2+4*right_lane+2) && d > (2+4*right_lane-2)));
      }
    }
    bool changed = false;
    if(!changed && lane > 0)
    {
      if(left_lane_free)
      {
        printf("Current Lane %d Left Lane changed to %d\n", lane, lane-1 > -1 ? lane-1 : lane);
        lane = lane-1 > -1 ? lane-1 : lane;
        changed = true;
      }
    }
    if(!changed && lane < 2)
    {
      if(right_lane_free)
      {
        printf("Current Lane %d Right Lane changed to %d\n", lane, lane+1 < 3 ? lane+1 : lane);
        lane = lane+1 < 3 ? lane+1 : lane;
        changed = true;
      }
    }
  }
}
```
### Making of the Spline
The code below shows the building of the 5 points making up the anchor points of the spline. The first two points are obtained from the trailing 2 points from the previous path list. The three other points are spaced 30m, 60m, 90m apart in front of the car and obtained from the map waypoints. These five points are originally based in the global coordinates which we transform to the local car coordinates. The five points in the local coordinates are then fed to the spline function to serve as anchor points.

```
// Create a lit of widely spaced waypoints (x,y) which are evenly spaced at 30m.
// Later we will interpolate these waypoints with a spline and fill it in with more points
vector<double> ptsx;
vector<double> ptsy;

// Reference x, y, yaw states.
// We will reference the starting point as where the car is at the previous point.
double ref_x = car_x;
double ref_y = car_y;
double ref_yaw = deg2rad(car_yaw);

// If previous size is small, use the car as the starting refrence.
if(prev_size < 2)
{
  // Use two points that make the path tangent to the car.
  double prev_car_x = car_x - cos(ref_yaw);
  double prev_car_y = car_y - sin(ref_yaw);

  ptsx.push_back(prev_car_x);
  ptsx.push_back(car_x);

  ptsy.push_back(prev_car_y);
  ptsy.push_back(car_y);
}
// Use the previous path and end point as starting reference.
else
{
  // Redefine reference state as previous path end point.
  ref_x = previous_path_x[prev_size-1];
  ref_y = previous_path_y[prev_size-1];

  double ref_x_prev = previous_path_x[prev_size-2];
  double ref_y_prev = previous_path_y[prev_size-2];
  ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

  // Use two points that make the path tangent to previous path end point.
  ptsx.push_back(ref_x_prev);
  ptsx.push_back(ref_x);

  ptsy.push_back(ref_y_prev);
  ptsy.push_back(ref_y);
}
//
// In Frenet coords, add evenly 30m spaced points ahead of the starting reference.
//
vector<double> next_wp0 = getXY(car_s+30.0, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s+60.0, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s+90.0, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);

ptsy.push_back(next_wp0[1]);
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);
//
// Shift the coords to car local frame.
//
for(int i = 0; i < ptsx.size(); i++)
{
  // Shift car coordinates from local to global.
  double shift_x = ptsx[i]-ref_x;
  double shift_y = ptsy[i]-ref_y;

  ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
  ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
}

// Create a spline.
tk::spline s;

// Set (x,y) points to the spline.
s.set_points(ptsx, ptsy);

```
### Using Spline Points to Fill Next list
Once we have the spline defined, we find the gap between the previous points list with the spline points till we have 50 points in the next_x_vals and next_y_vals list. This ensures the smooth transition between the previous path and the waypoints computed from the lane change and map waypoints computed from the behavior decided on earlier.



### Figures
The pictures below show the state of the car at 3 instants of time.
The picture below shows it when it is slowing down behind another slower car.
![alt text][image1]
The picture below shows the car changing lanes.
![alt text][image2]
The image below shows the car reaching the 4 mile point without any incidents.
![alt text][image3]
I let the simulator run several laps over time to make sure that the collision free path planning works as intended.




### Output
The movie of the simulator controlled by the MPC controller is shown below.[link to my video](./examples/PathPlanningRun.mp4)
![alt text][video1]

## References
