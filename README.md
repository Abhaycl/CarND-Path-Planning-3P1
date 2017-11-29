# Path Planning Project Starter Code

The goal of this project is to build a path planner that creates smooth, safe trajectories for the car to follow. The highway track has other vehicles, all going different speeds, but approximately obeying the 50 MPH speed limit.

The car transmits its location, along with its sensor fusion data, which estimates the location of all the vehicles on the same side of the road.

<!--more-->

[//]: # (Image References)

[image1]: /build/result.jpg "Sample final score"
[image2]: /build/result1.jpg "Sample final score"
[image3]: /build/result2.jpg "Sample final score"
[image4]: /build/result3.jpg "Sample final score"
[image5]: /build/result4.jpg "Sample final score"

#### How to run the program

```sh
1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./path_planning
6. and run the simulator and select Project 1: Path Planning
```

The summary of the files and folders int repo is provided in the table below:

| File/Folder               | Definition                                                                                  |
| :------------------------ | :------------------------------------------------------------------------------------------ |
| src/json.hpp              | Various definitions.                                                                        |
| src/PID.cpp               | Initializes variables, updates errors, error totalizer.                                     |
| src/PID.h                 | Definition of the package of pid.                                                           |
| src/main.cpp              | Has several functions within main(), communicates with the Term 3 Simulator receiving       |
|                           | data measurements, calls a function to run the Path Planning, these all handle the          |
|                           | uWebsocketIO communication between the simulator and it's self.                             |
|                           |                                                                                             |
| src                       | Folder where are all the source files of the project.                                       |
| build                     | Folder where the project executable has been compiled.                                      |
|                           |                                                                                             |


---
## Implementation

This implementation is summarized in the following five steps:
1. Construct interpolated waypoints of nearby area
2. Determine ego car parameters
3. Generate predictions from sensor fusion data
4. Determine best trajectory
5. Produce new path

### 1. Construct Interpolated Waypoints of Nearby Area 

The track waypoints given in the 'highway_map.csv' file are spaced roughly 30 meters apart, so the first step in the process is to interpolate a set of nearby map waypoints and produce a set of much more tightly spaced waypoints which help to produce more accurate results from the 'getXY' and 'getFrenet' methods and also account for the discontinuity in 's' values at the end/beginning of the track.

### 2. Determine Ego Car Parameters

The simulator returns instantaneous telemetry data for the ego vehicle, but it also returns the list of points from previously generated path. This is used to project the car's state into the future and a "planning state" is determined based on the difference between points at some prescribed number of points along the previous path. In effect, this can help to generate smoother transitions, handle latency from transmission between the controller and the simulator, and alleviate the trajectory generator of some computation overhead.

### 3. Generate Predictions from Sensor Fusion Data

The sensor fusion data received from the simulator in each iteration is parsed and trajectories for each of the other cars on the road are generated. These trajectories match the duration and interval of the ego car's trajectories generated for each available state and are used to determine a best trajectory for the ego car.

### 4. Determine Best Trajectory

Using the ego car "planning state", sensor fusion predictions an optimal trajectory is produced. 

1. Available states are updated based on the ego car's current position, with some extra assistance from immediate sensor fusion data (I think of this similar to ADAS, helping to, for example, prevent "lane change left" as an available state if there is a car immediately to the left). 
2. Each available state is given a target Frenet state (position, velocity, and acceleration in both s and d dimensions) based on the current state and the traffic predictions. 
3. A quintic polynomial, jerk-minimizing (JMT) trajectory is produced for each available state and target (*note: although this trajectory was used for the final path plan in a previous approach, in the current implementation the JMT trajectory is only a rough estimate of the final trajectory based on the target state and using the 'spline.h' library*).
4. Each trajectory is evaluated according to a set of cost functions, and the trajectory with the lowest cost is selected, these cost functions include:
  - Buffer cost: penalizes a trajectory that comes within a certain distance of another traffic vehicle trajectory.
  - In-lane buffer cost: penalizes driving in lanes with relatively nearby traffic.
  - Efficiency cost: penalizes trajectories with lower target velocity.
  - Not-middle-lane cost: penalizes driving in any lane other than the center in an effort to maximize available state options.

### 5. Produce New Path

The new path starts with a certain number of points from the previous path, which is received from the simulator at each iteration. From there a spline is generated beginning with the last two points of the previous path that have been kept (or the current position, heading, and velocity if no current path exists), and ending with two points 30 and 60 meters ahead and in the target lane. This produces a smooth x and y trajectory. To prevent excessive acceleration and jerk, the velocity is only allowed increment or decrement by a small amount, and the corresponding next x and y points are calculated along the x and y splines created earlier.

## Detail

**1.-** A proportional speed control has been used, here the target speed of the ego-vehicle is adjusted according to the state of the vehicle. At the beginning of the project the initial speed of the ego-vehicle is 0 mph which gradually increases. If there is no vehicle in front of the ego-vehicle, the target speed is set to 49.5 mph, which is just below the speed limit. However, if the ego-vehicle is near another vehicle in front of it, it adjusts the target speed to the vehicle ahead and maintains a safety distance of 30m between them.

```cpp
......Lines 222
// Velocity controller
PID vel_controller;
vel_controller.Init(0.005, 0.0, 0.0);
......Lines 224

......Lines 383
// Target velocity control
if (too_close) {
    // Keeping lane
    speed_limit = 0.0; //mph
} else {
    speed_limit = 49.5; //mph
}

double vel_error = ref_vel - speed_limit;
vel_controller.UpdateError(vel_error);
double new_vel = vel_controller.TotalError();
ref_vel += new_vel;
......Lines 394
```

**2.-** The logic to avoid occupied lane is employed by the ego-vehicle that is able to drive safely avoiding occupied lanes to the left and right. At each step of time, the logic checks that (I) other cars in front of the ego-vehicle, (II) cars in adjacent lanes, and (III) safety space in front of 30m and behind the ego-vehicle 20m. Therefore, (VI) the ego-vehicle only changes lanes if there are no cars within defined safety ranges in front of and behind the ego vehicle as well as both sides of the permitted lanes. The logic is shown in the following code fragment:

**NOTE:** *line = 0* is right lane, *line = 1* is center lane, *line = 2* is left lane

```cpp
......Lines 248
// Check which lanes are not free depending on the vehichle's lane
if (lane == 0) { // Left lane
    left_free = false;
}
if (lane == 2) { // Right lane
    right_free = false;
}
......Lines 254

......Lines 258
// Car is in my lane                                             (II)
float d = sensor_fusion[i][6];
                        
// Observe lane of car
int obs_lane = fabs(d / 4);
......Lines 262

......Lines 274
// Want to track the nearest car to the ego car
double car_dist = check_car_s - car_s;

// Check cars within dangerous range
// If car is in front, within 30m                                (III)
if ((car_dist > 0) && (car_dist < 30)) {
    // If an observed car is on the left side
    if (obs_lane == (lane - 1)) {
        if (space < left_space) { left_space = space; }
        left_free = false;
    }
    // If an observed car is on the right side
    if (obs_lane == (lane + 1)) {
        if (space < right_space) { right_space = space; }
        right_free = false;
    }
// If car is at the back, within 20m
} else if ((car_dist < 0) && (car_dist > -20)) {
    // If an observed car is on the left side
    if (obs_lane == (lane - 1)) {
        left_free = false;
    }
    // If an observed car is on the right side
    if (obs_lane == (lane + 1)) {
        right_free = false;
    }
}

// If observed car is in my lane                                 (I)
if ((d < 2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
    // If observed car is too close
    if ((check_car_s > car_s) && ((check_car_s - car_s) < safety_space)) {
        too_close = true;

        // If an observed car is on the left side                (II)
        if (obs_lane == (lane - 1)) {
            //if (space < left_space) { left_space = space; }
            left_free = false;
        }
        // If an observed car is on the right side
        if (obs_lane == (lane + 1)) {
            //if (space < right_space) { right_space = space; }
            right_free = false;
        }
    }
}
......Lines 319

......Lines 322
// If car is too close                                           (VI)
if (too_close) {
    // cout << "\nleft_side: " << left_free << ", " << left_space
    //      << " right_side: " << right_free << ", " << right_space
    //      << endl;
    if ((lane == 0) && right_free) { // Left lane and 
        lane = 1;
    } else if (lane == 1) {
        if (left_free && right_free) {
            if (right_space > left_space) {
                lane += 1;
            } else {
                lane -= 1;
            }
        } else if (left_free) {
            lane -= 1;
        } else if (right_free) {
            lane += 1;
        }
    } else if ((lane == 2) && left_free) {
        lane = 1;
    }
}

if (lane > 2) { lane = 2; }
......Lines 346
```

**3.-** For trajectory generation using a spline function, after a desired lane of the ego-vehicle has been established, we create the coordinates (x, y) of the waypoints in front of the ego-vehicle using the corresponding frenet spaced coordinates (s, d). From these waypoints, we use a spline function to generate evenly spaced points for a smooth path to the desired horizon. The car is expected to visit each of these points every 0.02 seconds, so the number of points is calculated by n = (target_dist / (0.02 * ref_vel / 2.24) to travel at the desired reference speed. In the implementation, 6 waypoints were created and a 30m horizon was chosen to generate the desired path. More details are shown in the code fragment below:

```cpp
......Lines 462
// Create a spline
tk::spline s;

// Set (x, y) points to the spline
s.set_points(ptsx, ptsy); // The anchor points / waypoints
......Lines 466

......Lines 485
// Fill up the rest of our path planner after filling it with previous points,
// here we will always output 50 points
for(int i = 1; i <= 50 - previous_path_x.size(); i++) {
    // Conversion to m/s
    double n = (target_dist / (0.02 * ref_vel / 2.24));
    double point_x = x_add_on + (target_x / n);
    double point_y = s(point_x);

    x_add_on = point_x;

    double x_ref = point_x;
    double y_ref = point_y;

    // Rotate back to normal after rotating it earlier
    point_x = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    point_y = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    point_x += ref_x;
    point_y += ref_y;

    next_x_vals.push_back(point_x);
    next_y_vals.push_back(point_y);
}
......Lines 506
```

Using the above implementation, the ego-vehicle is able to drive at least 4.32 miles without incidents, including; exceeding the speed limit, exceeding the max acceleration and jerk, collisions, and going out of lane.

## Conclusion

The resulting path planner works well, but not perfectly. It has managed to accumulate incident-free runs of over ten miles multiple times, and once navigating the track incident-free for over twenty miles (for which the image below is evidence). Improving the planner from this point is difficult due to the infrequency of infractions and inability to duplicate the circumstances that led up to an infraction. Overall, I am very satisfied with its performance.


![Final score][image5]

![Final score][image1]

![Final score][image2]

![Final score][image3]

![Final score][image4]
