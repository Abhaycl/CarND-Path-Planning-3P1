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
3. A quintic polynomial, jerk-minimizing (JMT) trajectory is produced for each available state and target (*note: although this trajectory was used for the final path plan in a previous approach, in the current implementation the JMT trajectory is only a rough estimate of the final trajectory based on the target state and using the 'spline.h' library).
4. Each trajectory is evaluated according to a set of cost functions, and the trajectory with the lowest cost is selected, these cost functions include:
  - Buffer cost: penalizes a trajectory that comes within a certain distance of another traffic vehicle trajectory.
  - In-lane buffer cost: penalizes driving in lanes with relatively nearby traffic.
  - Efficiency cost: penalizes trajectories with lower target velocity.
  - Not-middle-lane cost: penalizes driving in any lane other than the center in an effort to maximize available state options.

### 5. Produce New Path

The new path starts with a certain number of points from the previous path, which is received from the simulator at each iteration. From there a spline is generated beginning with the last two points of the previous path that have been kept (or the current position, heading, and velocity if no current path exists), and ending with two points 30 and 60 meters ahead and in the target lane. This produces a smooth x and y trajectory. To prevent excessive acceleration and jerk, the velocity is only allowed increment or decrement by a small amount, and the corresponding next x and y points are calculated along the x and y splines created earlier.

## Conclusion

The resulting path planner works well, but not perfectly. It has managed to accumulate incident-free runs of over ten miles multiple times, and once navigating the track incident-free for over twenty miles (for which the image below is evidence). Improving the planner from this point is difficult due to the infrequency of infractions and inability to duplicate the circumstances that led up to an infraction. Overall, I am very satisfied with its performance.


![Final score][image5]

![Final score][image1]

![Final score][image2]

![Final score][image3]

![Final score][image4]