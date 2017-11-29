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
#include "PID.h"
#include "spline.h"

using namespace std;

// For convenience
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

double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {
    double closestLen = 100000; //large number
    int closestWaypoint = 0;
    
    for(int i = 0; i < maps_x.size(); i++) {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x, y, map_x, map_y);
        if(dist < closestLen) {
            closestLen = dist;
            closestWaypoint = i;
        }
    }
    return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
    int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);
    
    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];
    
    double heading = atan2((map_y - y), (map_x - x));
    double angle = fabs(theta - heading);
    angle = min(2 * pi() - angle, angle);
    
    if (angle > pi() / 4) {
        closestWaypoint++;
        if (closestWaypoint == maps_x.size()) {
            closestWaypoint = 0;
        }
    }
    return closestWaypoint;
}

// Transform from Cartesian x, y coordinates to Frenet s, d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
    int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);
    
    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0) {
        prev_wp = maps_x.size() - 1;
    }
    
    double n_x = maps_x[next_wp] - maps_x[prev_wp];
    double n_y = maps_y[next_wp] - maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];
    
    // Find the projection of x onto n
    //double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_norm = (x_x * n_x + x_y * n_y) / (pow(n_x, 2) + pow(n_y, 2));
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;
    double frenet_d = distance(x_x, x_y, proj_x, proj_y);
    
    // See if d value is positive or negative by comparing it to a center point
    double center_x = 1000 - maps_x[prev_wp];
    double center_y = 2000 - maps_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);
    
    if (centerToPos <= centerToRef) {
        frenet_d *= -1;
    }
    
    // Calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++) {
        frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
    }
    
    frenet_s += distance(0, 0, proj_x, proj_y);
    
    return {frenet_s, frenet_d};
}

// Transform from Frenet s, d coordinates to Cartesian x, y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
    int prev_wp = -1;
    
    while(s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1))) {
        prev_wp++;
    }
    
    int wp2 = (prev_wp + 1) % maps_x.size();
    
    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
    // The x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);
    
    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);
    
    double perp_heading = heading - pi() / 2;
    
    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);
    
    return {x, y};
}

// Start in lane 1 (center lane)
int lane = 1;
// Have a reference velocity to target
double ref_vel = 0.0; //mph

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
    
    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            auto s = hasData(data);
            
            if (s != "") {
                auto j = json::parse(s);
                
                string event = j[0].get<string>();
                
                if (event == "telemetry") {
                    // j[1] Is the data JSON object
                    
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
                    
                    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
                    
                    // Velocity controller
                    PID vel_controller;
                    vel_controller.Init(0.005, 0.0, 0.0);
                    
                    // Save lane
                    int n_lane = lane;
                    int delay = 0;
                    bool too_close = false;
                    bool left_free = true;
                    bool right_free = true;
                    // Set a safety space
                    double safety_space = 30;
                    double left_space = 10000.0;
                    double right_space = 10000.0;
                    double speed_limit = 49.5; //mph
                    
                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];
                    //vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
                    
                    int prev_size = previous_path_x.size();
		            
                    if (prev_size > 0) {
                        car_s = end_path_s;
                    }
                    
                    // Check which lanes are not free depending on the vehichle's lane
                    if (lane == 0) { // Left lane
                        left_free = false;
                    }
                    if (lane == 2) { // Right lane
                        right_free = false;
                    }
                    
                    // Check through the data (cars) from sensor fussion output
                    for(int i = 0; i < sensor_fusion.size(); i++) {
                        // Car is in my lane
                        float d = sensor_fusion[i][6];
                        
                        // Observe lane of car
                        int obs_lane = fabs(d / 4);
                        
                        // Check the velocity of the car in my lane
                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];
                        double check_speed = sqrt(pow(vx, 2) + pow(vy, 2));
                        double check_car_s = sensor_fusion[i][5];
                        //double check_car_d = sensor_fusion[i][6];
                        // Project where the car might be in the future
                        check_car_s += ((double)prev_size * 0.02 * check_speed);
                        double space = check_car_s - end_path_s;
                        
                        // Want to track the nearest car to the ego car
                        double car_dist = check_car_s - car_s;
                        
                        // Check cars within dangerous range
                        // If car is in front, within 30m
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
                        
                        // If observed car is in my lane
                        if ((d < 2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
                            // If observed car is too close
                            if ((check_car_s > car_s) && ((check_car_s - car_s) < safety_space)) {
                                too_close = true;
                                
                                // If an observed car is on the left side
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
                    } // End of search through sensor fussion
                    
                    // If car is too close
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
                    
                    // To prevent instantaneous lane change by applying delay
                    if (n_lane != lane) {
                        delay ++;
                    } else {
                        delay = 0;
                    }
                    if (delay < 12) {
                        n_lane = lane;
                    } else {
                        delay = 0;
                    }
                    
                    /*
                        // if observed car is in my lane:
                        if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
                            double vx = sensor_fusion[i][3];
                            double vy = sensor_fusion[i][4];
                            //double check_speed = sqrt(vx * vx + vy * vy);
                            double check_speed = sqrt(pow(vx, 2) + pow(vy, 2));
                            double check_car_s = sensor_fusion[i][5];
                            
                            // If using previous points can project s value output
                            check_car_s += ((double)prev_size * 0.02 * check_speed);
                            // Check s values greater than mine and s gap
                            if ((check_car_s > car_s) && (check_car_s - car_s) < 30) {
                                // Do some logic here, lower reference velocity so we do not crash into the car infront of us, could also flag to try to change lanes
                                //ref_vel = 29.5; //mph
                                too_close = true;
                                if (lane > 0) {
                                    lane = 0;
                                }
                            }
                        }
                    }*/
                    
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
                    
                    /*if (too_close) {
                        ref_vel -= 0.224;
                    } else if (ref_vel < 49.5) {
                        ref_vel += 0.224;
                    }*/
                    
                    // Create a list of widely spaced (x, y) waypoints, evenly spaced at 30m, later we will interpolate these waypoints with a spline and fill it in with more points that control speed
                    vector<double> ptsx;
                    vector<double> ptsy;
                    
                    // Reference x, y, yaw states, either we will reference the starting point as where the car is or at the previous paths end point
                    double ref_x = car_x;
                    double ref_y = car_y;
                    double ref_yaw = deg2rad(car_yaw);
                    
                    // If previous size is almost empty, use the car as starting reference
                    if (prev_size < 2) {
                        // Use two points that make the path tangent to the car
                        double prev_car_x = car_x - cos(car_yaw);
                        double prev_car_y = car_y - sin(car_yaw);
                        
                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(car_x);
                        ptsy.push_back(prev_car_y);
                        ptsy.push_back(car_y);
                    // Use the previous path is end point as starting reference
                    } else {
                        // Redefine reference state as previous path end point
                        ref_x = previous_path_x[prev_size - 1];
                        ref_y = previous_path_y[prev_size - 1];
                        
                        double ref_x_prev = previous_path_x[prev_size - 2];
                        double ref_y_prev = previous_path_y[prev_size - 2];
                        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
                        
                        // Use two points that make the path tangent to the previous path is end point
                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_x);
                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back(ref_y);
                    }
                    
                    // In Frenet add evenly 20m spaced points ahead of the starting reference
                    vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    vector<double> next_wp1 = getXY(car_s + 50, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    vector<double> next_wp2 = getXY(car_s + 70, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    vector<double> next_wp3 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    
                    ptsx.push_back(next_wp0[0]);
                    ptsx.push_back(next_wp1[0]);
                    ptsx.push_back(next_wp2[0]);
                    ptsx.push_back(next_wp3[0]);
                    ptsy.push_back(next_wp0[1]);
                    ptsy.push_back(next_wp1[1]);
                    ptsy.push_back(next_wp2[1]);
                    ptsy.push_back(next_wp3[1]);
                    
                    for(int i = 0; i < ptsx.size(); i++) {
                        // Shift car reference angle to 0 degrees
                        double shift_x = ptsx[i] - ref_x;
                        double shift_y = ptsy[i] - ref_y;
                        
                        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
                        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
                    }
                    
                    // Create a spline
                    tk::spline s;
                    
                    // Set (x, y) points to the spline
                    s.set_points(ptsx, ptsy);
                    
                    // Define the actual (x, y) point we will use for the planner
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;
                    
                    // Start with all of the previous path points from last time
                    for(int i = 0; i < previous_path_x.size(); i++) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }
                    
                    // Calculate how to break up spline points so that we travel at our desired reference velocity
                    double target_x = 30.0;
                    double target_y = s(target_x);
                    //double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
                    double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
                    double x_add_on = 0;
                    
                    // Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
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
                    
                    json msgJson;
                    
                    //vector<double> next_x_vals;
                    //vector<double> next_y_vals;
                    
                    /*
                    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
                    
                    double dist_inc = 0.3;  //50 miles
                    for(int i = 0; i < 50; i++) { //50 points
                        double next_s = car_s +( i + 1) * dist_inc;
                        double next_d = 6;
                        vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                        
                        next_x_vals.push_back(xy[0]);
                        next_y_vals.push_back(xy[1]);
                    }*/
					
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
    
    // We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // I guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });
    
    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
        // Have a reference velocity to target
        ref_vel = 0.0; //mph
    });
    
    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
