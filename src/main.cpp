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
#include <math.h>
#include <algorithm>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

const double speedlim = 50;
const double min_gap = 30;

double speed_cost(int hostlane, int currlane, double hostspeed, double targetfrontspeed, double targetrearspeed){
  double cost;
  if (currlane == hostlane){
    cost = 1 - exp(-speedlim/targetfrontspeed);
  } else {
    cost = (1 - exp(-speedlim/targetfrontspeed))*0.2 +  (1 - exp(-targetrearspeed/hostspeed))*0.8;
  }
  return cost;
}

double collision_cost (int hostlane, int currlane, double frontgap, double reargap){
  double cost;
  if (currlane == hostlane){
    cost = 1/(1+ exp(-(min_gap - frontgap)/frontgap));
  } else {
    // buffer is to ensure safe lane change
    // add heavy cost for potential collision in adjacent lanes
    double buffer = 20;
    if ((frontgap < (min_gap + buffer)) or (reargap < (min_gap+buffer))){
      cost = 1;
    } else {
      cost = 0;
      }    
  }  
}
//frequent lane change cost
double freqlc_cost (double lc_timer){
  double cost;
  if (lc_timer ==0){
    cost = 0;
  } else {
    cost = 1 - exp(-1/lc_timer);
  }
  return cost;
}


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
  double ref_vel = 0.0; // mph
  int lc_timer = 0; //lane change timer

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, &lc_timer]
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

         // std::cout<<"car_x:"<<car_x<<",car_y"<<car_y<<",car yaw: "<<car_yaw<<std::endl;
          
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

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           *  define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          double max_accel = 10;

          int prev_size = previous_path_x.size();
          double wlane = 4; //lane width
          double time_step = 0.02;
          bool too_close = false;
          bool speedctrl = true;
          double hostlanespeed = 99999; // inital to a large value, will search for smallest in the loop of target vehilces
          double leftlanefrontspeed = 99999; 
          double rightlanefrontspeed = 99999;
          double leftlanerearspeed = 99999; 
          double rightlanerearspeed = 99999;          
          
          double leftlanefrontgap = 99999;  // host vehicle gap with respect to the vehicle immediately ahead
          double rightlanefrontgap = 99999; // host vehicle gap with respect to the vehicle immediately ahead
          double leftlanereargap = 99999;  // host vehicle gap with respect to the vehicle immediately behind
          double rightlanereargap = 99999; // host vehicle gap with respect to the vehicle immediately behind        
          double hostlanegap = 99999;
          
          // set car_s to the end path if prev_size>0
          if (prev_size > 0){
            car_s = end_path_s;
          }
          
           // loop through all targets
          for (int i=0; i<sensor_fusion.size(); ++i){
            float d = sensor_fusion[i][6];
           
          //target car is in the host lane
          if ( d>wlane*lane and d<(lane+1)*wlane){
              // target car's speed
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double vtarget = sqrt(vx*vx + vy*vy); //target speed
              double starget = sensor_fusion[i][5]; //target s 

              // min speed at the current lane for the target ahead
              if ((starget > end_path_s) and (vtarget < hostlanespeed)){
                hostlanespeed = vtarget;
              }

              // estimated target s with previous points
              starget += (double) prev_size * time_step * vtarget;

              // min gap at the current lane for the target ahead
              if ((starget > end_path_s) and ((starget - end_path_s) < hostlanegap)){
                hostlanegap = starget - end_path_s;
              }           

              if (hostlanegap < min_gap){
                // collision waning
                too_close = true;  
                max_accel = (1 - hostlanegap/min_gap)*5+10;
              }  //end ifelse collision waring
            }//end ifelse vehicle in lane
            
          //target vehicle in the left ajacent lane
          if (lane>0){
             if ( d>wlane*(lane-1) and d<(lane)*wlane){
               // target car's speed
               double vx = sensor_fusion[i][3];
               double vy = sensor_fusion[i][4];
               double vtarget = sqrt(vx*vx + vy*vy); //target speed
               double starget = sensor_fusion[i][5]; //target s 

               // min speed at the left adjacent lane for the target ahead
               if ((starget > end_path_s) and (vtarget < leftlanefrontspeed)){
                 leftlanefrontspeed = vtarget;
               }    
             
               // estimated target s with previous points
               starget += (double) prev_size * time_step * vtarget;
             
               // min gap at the left adjacent lane for the target ahead
               if ((starget > end_path_s) and ((starget - end_path_s) < leftlanefrontgap)){
                 leftlanefrontgap = starget - end_path_s;
               }
               
               // min gap at the left adjacent lane for the target behind
               if ((starget < end_path_s) and ((end_path_s - starget) < leftlanereargap)){
                 leftlanereargap = end_path_s - starget;
                 leftlanerearspeed = vtarget;
               }
               
               // if the target ahead plan right lane change
               if ((leftlanefrontgap < min_gap) and ( d>wlane*((lane-1)+0.75))){
                 too_close = true;
                 max_accel = (1 - leftlanefrontgap/min_gap)*10+10;
               }            
              } // end left adjacent lane            
          } else { //no left ajacent lane
              leftlanefrontspeed = 0;
              leftlanefrontgap = 0; 
              leftlanerearspeed = 0;
              leftlanereargap = 0; 
          }  //end ifelse vehicle in left adjacent lane

          //target vehicle in the right adjacent lane
          if (lane<2){
              if ( d>wlane*(lane+1) and d<(lane+2)*wlane){
                // target car's speed
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double vtarget = sqrt(vx*vx + vy*vy); //target speed
                double starget = sensor_fusion[i][5]; //target s 
             
                // min speed at the left adjacent lane for the target  ahead
                if ((starget > end_path_s) and (vtarget < rightlanefrontspeed)){
                  rightlanefrontspeed = vtarget;
                }
             
                // estimated target s with previous points
                starget += (double) prev_size * time_step * vtarget;

                // min gap at the right adjacent lane for the target ahead
                if ((starget > end_path_s) and ((starget - end_path_s) < rightlanefrontgap)){
                  rightlanefrontgap = starget - end_path_s;
                } 
                // min gap at the right adjacent lane for the target behind
                if ((starget < end_path_s) and ((end_path_s - starget) < rightlanereargap)){
                  rightlanereargap = end_path_s - starget;
                  rightlanerearspeed = vtarget;
                } 
                
                // if the target ahead plan left lane change
               if ((rightlanefrontgap < min_gap) and (d<(lane+1+0.25)*wlane)){
                  too_close = true;
                  max_accel = (1 - rightlanefrontgap/min_gap)*10+10;
               }
              } // end target in the right adjacent lane
           } else { //no right adjacent lane
            rightlanefrontspeed = 0;
            rightlanefrontgap = 0;
            rightlanerearspeed = 0;
            rightlanereargap = 0;
           }  //end ifelse vehicle in right adjacent lane                    
          } // end for loop of target vehicles
          
          std::cout<<"too close:"<<too_close<<std::endl;
          // too close, cost function
          if (too_close){
            
            //determin possible lane & cost
            vector<int> possible_lane;
            vector<double> totalcost;
            double wspeed = 0.1;
            double wcollision = 0.8;
            double wfreqlc = 0.1;
            double calc_cost;
            if (lane == 0){
              possible_lane.push_back(0);
              possible_lane.push_back(1);
              calc_cost = wspeed*speed_cost(0, 0, ref_vel,hostlanespeed, 0) + wcollision*collision_cost (0, 0, hostlanegap, 0) + wfreqlc * 0;
              std::cout<<"lane 0 cost:"<<calc_cost<<std::endl;
              totalcost.push_back(calc_cost);
              calc_cost = wspeed*speed_cost(0, 1, ref_vel,rightlanefrontspeed, rightlanerearspeed) + wcollision*collision_cost (0, 1, rightlanefrontgap, rightlanereargap)
                + wfreqlc *freqlc_cost(lc_timer);
              totalcost.push_back(calc_cost);
              std::cout<<"lane 1 cost:"<<calc_cost<<std::endl;
            }
            else if (lane ==1){
              possible_lane.push_back(0);
              possible_lane.push_back(1);
              possible_lane.push_back(2);
              calc_cost = wspeed*speed_cost(1, 0,ref_vel, leftlanefrontspeed, leftlanerearspeed) + wcollision*collision_cost (1, 0, leftlanefrontgap, leftlanereargap)
                 + wfreqlc *freqlc_cost(lc_timer);
              std::cout<<"lane 0 cost:"<<calc_cost<<std::endl;
              totalcost.push_back(calc_cost);
              calc_cost = wspeed*speed_cost(1, 1,ref_vel, hostlanespeed, 0) + wcollision*collision_cost (1, 1, hostlanegap, 0)+ wfreqlc * 0;
              totalcost.push_back(calc_cost);
              std::cout<<"lane 1 cost:"<<calc_cost<<std::endl;
              calc_cost = wspeed*speed_cost(1, 2,ref_vel, rightlanefrontspeed, rightlanerearspeed) + wcollision*collision_cost (1, 2, rightlanefrontgap, rightlanereargap)
                 + wfreqlc *freqlc_cost(lc_timer);
              totalcost.push_back(calc_cost);
              std::cout<<"lane 2 cost:"<<calc_cost<<std::endl;
            }
            else if (lane ==2){
              possible_lane.push_back(1);
              possible_lane.push_back(2);
              calc_cost = wspeed*speed_cost(2, 1,ref_vel, leftlanefrontspeed, leftlanerearspeed) + wcollision*collision_cost (2, 1, leftlanefrontgap, leftlanereargap)
                 + wfreqlc *freqlc_cost(lc_timer);
              std::cout<<"lane 1 cost:"<<calc_cost<<std::endl;
              totalcost.push_back(calc_cost);
              calc_cost = wspeed*speed_cost(2, 2,ref_vel, hostlanespeed, 0) + wcollision*collision_cost (2, 2, hostlanegap, 0)+ wfreqlc * 0;
              totalcost.push_back(calc_cost);
              std::cout<<"lane 2 cost:"<<calc_cost<<std::endl;
            }
            
            int mincostIdx = std::min_element(totalcost.begin(),totalcost.end()) - totalcost.begin();
            int new_lane = possible_lane[mincostIdx];
            std::cout<<"curr lane is:"<<lane<<", new lane is:"<<new_lane<<std::endl;
            if (lane == new_lane) {
              // reduce speed
              ref_vel -= max_accel * time_step;
            } else {
              // lane change
              lane = new_lane;
              // reset lane change request counter and count the time to previous lane change
              lc_timer = 0;
              lc_timer ++;
            }
          } //end too close 
          else {
            // speed control - limit to 49.5mph
            if (ref_vel < 49.5){
              ref_vel += max_accel*time_step;
            }
            //count time from the previous lane change
            if (lc_timer>0){
              lc_timer ++;
            }
          }
                    
          //std::cout<<"ref vel" << ref_vel<<std::endl;
          //std::cout<<"lane:"<<lane<<std::endl;
          //std::cout<<"pre size:"<<prev_size<<std::endl;
          //host coordinate in the map coordinate
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          vector<double> anchor_x; 
          vector<double> anchor_y;
          double dist_ahead = 30;        
          
          // create 5 points for spline 
          // use the previous two points and three new points
          if (prev_size <2){
            // use car location & a line tanggent to car heading
            double prev_x = car_x - 50*0.02*cos(ref_yaw);
            double prev_y = car_y - 50*0.02*sin(ref_yaw);
            anchor_x.push_back(prev_x);
            anchor_x.push_back(car_x);
            anchor_y.push_back(prev_y);
            anchor_y.push_back(car_y);
          } else {
            ref_x = previous_path_x[prev_size -1];
            ref_y = previous_path_y[prev_size - 1];
            
            double prev_ref_x = previous_path_x[prev_size -2];
            double prev_ref_y = previous_path_y[prev_size -2];            
            ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);     
            anchor_x.push_back(prev_ref_x);
            anchor_x.push_back(ref_x);
            anchor_y.push_back(prev_ref_y);
            anchor_y.push_back(ref_y);           
            
          } // end if else for previous points
          //std::cout<<"ref yaw:"<<ref_yaw<<std::endl;
          // three new points are defined in Frenet coordinate, each with adding dist_ahead
          vector<double> future_path;
          for (int i=1; i<=3; ++i){
            future_path = getXY(car_s + (double) i*dist_ahead, wlane*(lane + 0.5), map_waypoints_s,map_waypoints_x,map_waypoints_y);
            anchor_x.push_back(future_path[0]);
            anchor_y.push_back(future_path[1]);
          }
          //std::cout<<"anchor x:"<<anchor_x[0]<<", "<<anchor_x[1]<<", "<<anchor_x[2]<<", "<<anchor_x[3]<<", "<<anchor_x[4]<<std::endl;
          //std::cout<<"anchor y:"<<anchor_y[0]<<", "<<anchor_y[1]<<", "<<anchor_y[2]<<", "<<anchor_y[3]<<", "<<anchor_y[4]<<std::endl;
          // shift and roate the anchor points from map coordinate to reference cordinate
          vector<double> anchor_tocar_x;
          vector<double> anchor_tocar_y;
          for (int i=0; i<anchor_x.size();++i){
            double shift_x = anchor_x[i] - ref_x;
            double shift_y = anchor_y[i] - ref_y;
            anchor_tocar_x.push_back(shift_x *cos(-ref_yaw) - shift_y* sin(-ref_yaw));
            anchor_tocar_y.push_back(shift_x *sin(-ref_yaw) + shift_y* cos(-ref_yaw));
          }
         //std::cout<<"anchor size"<<anchor_x.size()<<std::endl;
         //std::cout<<"anchor to car x:"<<anchor_tocar_x[0]<<","<<anchor_tocar_x[1]<<","<<anchor_tocar_x[2]<<","<<anchor_tocar_x[3]<<","<<anchor_tocar_x[4]<<std::endl;
          //std::cout<<"anchor to car y:"<<anchor_tocar_y[0]<<","<<anchor_tocar_y[1]<<","<<anchor_tocar_y[2]<<","<<anchor_tocar_y[3]<<","<<anchor_tocar_y[4]<<std::endl;
         
          // spline fit the five anchor points
          tk::spline S;
          S.set_points(anchor_tocar_x,anchor_tocar_y);
          
          // use all the previous path points for the new path
          for (int i=0; i<prev_size; ++i){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          
          // generate a path in reference coordinate with spline line for the new points (50 - prev_size). the path has total 50 points
          // prev path points + new points
          double target_x = dist_ahead;
          double target_y = S(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          int num_pts = 50 - prev_size;  
          
          double new_x=0;
          double new_y;
          double N = (target_dist /(ref_vel*0.02/2.24));
          for (int i=1; i<=num_pts; ++i){
            new_x+= target_x/N;
            new_y = S(new_x);
              
            // convert reference coordinate to map coordinate
            double new_x_map = ref_x + cos(ref_yaw) * new_x - sin(ref_yaw) *new_y;  
            double new_y_map = ref_y + sin(ref_yaw) * new_x + cos(ref_yaw) *new_y;
            next_x_vals.push_back(new_x_map);
            next_y_vals.push_back(new_y_map);
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