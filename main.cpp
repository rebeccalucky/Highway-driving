#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "cost.h"
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
  
  double ref_v = 0.;
  double car_lane = 1.;

  h.onMessage([&ref_v, &car_lane,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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
          double lane_width = 4.;
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

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          if(previous_path_x.size()>0){
            car_s = end_path_s;
            car_d = end_path_d;
          }
          
          int path_size = 50;
          double step_time = 0.02;
          double t = previous_path_x.size() * step_time;
          double max_v = 49.5 * (1600.0 / 3600.0); //Change mph to m/s
          double max_acc = 9.5;
          int num_lane = 3;
          
          //predictions of other cars' position at the end of the path.
          double dist;
          double min_dist = 1000;
          double lf_min = 1000;
          double lb_min = -1000;
          double rf_min = 1000;
          double rb_min = -1000;
          double front_v = 1000;
          double left_fv = 1000;
          double left_bv = 1000;
          double right_fv = 1000;
          double right_bv = 1000;
  
          for(int i=0; i<sensor_fusion.size(); ++i){
            double carvx = sensor_fusion[i][3];
            double carvy = sensor_fusion[i][4];
            double vel = sqrt(carvx * carvx + carvy * carvy);
            double cars = sensor_fusion[i][5];
            double card = sensor_fusion[i][6];
            double cars_lane = FindLane(card, lane_width);
            double s = cars + vel * t;
            
            if(cars_lane == car_lane){
              dist = s - car_s;
              if(dist>0 && dist<90 && dist<min_dist){
                min_dist = dist;
                front_v = vel;
              }
            }else if(cars_lane == car_lane-1){
              dist = s - car_s;
              if(dist>0 && dist<90 && dist<lf_min){
                lf_min = dist;
                left_fv = vel;
              }else if(dist<0 && dist>-90 && dist>lb_min){
                lb_min = dist;
                left_bv = vel;
              }
            }else if(cars_lane == car_lane+1){
              dist = s - car_s;
              if(dist>0 && dist<90 && dist<rf_min){
                rf_min = dist;
                right_fv = vel;
              }else if(dist<0 && dist>-90 && dist>rb_min){
                rb_min = dist;
                right_bv = vel;
              }
            }           
          }
          
          /**Behavior planner.
           * KL is keep lane.
           * LT is left turn.
           * RT is right turn.
           */
          
          //Calculate cost and choose the minimum cost.
          double min_cost = 10000;
          string behv = "null";
          double kl_cost = KL_Cost(min_dist);
          if(kl_cost<min_cost){
            min_cost = kl_cost;
            behv = "KL";
          }
          double tl_cost = TL_Cost(car_lane, lf_min, lb_min);
          if(tl_cost<min_cost){
            min_cost = tl_cost;
            behv = "TL";
          }
          double tr_cost = TR_Cost(car_lane, rf_min, rb_min);
          if(tr_cost<min_cost){
            min_cost = tr_cost;
            behv = "TR";
          }

          //Create trajectory according to the choice.
          double speed_diff;
          double target_d;
          
          if(behv == "KL"){
            car_lane = car_lane;
            target_d = 2+car_lane*4;
            speed_diff = front_v - ref_v;
          }else if(behv == "TL"){
            car_lane = car_lane - 1;
            target_d = 2+car_lane*4;
            speed_diff = left_fv - ref_v;
          }else if(behv == "TR"){
            car_lane = car_lane + 1;
            target_d = 2+car_lane*4;
            speed_diff = right_fv - ref_v;
          }
          
          if(speed_diff>0){
            speed_diff = max_acc * step_time;
          }else if(speed_diff<0){
            speed_diff = (-max_acc * step_time);
          }
          
          vector<double> sp_x;
          vector<double> sp_y;
                
          //Take 2 previous path points for spline.
          
          double ref_x;
          double ref_y;
          double ref_yaw;
          
          if(previous_path_x.size()<2){
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            ref_x = car_x;
            ref_y = car_y;
            ref_yaw = deg2rad(car_yaw);
            sp_x.push_back(prev_car_x);
            sp_y.push_back(prev_car_y);
            sp_x.push_back(car_x);
            sp_y.push_back(car_y);
          }else{
            double prev_car_x = previous_path_x[previous_path_x.size()-2];
            double prev_car_y = previous_path_y[previous_path_x.size()-2];
            ref_x = previous_path_x[previous_path_x.size()-1];
            ref_y = previous_path_y[previous_path_x.size()-1];
            ref_yaw = atan2((ref_y-prev_car_y), (ref_x-prev_car_x));
            sp_x.push_back(prev_car_x);
            sp_y.push_back(prev_car_y);
            sp_x.push_back(ref_x);
            sp_y.push_back(ref_y);          
          }
          
          //Create 2 more points for spline.
          vector<double> nextp0 = getXY(car_s + 30, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> nextp1 = getXY(car_s + 60, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> nextp2 = getXY(car_s + 90, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          //Initialize the spline.
          sp_x.push_back(nextp0[0]);
          sp_x.push_back(nextp1[0]);
          sp_x.push_back(nextp2[0]);          

          sp_y.push_back(nextp0[1]);
          sp_y.push_back(nextp1[1]);
          sp_y.push_back(nextp2[1]);
          
          
          //Change sp points into local car coordinates.
          for(int i=0; i<sp_x.size(); ++i){
            sp_x[i] = (sp_x[i] - ref_x) * cos(0-ref_yaw) - (sp_y[i] - ref_y) * sin(0-ref_yaw);
            sp_y[i] = (sp_x[i] - ref_x) * sin(0-ref_yaw) + (sp_y[i] - ref_y) * cos(0-ref_yaw);
          }
          
          tk::spline s;
          s.set_points(sp_x, sp_y);
          
          for(int i=0; i<previous_path_x.size(); ++i){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          //Calculate 30m ahead of the car position.
          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);
          double x_add = 0; 
          
          for(int i=0; i<path_size-previous_path_x.size(); ++i){
            ref_v += speed_diff;
            if(ref_v>max_v){
              ref_v = max_v;
            }
            double N = target_dist / (ref_v * step_time);           
            double x = x_add + target_x / N;;
            double y = s(x);
            x_add = x;
            ref_x = x;
            ref_y = y;
            x = ref_x * cos(ref_yaw) - ref_y * sin(ref_yaw);
            y = ref_x * sin(ref_yaw) + ref_y * cos(ref_yaw);
            x += ref_x;
            y += ref_y;
            
            next_x_vals.push_back(x);
            next_y_vals.push_back(y);
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
