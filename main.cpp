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
using json = nlohmann::json;
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
  
  double ref_v = 0;    //Initialize the velocity and car lane.
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


          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          int prev_size = previous_path_x.size();
          
          if(prev_size > 0){
            car_s = end_path_s;
          }
          
          int path_size = 50;
          double step_time = 0.02;    
          double t = prev_size * step_time;  //Prediction time.
          double max_v = 49.5 * (1600.0 / 3600.0); //Set maximum velocity. Change mph to m/s
          double max_acc = 9;   //Set maximum acceloration.
          int num_lane = 3;    //Set total amount of lane.

          //predictions of other cars' position at the end of the path.
          double dist;
          double min_dist = 1000;  //Will be minimum distance between front car.
          double lf_min = 1000;   //Will be minimum distance between left front car.        
          double lb_min = -1000;  //Will be minimum distance between left rear car.
          double rf_min = 1000;  //Will be minimum distance between right front car.
          double rb_min = -1000;  //Will be minimum distance between right rear car.
          double front_v = 1000;  //Will be nearest front car's velocity.
          double left_fv = 1000;  //Will be nearest left front car's velocity.
          double right_fv = 1000;  //Will be nearest right front car's velocity.
          double lf2_min = 1000;  //Will be minimum distance between 2 lanes left front car.
          double left2_fv = 1000;  //Will be nearest 2 lanes left front car's velocity.
          double lb2_min = -1000;  //Will be minimum distance between 2 lanes left rear car.
          double rf2_min = 1000;  //Will be minimum distance between 2 lanes right front car.
          double right2_fv = 1000;  //Will be nearest 2 lanes right front car's velocity.
          double rb2_min = -1000;  //Will be minimum distance between 2 lanes right rear car's velocity.
  
          for(int i=0; i<sensor_fusion.size(); i++){
            double carvx = sensor_fusion[i][3];
            double carvy = sensor_fusion[i][4];
            double vel = sqrt(carvx * carvx + carvy * carvy);
            double cars = sensor_fusion[i][5];
            double card = sensor_fusion[i][6];
            double cars_lane = FindLane(card);
            double s = cars + vel * t;
            
            if(cars_lane == car_lane){
              dist = s - car_s;
              if(dist>0 && dist<30 && dist<min_dist){  //If distance is larger than 30m, consider there is no car.
                min_dist = dist;
                front_v = vel;
              }
            }else if(cars_lane == car_lane-1){
              dist = s - car_s;
              if(dist>0 && dist<30 && dist<lf_min){
                lf_min = dist;
                left_fv = vel;
              }else if(dist<0 && dist>-30 && dist>lb_min){
                lb_min = dist;
              }
            }else if(cars_lane == car_lane-2){
              dist = s - car_s;
              if(dist>0 && dist<30 && dist<lf2_min){
                lf2_min = dist;
                left2_fv = vel;
              }else if(dist<0 && dist>-30 && dist>lb2_min){
                lb2_min = dist;
              }
            }else if(cars_lane == car_lane+1){
              dist = s - car_s;
              if(dist>0 && dist<30 && dist<rf_min){
                rf_min = dist;
                right_fv = vel;
              }else if(dist<0 && dist>-30 && dist>rb_min){
                rb_min = dist;
              }
            }else if(cars_lane == car_lane+2){
              dist = s - car_s;
              if(dist>0 && dist<30 && dist<rf2_min){
                rf2_min = dist;
                right2_fv = vel;
              }else if(dist<0 && dist>-30 && dist>rb2_min){
                rb2_min = dist;
              }
            }
          }
          
          //Behavior planner.
          // KL is keep lane.
          // LT is left turn.
          // RT is right turn.
          //PTL is turn 2 lanes left.
          //PRL is turn 2 lanes right.
          
          //Calculate cost and choose the minimum.
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
          double ptl_cost = PTL_Cost(car_lane, min_dist, lf2_min, lb2_min, lf_min, lb_min);
          if(ptl_cost<min_cost){
            min_cost = ptl_cost;
            behv = "PTL";
          }
          double rtl_cost = PRL_Cost(car_lane, min_dist, rf2_min, rb2_min, rf_min, rb_min);
          if(rtl_cost<min_cost){
            min_cost = rtl_cost;
            behv = "RTL";
          }
          
          //Create trajectory according to the choice.
          double speed_diff;
          double target_d;
          double target_s;
          
          if(behv == "KL"){
            car_lane = car_lane;
            target_d = 2+car_lane*4;
            speed_diff = front_v - ref_v;
            target_s = min_dist;
          }else if(behv == "TL"){
            car_lane = car_lane - 1;
            target_d = 2+car_lane*4;
            speed_diff = left_fv - ref_v;
            target_s = lf_min;
          }else if(behv == "TR"){
            car_lane = car_lane + 1;
            target_d = 2+car_lane*4;
            speed_diff = right_fv - ref_v;
            target_s = rf_min;
          }else if(behv == "PTL"){
            car_lane = car_lane - 1;
            target_d = 2+car_lane*4;
            speed_diff = left2_fv - ref_v;
            target_s = lf_min;
          }else if(behv == "PRL"){
            car_lane = car_lane + 1;
            target_d = 2+car_lane*4;
            speed_diff = right2_fv - ref_v;
            target_s = rf_min;
          }
          
          if((target_s == 1000) && (speed_diff>0)){  
            speed_diff = max_acc * step_time;  //If there is no target lane front car. Accelorate.
          }else if(speed_diff<0){              
            speed_diff = (-max_acc * step_time);  //If front car velocity is smaller than me, Decelorate.
          }else{
            speed_diff = 0;  //If front car velocity is bigger. Keep velocity.
          }
 
          vector<double> sp_x;
          vector<double> sp_y;
                
          //Take 2 previous path points for spline.
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          //If there is not many previous points. Use current car point and the point before it.
          if(prev_size < 2){
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            sp_x.push_back(prev_car_x);
            sp_y.push_back(prev_car_y);
            sp_x.push_back(car_x);
            sp_y.push_back(car_y);
          }else{
            double prev_path_x = previous_path_x[prev_size - 5];
            double prev_path_y = previous_path_y[prev_size - 5];
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            ref_yaw = atan2((ref_y-prev_path_y), (ref_x-prev_path_x));
            sp_x.push_back(prev_path_x);
            sp_y.push_back(prev_path_y);
            sp_x.push_back(ref_x);
            sp_y.push_back(ref_y);          
          }
          
          //Create 3 more points for spline.
          vector<double> nextp0 = getXY(car_s + 50, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> nextp1 = getXY(car_s + 70, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> nextp2 = getXY(car_s + 90, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          sp_x.push_back(nextp0[0]);
          sp_x.push_back(nextp1[0]);
          sp_x.push_back(nextp2[0]);          

          sp_y.push_back(nextp0[1]);
          sp_y.push_back(nextp1[1]);
          sp_y.push_back(nextp2[1]);
          
          
          //Change sp points into local car coordinates.
          for(int i=0; i < sp_x.size(); i++){
            double delta_x = sp_x[i] - ref_x;
            double delta_y = sp_y[i] - ref_y;
            sp_x[i] = delta_x * cos(0 - ref_yaw) - delta_y * sin(0 - ref_yaw);
            sp_y[i] = delta_x * sin(0 - ref_yaw) + delta_y * cos(0 - ref_yaw);
          }
          
          //Initialize the spline.
          tk::spline s;
          s.set_points(sp_x, sp_y);
      
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          //Use reserved previous path points for next path.
          for(int i=0; i < prev_size; i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          //Calculate 30m ahead of the car position.
          double target_x = 30.;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);
          double x_add = 0; 
          
          for(int i=1; i < path_size - prev_size; i++){
            ref_v += speed_diff;
            if(ref_v>max_v){
              ref_v = max_v;
            }
            double N = target_dist / (ref_v * step_time);           
            double x = x_add + target_x / N;;
            double y = s(x);
            x_add = x;
            double ref_x0 = x;
            double ref_y0 = y;
            x = ref_x0 * cos(ref_yaw) - ref_y0 * sin(ref_yaw);
            y = ref_x0 * sin(ref_yaw) + ref_y0 * cos(ref_yaw);
            x += ref_x;
            y += ref_y;
            
            next_x_vals.push_back(x);
            next_y_vals.push_back(y);
          }

          json msgJson;
             
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
