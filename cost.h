#include <iostream>
#include <string>
#include <vector>
#include "helpers.h"

using std::string;

double KL_Cost(double min_dist){
  double cost;
  if(min_dist<15){
    cost = 1000;   //Risk of crash, so cost is the biggest.
  }else if(min_dist == 1000){
    cost = 0;     //No cars in the front.
  }else{
    cost = 40;
  }
  return cost;
}

double TL_Cost(double car_lane, double lf_min, double lb_min){
  double cost;
  if((car_lane == 0) || (lf_min<15) || (lb_min>-15)){
    cost = 1001;
  }else if((lf_min == 1000) && (lb_min == -1000)){
    cost = 1;
  }else if((lf_min == 1000) && ((lb_min<-15) && (lb_min>-30))){
    cost = 30;
  }else{
    cost = 50;
  }
  return cost;
}

double TR_Cost(double car_lane, double rf_min, double rb_min){
  double cost;
  if((car_lane == 2) || (rf_min<15) || (rb_min>-15)){
    cost = 1000;
  }else if((rf_min == 1000) && (rb_min == -1000)){
    cost = 1;
  }else if((rf_min == 1000) && ((rb_min<-15) && (rb_min>-30))){
    cost = 30;
  }else{
    cost = 50;
  }
  return cost;
}

double PTL_Cost(double car_lane, double min_dist, double lf2_min, double lb2_min, double lf_min, double lb_min){
  double cost;
  if((car_lane == 2) && (min_dist<30) && (lf_min>15) && (lf_min<30) && (lb_min<-15) && (lf2_min==1000) && (lb2_min<-15)){
    cost = 0;
  }else{
    cost = 1002;
  }
  return cost;
}

double PRL_Cost(double car_lane, double min_dist, double rf2_min, double rb2_min, double rf_min, double rb_min){
  double cost;
  if((car_lane == 0) && (min_dist<30) && (rf_min>15) && (rf_min<30) && (rb_min<-15) && (rf2_min==1000) && (rb2_min<-15)){
    cost = 0;
  }else{
    cost = 1002;
  }
  return cost;
}

