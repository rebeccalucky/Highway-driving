#include <iostream>
#include <string>
#include <vector>
#include "helpers.h"

using std::string;

double KL_Cost(double min_dist){
  double cost;
  if(min_dist<30){
    cost = 1000;   //Risk of crash, so cost is the biggest.
  }else if(min_dist == 100){
    cost = 0;     //No cars in the front.
  }else{
    cost = 40;
  }
  return cost;
}

double TL_Cost(double car_lane, double lf_min, double lb_min){
  double cost;
  if((car_lane == 0) || (lf_min<30) || (lb_min>-30)){
    cost = 1000;
  }else if((lf_min == 100) && (lb_min == 100)){
    cost = 1;
  }else if((lf_min == 100) && (lb_min<-30)){
    cost = 30;
  }else{
    cost = 50;
  }
  return cost;
}

double TR_Cost(double car_lane, double rf_min, double rb_min){
  double cost;
  if((car_lane == 2) || (rf_min<30) || (rb_min>-30)){
    cost = 1000;
  }else if((rf_min == 100) && (rb_min == 100)){
    cost = 2;
  }else if((rf_min == 100) && (rb_min<-30)){
    cost = 30;
  }else{
    cost = 50;
  }
  return cost;
}
