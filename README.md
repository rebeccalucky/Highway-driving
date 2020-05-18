# Write up
  This project includes several parts: main.cpp, cost.h, helpers.h, spline.h, json.hpp.
  
## cost.h
  This file contains the functions that calculate the cost of all the potential path.
1. KL_Cost. Calculate the cost if we want to keep lane.
  If the minimum distance of front car is smaller than 15m, there is a risk of crash. So the cost is 1000. 
  If there is no car in the front, the cost is the smallest, 0.
  If there is car in the front and distance is bigger than 15m, should consider changing lane. Cost is 40
2. TL_Cost. Calculate the cost if we want to turn left.
  If the car is in the leftest lane, or left front/rear car distance is smaller than 15m. There is a risk of crash. Cost is 1000.
  If there is no car in the left, changing lane is good. But if there is no car in the front, still should keep lane. So the cost should be 1, a little bigger than 0.
  If there is no car in the front left but a car in the rear left, changing left lane should be better when there is a car in the front. So cost is 30.
  If there is car both in the front and rear, changing left is not good, but is better than crash. Cost is 50.
3. RL_Cost. Calculate the cost if we want to turn right.
  The same with TL_Cost.
4. PTL_Cost. Calculate the cost if we want to turn left for 2 lanes.
  If the car is in the rightest lane, and there is a car in the front and in the left front, but no car in the 2 lanes left, turn left to prepare for turn 2 lanes left is good. Cost is 0.
5. PRL_Cost. Calculate the cost if we want to turn right for 2 lanes.
  The same with PRL_Cost.

## main.cpp
1. Initialize.
  Initialize the car velocity to 0, car lane to 1 (Line 56,57) and car location to end of previous path point if there are previous path points (Line 105). Initialize distance of other cars and velocity to very big values (Line 118 to 131).
2. Prediction of other cars.
  For cars in the same lane, if in the front, distance is smaller than 30m and is smaller than minimum distance. Then update front car minimum distance and velocity. 
  For cars in the left/right lane, if in the front, distance is smaller than 30m and is smaller than minimum distance. Then update front car minimum distance and velocity. If in the rear, distance is smaller than 30m and is smaller than minimum distance. Then update rear car minimum distance and velocity.
3. Make decision which lane to choose.
  Use cost function to calculate all potential choices'costs, and choose the minimum cost path. According to the choice set target s and front cars velocity. If there is no front car in target lane, set speed difference to max acceloration. If there is front car in target lane and is faster than current car, set speed difference to 0. If front car is slower, set speed difference to minus max acceloration.
4. Generate trajectory.
  Use spline.h to fit the trajectory. Choose the last and last fifth points to be the first two. Choose 50,70 and 90m ahead of current car position to be the next three points. So that the trajectory can be smooth and jerk can be small. Firstly transform these points to local car coordinate, so there will be only one y value gotten with one x value.
5. Generate next path values.
  Use reserved previous path points as next path points and generate further points for the amount less than 50. target x is set 30m ahead end of previous path. Upgrade velocity and (x, y) in every step. (x, y) should be transformed back to world coordinate.
