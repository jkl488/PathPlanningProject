//
//  Map.cpp
//  path_planning
//
//  Created by Jakob Klein on 18.06.21.
//

#include "Map.hpp"
#include <vector>
#include "HelperFunctions.hpp"
#include <iostream>
#include <sstream>
#include <fstream>
#include "config.hpp"

void Map::ReadFile(std::string map_file_name)
{
    std::ifstream in_map_(map_file_name.c_str(), std::ifstream::in);
    std::string line;
    bool first_iter = true;
    double first_x, first_y, first_dx, first_dy;
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
      map_waypoints_x_.push_back(x);
      map_waypoints_y_.push_back(y);
      map_waypoints_s_.push_back(s);
//      last_s = s;
      map_waypoints_dx_.push_back(d_x);
      map_waypoints_dy_.push_back(d_y);
        if(first_iter)
        {
            first_x = x;
            first_y = y;
            first_dx = d_x;
            first_dy = d_y;
            first_iter = false;
        }
    }
    //temporary, for better spline between last and first
    map_waypoints_x_.push_back(first_x);
    map_waypoints_y_.push_back(first_y);
    map_waypoints_s_.push_back(CONFIGURATION::max_map_s);//(MAX_S);
    map_waypoints_dx_.push_back(first_dx);
    map_waypoints_dy_.push_back(first_dy);
    map_waypoints_x_norm_.push_back(first_x + 10.0 * first_dx);
    map_waypoints_y_norm_.push_back(first_y + 10.0 * first_dy);
    
    x_spline_.set_points(map_waypoints_s_, map_waypoints_x_);
    y_spline_.set_points(map_waypoints_s_, map_waypoints_y_);
    dx_spline_.set_points(map_waypoints_s_, map_waypoints_dx_);
    dy_spline_.set_points(map_waypoints_s_, map_waypoints_dy_);
    
    //remove temp points
    map_waypoints_x_.pop_back();
    map_waypoints_y_.pop_back();
    map_waypoints_s_.pop_back();
    map_waypoints_dx_.pop_back();
    map_waypoints_dy_.pop_back();
    map_waypoints_x_norm_.pop_back();
    map_waypoints_y_norm_.pop_back();
    
    double reference_length = 0.0;
      double previous_x = x_spline_(0.0);
      double previous_y = y_spline_(0.0);
      for (int s = 1; s <= floor(CONFIGURATION::max_map_s); ++s) {
        double x = x_spline_(static_cast<double>(s));
        double y = y_spline_(static_cast<double>(s));
        reference_length += helpers::Distance(x, y, previous_x, previous_y);
        previous_x = x;
        previous_y = y;
      }
    
    //Create finer waypoint map
    for(int s = 0; s <= floor(CONFIGURATION::max_map_s); ++s )
    {
        detailed_map_waypoints_x_.push_back(x_spline_(static_cast<double>(s)));
        detailed_map_waypoints_y_.push_back(y_spline_(static_cast<double>(s)));
        detailed_map_waypoints_dx_.push_back(dx_spline_(static_cast<double>(s)));
        detailed_map_waypoints_dy_.push_back(dy_spline_(static_cast<double>(s)));
    }
    

    double f_s{0.0};
    map_s_.push_back(f_s);
    detailed_map_s_.push_back(f_s);
    for(int i = 1; i < map_waypoints_x_.size(); ++i)
    {
        f_s += helpers::Distance(map_waypoints_x_.at(i), map_waypoints_y_.at(i), map_waypoints_x_.at(i-1), map_waypoints_y_.at(i-1));
        map_s_.push_back(f_s);
        
        detailed_map_s_.push_back(i);
    }
    
}

// Calculate closest waypoint to current x, y position
int Map::ClosestWaypoint(double x, double y, const std::vector<double> &maps_x,
                    const std::vector<double> &maps_y) {
  double closestLen = 100000.0; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = helpers::Distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int Map::NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x,
                 const std::vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min( 2.0 * M_PI - angle, angle);

  if (angle > M_PI/4) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> Map::GetFrenet(double x, double y, double theta) {
    
    std::vector<double> &maps_s = detailed_map_s_; ;
    std::vector<double> &maps_x = detailed_map_waypoints_x_;
    std::vector<double> &maps_y = detailed_map_waypoints_y_;
    
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = helpers::Distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = CONFIGURATION::x_center - maps_x[prev_wp];
  double center_y = CONFIGURATION::y_center - maps_y[prev_wp];
  double centerToPos = helpers::Distance(center_x,center_y,x_x,x_y);
  double centerToRef = helpers::Distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0.0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += helpers::Distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += helpers::Distance(0.0,0.0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> Map::GetCartesian(double s, double d) {
    
  std::vector<double> &maps_s = map_s_;
  std::vector<double> &maps_x = map_waypoints_x_;
  std::vector<double> &maps_y = map_waypoints_y_;
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-M_PI/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

std::vector<double> Map::GetCartesianSpline(double s, double d) {
  s = fmod(s, CONFIGURATION::max_map_s); // bug fix for JMT wraparound
    double x = x_spline_(s) + d * dx_spline_(s);
    double y = y_spline_(s) + d * dy_spline_(s);

    return {x,y};
}

double Map::GetSpeedToFrenet(double Vxy, double s) {
  s = fmod(s, CONFIGURATION::max_map_s);
  double dx_over_ds = x_spline_.deriv(1, s);
  double dy_over_ds = y_spline_.deriv(1, s);
  double Vs = (Vxy / sqrt(dx_over_ds * dx_over_ds + dy_over_ds * dy_over_ds));
  return Vs;
}
