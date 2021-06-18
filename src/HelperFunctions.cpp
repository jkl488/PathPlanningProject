//
//  HelperFunctions.cpp
//  Path_Planning
//
//  Created by Jakob Klein on 24.05.21.
//  Moved content of helpers.h here
//

#include "HelperFunctions.hpp"



//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
double helpers::Deg2rad(double x)
{ return x * M_PI / 180; }
double helpers::Rad2deg(double x)
{ return x * 180 / M_PI; }

// Calculate distance between two points
double helpers::Distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
std::string helpers::HasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


