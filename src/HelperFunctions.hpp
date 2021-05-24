//
//  HelperFunctions.hpp
//  Path_Planning
//
//  Created by Jakob Klein on 24.05.21.
//

#ifndef HelperFunctions_hpp
#define HelperFunctions_hpp

#include <stdio.h>
#include <math.h>
#include <string>
#include <vector>

namespace helpers
{
std::string hasData(std::string s);
double deg2rad(double x);
double rad2deg(double x);
double distance(double x1, double y1, double x2, double y2);
int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x,
                             const std::vector<double> &maps_y);
int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x,
                          const std::vector<double> &maps_y);
std::vector<double> getFrenet(double x, double y, double theta,
                         const std::vector<double> &maps_x,
                                       const std::vector<double> &maps_y);
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s,
                     const std::vector<double> &maps_x,
                          const std::vector<double> &maps_y);


} //namespace helpers
#endif /* HelperFunctions_hpp */
