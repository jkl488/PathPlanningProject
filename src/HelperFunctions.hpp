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

double Deg2rad(double x);
double Rad2deg(double x);
double Distance(double x1, double y1, double x2, double y2);
std::string HasData(std::string s);




} //namespace helpers
#endif /* HelperFunctions_hpp */
