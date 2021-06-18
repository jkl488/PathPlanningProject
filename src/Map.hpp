//
//  Map.hpp
//  path_planning
//
//  Created by Jakob Klein on 18.06.21.
//

#ifndef Map_hpp
#define Map_hpp

#include <stdio.h>
#include "spline.h"
#include <vector>

class Map
{
public:
    //Default behavior for constructing/destructing
    Map() = default;
    ~Map() = default;
    //Default behavior for copy constructor and copying
    Map(const Map& rhs) = default;
    Map& operator= (const Map& rhs) = default;
    //Default behavior for move constructor and moving
    Map(Map&& rhs) = default;
    Map& operator= (Map&& rhs) = default;
    
    
    
    
    

    
    std::vector<double> GetFrenet(double x, double y, double theta);
    
    std::vector<double> GetCartesian(double s, double d);
    
    std::vector<double> GetCartesianSpline(double s, double d);
    
    double GetSpeedToFrenet(double speed, double s);
    
    void ReadFile(std::string map_file_name);
    
private:
    
    int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x,
                                 const std::vector<double> &maps_y);
    
    int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x,
                     const std::vector<double> &maps_y);
    
    
    std::vector<double> map_waypoints_x_;
    std::vector<double> map_waypoints_y_;
    std::vector<double> map_waypoints_s_;
    std::vector<double> map_waypoints_dx_;
    std::vector<double> map_waypoints_dy_;
    std::vector<double> map_waypoints_x_norm_;
    std::vector<double> map_waypoints_y_norm_;
    
    std::vector<double> map_s_;
    
    tk::spline x_spline_;
    tk::spline y_spline_;
    tk::spline dx_spline_;
    tk::spline dy_spline_;
    
    std::vector<double> detailed_map_waypoints_x_;
    std::vector<double> detailed_map_waypoints_y_;
    std::vector<double> detailed_map_waypoints_dx_;
    std::vector<double> detailed_map_waypoints_dy_;
    std::vector<double> detailed_map_s_;
    
    
};





#endif /* Map_hpp */
