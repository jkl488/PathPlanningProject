//
//  DataTypes.hpp
//  path_planning
//
//  Created by Jakob Klein on 23.05.21.
//
#ifndef DataTypes_hpp
#define DataTypes_hpp

#include <array>
#include <vector>
//Circle with radius and center coordinates
struct Circle
{
    double radius;
    double position_x;
    double position_y;
};

//Represents the vehicle for collision checks
struct BoundingCircles
{
    std::array<Circle,3> circles;
};

//Represents a global map consisting of waypoints
struct WaypointMap
{
    std::vector<double> map_waypoints_x;
    std::vector<double> map_waypoints_y;
    std::vector<double> map_waypoints_s;
    std::vector<double> map_waypoints_dx;
    std::vector<double> map_waypoints_dy;
};

//Represents the current state of the vehicle
struct VehicleState {
    double position_x;
    double position_y;
    double position_s;
    double position_d;
    double heading;
    double speed;
    BoundingCircles bounding_circles;
};
//Represents a object
struct Object{
    unsigned int id;
    double position_x_world;
    double position_y_world;
    double velocity_x_world;
    double velocity_y_world;
    double position_s;
    double position_d;
    BoundingCircles bounding_circles;
};

//Represents a list of all objects on the right side of the road
struct ObjectList{
    std::array<Object,12> objects;
};

//Represents the lists of points in x and y
struct GoalCoordianteList
{
    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;
};

//Represents the trajectory data given to the simulator
struct OutputPath
{
    GoalCoordianteList planned_coordinates_cartesian;
    double end_path_s;
    double end_path_d;
};


//Polynomial coefficients of a fifth order polynomial
struct FifthOrderPolynomial
{
    double a0;
    double a1;
    double a2;
    double a3;
    double a4;
    double a5;
};

//State consisting of position, velocity and acceleration
struct PlanningState
{
    double position;
    double position_dot;
    double position_dot_dot;
};

struct GoalPosition
{
    
};

#endif /* DataTypes_hpp */
