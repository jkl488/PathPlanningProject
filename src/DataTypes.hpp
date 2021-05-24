//
//  DtaTypes.hpp
//  path_planning
//
//  Created by Jakob Klein on 23.05.21.
//

#include <array>
#include <vector>

//Represents the current state of the vehicle
struct VehicleState {
    double position_x;
    double position_y;
    double position_s;
    double position_d;
    double heading;
    double speed;
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



//Represents a trajectory in frenet coordinates
struct FrenetTrajectory
{
    
};


struct PolynomialTrajectory
{
    
};
