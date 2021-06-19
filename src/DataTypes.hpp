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
#include "config.hpp"

//Vector in 2 dimensional space
struct Vector2d
{
    double x;
    double y;
};

//Circle with radius and center coordinates
struct Circle
{
    double postition_s;
    double position_d;
    double radius;
};

enum class Lane : unsigned int
{
    leftmost = 0,
    middle = 1,
    right = 2,
    unknown = 255
};

typedef Lane Lane;

//Represents the current state of the vehicle
struct VehiclePose {
    Vector2d position;
    double position_s{0.0};
    double previous_position_s{0.0};
    double position_s_dot{0.0};
    double position_s_dot_dot{0.0};
    double position_d{0.0};
    double previous_position_d{0.0};
    double position_d_dot{0.0};
    double position_d_dot_dot{0.0};
    double heading{0.0};
    double speed{0.0};
    Lane lane_assignment{Lane::unknown}; //0 leftmost, 1 middle, 2 right
    Circle bounding_circle;
};
//Represents a object
struct Object{
    unsigned int id;
    Vector2d position;
    Vector2d velocity;
    double position_s;
    double position_d;
    Lane lane_assignment{Lane::unknown};
    double heading;
    Circle bounding_circle;
    std::array<double,CONFIGURATION::num_trajectory_points> predicted_longitudinal_position; //assume it holds its position in the lane
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

enum class TrajectoryType : unsigned int
{
    VelocityKeeping = 0,
    VehicleFollowing = 1
    
//    Follow
//    LaneChange
//    Merge
    
};

typedef enum TrajectoryType TrajectoryType;

//Polynomial coefficients of a fifth order polynomial
struct QuinticPolynomial
{
    double a0;
    double a1;
    double a2;
    double a3;
    double a4;
    double a5;
};

struct QuinticPolynomialTrajectory
{
    QuinticPolynomial polynomial;
    double max_planning_time;
    TrajectoryType trajectory_type;
    
    double GetPosition(double time) const
    {
        return polynomial.a0 + polynomial.a1 * time + polynomial.a2 * time * time + polynomial.a3 * time * time * time + polynomial.a4 * time * time * time * time + polynomial.a5 * time * time * time * time * time;
    }
    double GetVelocity(double time) const
    {
        return polynomial.a1 + 2.0 *  polynomial.a2 * time +  3.0 * polynomial.a3 * time * time + 4.0 *  polynomial.a4 * time * time * time + 5.0 *  polynomial.a5 * time * time * time * time;
    }
    double GetAcceleration(double time) const
    {
        return 2.0 * polynomial.a2 + 6.0 * polynomial.a3 * time + 12.0 * polynomial.a4 * time * time + 20.0 * polynomial.a5 * time * time * time;
    }
    double GetJerk(double time) const
    {
        return 6.0 * polynomial.a3 + 24.0 * polynomial.a4 * time + 60.0 * polynomial.a5 * time * time;
    }
    
};

//Polynomial coefficients of a fourth order polynomial
struct QuarticPolynomial
{
    double a0;
    double a1;
    double a2;
    double a3;
    double a4;
};

struct QuarticPolynomialTrajectory
{
    QuarticPolynomial polynomial;
    double max_planning_time;
    TrajectoryType trajectory_type;
    
    double GetPosition(double time) const
    {
        return polynomial.a0 + polynomial.a1 * time + polynomial.a2 * time * time + polynomial.a3 * time * time * time + polynomial.a4 * time * time * time * time;
    }
    double GetVelocity(double time) const
    {
        return polynomial.a1 + 2.0 * polynomial.a2 * time + 3.0 * polynomial.a3 * time * time + 4.0 * polynomial.a4 * time * time * time;
    }
    double GetAcceleration(double time) const
    {
        return 2.0 * polynomial.a2 + 6.0 * polynomial.a3 * time + 12.0 * polynomial.a4 * time * time;
    }
    double GetJerk(double time) const
    {
        return 6.0 * polynomial.a3 + 24.0 * polynomial.a4 * time;
    }
    
};

//State consisting of position, velocity, acceleration and goal planning time
struct PlanningState
{
    double position;
    double position_dot;
    double position_dot_dot;
};

struct GoalState
{
    PlanningState lateral;
    PlanningState longitudinal;
};

struct TrajectoryPoint
{
    //Vector2d position;
    double position{0.0};
    double position_dot{0.0};
    double position_dot_dot{0.0};
    double position_dot_dot_dot{0.0};
    double time{0.0};
    double cost;
};

struct DiscretizedTrajectory
{
    std::vector<TrajectoryPoint> points;
    TrajectoryType trajectory_type;
    bool exceeds_jerk_limit = false;
    bool exceeds_acceleration_limit = false;
    double score{0.0};
};

struct ObjectPath
{
    std::array<Vector2d,CONFIGURATION::num_trajectory_points> predicted_positions;
};

//Defining the states the state machine can be in
enum class Maneuver : unsigned int
{
    ManeuverInit = 0U,
    ManeuverKeepLane = 1U,
    ManeuverPrepareLaneChangeLeft = 2U,
    ManeuverLaneChangeLeft = 3U,
    ManeuverPrepareLaneChangeRight = 4U,
    ManeuverLaneChangeRight = 5U
};

typedef enum Maneuver Maneuver;


#endif /* DataTypes_hpp */
