//
//  HighwayPlanner.cpp
//  path_planning
//
//  Created by Jakob Klein on 23.05.21.
//

#include <iostream>
#include <iomanip>
#include "HighwayPlanner.hpp"
#include <vector>
#include <cmath>
#include "HelperFunctions.hpp"

//#include "spline.h"

HighwayPlanner::HighwayPlanner()
{
    vehicle_pose_ptr_ = std::make_shared<VehiclePose>();
    object_list_ptr_ = std::make_shared<ObjectList>();
    output_path_ptr_ = std::make_shared<OutputPath>();
    previous_path_ptr_ = std::make_shared<OutputPath>();
    global_map_ptr_ = std::make_shared<WaypointMap>();
    
}

HighwayPlanner::~HighwayPlanner()
{}

void HighwayPlanner::init()
{
    if(vehicle_pose_ptr_ && object_list_ptr_ && output_path_ptr_ && global_map_ptr_&&previous_path_ptr_)
    {
        is_initialized_ = true;
        behavior_planner_.init(vehicle_pose_ptr_, object_list_ptr_, global_map_ptr_);
        trajectory_planner_.init(vehicle_pose_ptr_, object_list_ptr_, global_map_ptr_, output_path_ptr_,previous_path_ptr_);
        object_prediction_.init(object_list_ptr_);
    }
    
}
void HighwayPlanner::step()
{
    if(!is_initialized_)
    {
        std::cout << "ERROR: Highway planner is not properly initialized..." << '\n';
        return;
    }
    if(cycle_counter_== (std::numeric_limits<long long>::max() - 1))
    {
        cycle_counter_ = 0;
    }
    cycle_counter_++;
    //Predict objects movements
    object_prediction_.predictObjectTrajectories();
    //run the behavior state machine and generate requested maneuver
    behavior_planner_.step();
    //transmit requested maneuver to trajectory planning
    trajectory_planner_.SetManeuverRequest(behavior_planner_.GetCurrentManeuver());
    //run trajectory planning to generate a trajectory for the current requested maneuver
    trajectory_planner_.step();
    
    
//     1. try to understand the communication and its signals
//
//     2. implement a longitudinal planner and try to drive safely in one lane, without hitting the preceding vehicles and travelling with the fastest admissible speed.
//
//     3. Implement the lane change trajectory with a very basic lane change decision making
//
//     4. Improve your lane change decision making by setting up a cost function
//
//     5. refine the cost function
}

void HighwayPlanner::setMap(std::vector<double> waypoints_x, std::vector<double> waypoints_y, std::vector<double> waypoints_s, std::vector<double> waypoints_dx, std::vector<double> waypoints_dy)
{
    global_map_ptr_ = std::make_shared<WaypointMap>(std::move(WaypointMap{waypoints_x,waypoints_y,waypoints_s,waypoints_dx,waypoints_dy}));
}

void HighwayPlanner::setVehiclePose(double x, double y, double s, double d, double heading, double speed, int num_previous_path_left)
{
    if(!vehicle_pose_ptr_)
    {
        std::cout << "ERROR: vehicle_pose_ is nullptr!" << '\n';
    }
    //cartesian position
    vehicle_pose_ptr_->position.x = x;
    vehicle_pose_ptr_->position.y = y;
    //frenet position
    vehicle_pose_ptr_->position_s = s;
    vehicle_pose_ptr_->position_d = d;
    //heading and speed
    vehicle_pose_ptr_->heading = heading;
    vehicle_pose_ptr_->speed = speed;
    
    int steps_travelled = CONFIGURATION::num_trajectory_points - num_previous_path_left;
    double time_travelled = (num_previous_path_left==0) ? 0.0 : steps_travelled * CONFIGURATION::cycle_time_simulator;
    if(cycle_counter_ == 0)
    {
        vehicle_pose_ptr_->previous_position_s = vehicle_pose_ptr_->position_s;
        vehicle_pose_ptr_->position_s_dot_dot = 0.0;
        vehicle_pose_ptr_->position_s_dot = 0.0;
        vehicle_pose_ptr_->previous_position_d = vehicle_pose_ptr_->position_d;
        vehicle_pose_ptr_->position_d_dot_dot = 0.0;
        vehicle_pose_ptr_->position_d_dot = 0.0;
    }
    else
    {
        double s_diff = (vehicle_pose_ptr_->position_s-vehicle_pose_ptr_->previous_position_s);
        vehicle_pose_ptr_->previous_position_s = vehicle_pose_ptr_->position_s;
        std::cout << "s_diff: " << s_diff << std::endl;
        
        
        vehicle_pose_ptr_->position_s_dot = s_diff/time_travelled;
        vehicle_pose_ptr_->position_s_dot_dot = (vehicle_pose_ptr_->position_s_dot - vehicle_pose_ptr_->position_s_dot)/time_travelled;
        
        vehicle_pose_ptr_->position_d_dot = (vehicle_pose_ptr_->position_d-vehicle_pose_ptr_->previous_position_d)/time_travelled;
        vehicle_pose_ptr_->previous_position_d = vehicle_pose_ptr_->position_d;
        vehicle_pose_ptr_->position_d_dot_dot = (vehicle_pose_ptr_->position_d_dot-vehicle_pose_ptr_->position_d_dot)/time_travelled;
    }

    //Assign lane
    if(vehicle_pose_ptr_->position_d > CONFIGURATION::left_lane_lower_limit && vehicle_pose_ptr_->position_d <= CONFIGURATION::left_lane_upper_limit)
    {vehicle_pose_ptr_->lane_assignment = 0;}
    else if(vehicle_pose_ptr_->position_d > CONFIGURATION::middle_lane_lower_limit && vehicle_pose_ptr_->position_d <= CONFIGURATION::middle_lane_upper_limit)
    {vehicle_pose_ptr_->lane_assignment = 1;}
    else if(vehicle_pose_ptr_->position_d > CONFIGURATION::right_lane_lower_limit && vehicle_pose_ptr_->position_d <= CONFIGURATION::right_lane_upper_limit)
    {vehicle_pose_ptr_->lane_assignment = 2;}
    
    std::cout << std::fixed;
    std::cout << std::setprecision(6);
    std::cout << "setVehiclePose: Current position: x-> " << vehicle_pose_ptr_->position.x << " y-> " << vehicle_pose_ptr_->position.y << '\n';
    std::cout << "setVehiclePose: Current heading: " << vehicle_pose_ptr_->heading << '\n';
    std::cout << "setVehiclePose: Current speed: " << vehicle_pose_ptr_->speed << '\n';
    std::cout << "setVehiclePose: Current lane: " << vehicle_pose_ptr_->lane_assignment << '\n';
    std::cout << "setVehiclePose: Current s_dot: " << vehicle_pose_ptr_->position_s_dot << '\n';
    std::cout << "setVehiclePose: Current s_dot_Dot: " << vehicle_pose_ptr_->position_s_dot_dot << '\n';
}

void HighwayPlanner::setObjectToIndex(int index, unsigned int id, double pos_x, double pos_y, double vx, double vy, double s, double d)
{
    if(!object_list_ptr_)
    {
        std::cout << "ERROR: object_list_ is nullptr!" << '\n';
    }
    //assign lane
    unsigned int lane_assignment;
    if(d > CONFIGURATION::left_lane_lower_limit && d <= CONFIGURATION::left_lane_upper_limit)
    {lane_assignment = 0;}
    else if(d > CONFIGURATION::middle_lane_lower_limit && d <= CONFIGURATION::middle_lane_upper_limit)
    {lane_assignment = 1;}
    else if(d > CONFIGURATION::right_lane_lower_limit && d <= CONFIGURATION::right_lane_upper_limit)
    {lane_assignment = 2;}
    
    //Create bounding circle
    Circle bounding_circle{s,d,CONFIGURATION::object_bounding_circle_radius};

    //get heading
    double heading = std::atan2(vy, vx);//TODO: verify
    object_list_ptr_->objects.at(index) = std::move(Object{id,pos_x,pos_y,vx,vy,s,d,lane_assignment,heading,bounding_circle});
}

void HighwayPlanner::clearObjectList()
{
    if(!object_list_ptr_)
    {
        std::cout << "ERROR: object_list_ is nullptr!" << '\n';
        return;
    }
    unsigned int id;
    Vector2d empty{0.0,0.0};
    Vector2d velocity;
    double position_s;
    double position_d;
    unsigned int lane_assignment;
    double heading;
    Circle empty_circle{position_s,position_s,0.0};
    
    Object empty_object{255,empty,empty,0.0,0.0,255,0.0,empty_circle};
    
    for(Object& obj : object_list_ptr_->objects)
    {
        obj = empty_object;
    }
}


GoalCoordianteList HighwayPlanner::getOutputPath()
{
    if(!output_path_ptr_)
    {
        std::cout << "ERROR: output_path_ is nullptr!" << '\n';
    }
    return output_path_ptr_->planned_coordinates_cartesian;
}

void HighwayPlanner::setPreviousPath(OutputPath previous_path)
{
    previous_path_ptr_->planned_coordinates_cartesian.next_x_vals = std::move(previous_path.planned_coordinates_cartesian.next_x_vals);
    previous_path_ptr_->planned_coordinates_cartesian.next_y_vals = std::move(previous_path.planned_coordinates_cartesian.next_y_vals);
    previous_path_ptr_->end_path_d = std::move(previous_path.end_path_d);
    previous_path_ptr_->end_path_s = std::move(previous_path.end_path_s);

    
}
