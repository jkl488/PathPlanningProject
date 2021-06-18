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
    global_map_ptr_ = std::make_shared<Map>();
    
}


void HighwayPlanner::Init()
{
    if(vehicle_pose_ptr_ && object_list_ptr_ && output_path_ptr_ && global_map_ptr_&&previous_path_ptr_)
    {
        is_initialized_ = true;
        behavior_planner_.Init(vehicle_pose_ptr_, object_list_ptr_, global_map_ptr_);
        trajectory_planner_.Init(vehicle_pose_ptr_, object_list_ptr_, global_map_ptr_, output_path_ptr_,previous_path_ptr_);
        object_prediction_.Init(object_list_ptr_);
    }
    
}
void HighwayPlanner::Step()
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
    object_prediction_.PredictObjectTrajectories();
    //run the behavior state machine and generate requested maneuver
    behavior_planner_.Step();
    //transmit requested maneuver to trajectory planning
    trajectory_planner_.SetManeuverRequest(behavior_planner_.GetCurrentManeuver());
    //run trajectory planning to generate a trajectory for the current requested maneuver
    trajectory_planner_.Step();
    
    //pre calculate all time inverse matrices for quintic
    
    
    
    
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

void HighwayPlanner::SetMap(const Map& highway_map)
{
    global_map_ptr_ = std::make_shared<Map>(highway_map);
}

void HighwayPlanner::SetVehiclePose(double x, double y, double s, double d, double heading, double speed, int num_previous_path_left)
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
        //(current_s - previous_s)/time_travelled
        double s_dot = this->trajectory_planner_.GetLongitudinalTrajectory().points.at(steps_travelled-1).position_dot;
        //double s_dot = ((vehicle_pose_ptr_->position_s - vehicle_pose_ptr_->previous_position_s))/time_travelled;
        vehicle_pose_ptr_->previous_position_s = vehicle_pose_ptr_->position_s;
        //(current_s_dot - previous_s_dot)/time_travelled
        vehicle_pose_ptr_->position_s_dot_dot = (vehicle_pose_ptr_->position_s_dot - s_dot)/time_travelled;
        vehicle_pose_ptr_->position_s_dot = s_dot;
        //(current_d - previous_d)/time_travelled
        
        
        double d_dot = this->trajectory_planner_.GetLateralTrajectory().points.at(steps_travelled-1).position_dot;
        //double d_dot = (vehicle_pose_ptr_->position_d-vehicle_pose_ptr_->previous_position_d)/time_travelled;
        vehicle_pose_ptr_->previous_position_d = vehicle_pose_ptr_->position_d;
        //(current_d_dot - previous_d_dot)/time_travelled
        vehicle_pose_ptr_->position_d_dot_dot = (d_dot-vehicle_pose_ptr_->position_d_dot)/time_travelled;
        vehicle_pose_ptr_->position_d_dot = d_dot;
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

void HighwayPlanner::InsertObjects(std::vector<std::vector<double>> sensor_fusion)
{
    if(!object_list_ptr_)
    {
        std::cout << "ERROR: object_list_ is nullptr!" << '\n';
    }
    for(int i = 0; i < sensor_fusion.size(); ++i)
    {
        if(i<12)
        {
            unsigned int id = static_cast<unsigned int>(sensor_fusion[i][0]);
            double& pos_x = sensor_fusion[i][1];
            double& pos_y = sensor_fusion[i][2];
            double& vx = sensor_fusion[i][3];
            double& vy = sensor_fusion[i][4];
            double& s = sensor_fusion[i][5];
            double& d = sensor_fusion[i][6];
            
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
            object_list_ptr_->objects.at(i) = std::move(Object{id,pos_x,pos_y,vx,vy,s,d,lane_assignment,heading,bounding_circle});
        
            
        }
        else{std::cout << "ERROR: Objects array out of range..."<<'\n';}
    }
    
}


//void HighwayPlanner::setObjectToIndex(int index, unsigned int id, double pos_x, double pos_y, double vx, double vy, double s, double d)
//{
//    if(!object_list_ptr_)
//    {
//        std::cout << "ERROR: object_list_ is nullptr!" << '\n';
//    }
//
//}

//void HighwayPlanner::clearObjectList()
//{
//    if(!object_list_ptr_)
//    {
//        std::cout << "ERROR: object_list_ is nullptr!" << '\n';
//        return;
//    }
//    unsigned int id;
//    Vector2d empty{0.0,0.0};
//    Vector2d velocity;
//    double position_s;
//    double position_d;
//    unsigned int lane_assignment;
//    double heading;
//    Circle empty_circle{position_s,position_s,0.0};
//    
//    Object empty_object{255,empty,empty,0.0,0.0,255,0.0,empty_circle};
//    
//    for(Object& obj : object_list_ptr_->objects)
//    {
//        obj = empty_object;
//    }
//}


GoalCoordianteList HighwayPlanner::GetOutputPath()
{
    if(!output_path_ptr_)
    {
        std::cout << "ERROR: output_path_ is nullptr!" << '\n';
    }
    return output_path_ptr_->planned_coordinates_cartesian;
}

void HighwayPlanner::SetPreviousPath(OutputPath previous_path)
{
    previous_path_ptr_->planned_coordinates_cartesian.next_x_vals = std::move(previous_path.planned_coordinates_cartesian.next_x_vals);
    previous_path_ptr_->planned_coordinates_cartesian.next_y_vals = std::move(previous_path.planned_coordinates_cartesian.next_y_vals);
    previous_path_ptr_->end_path_d = std::move(previous_path.end_path_d);
    previous_path_ptr_->end_path_s = std::move(previous_path.end_path_s);

    
}
