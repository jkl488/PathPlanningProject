//
//  HighwayPlanner.cpp
//  path_planning
//
//  Created by Jakob Klein on 23.05.21.
//

#include <iostream>

#include "HighwayPlanner.hpp"
#include <vector>
#include <cmath>

HighwayPlanner::HighwayPlanner()
{
    vehicle_state_ = std::make_shared<VehicleState>();
    object_list_ = std::make_shared<ObjectList>();
    output_path_ = std::make_shared<OutputPath>();
}
HighwayPlanner::~HighwayPlanner()
{}
void HighwayPlanner::init()
{
    
}
void HighwayPlanner::step()
{
    //road model
    //previous path
    
    //run pocessing if needed
    //run behavior planning
    //run trajectory planning
    /*for(int i = 0; i < object_list_->objects.size(); ++i)
    {
        std::cout << "ID: " << object_list_->objects.at(i).id << '\n';
        std::cout << "x: " << object_list_->objects.at(i).position_x_world << '\n';
        std::cout << "y: " << object_list_->objects.at(i).position_y_world << '\n';
        std::cout << "vx: " << object_list_->objects.at(i).velocity_x_world << '\n';
        std::cout << "vy: " << object_list_->objects.at(i).velocity_y_world << '\n';
        std::cout << "s: " << object_list_->objects.at(i).position_s << '\n';
        std::cout << "d: " << object_list_->objects.at(i).position_d << '\n';
    }*/
    
    
    double dist_inc = 0.05;
    output_path_->planned_coordinates_cartesian.next_x_vals.clear();
    output_path_->planned_coordinates_cartesian.next_y_vals.clear();
    for (int i = 0; i < 50; ++i) {
        double next_s = vehicle_state_->position_s+(i+1)*dist_inc;
        double next_d = 6;
        
        output_path_->planned_coordinates_cartesian.next_x_vals.push_back(vehicle_state_->position_x+(dist_inc*i)*cos(vehicle_state_->heading * M_PI / 180));
        output_path_->planned_coordinates_cartesian.next_y_vals.push_back(vehicle_state_->position_y+(dist_inc*i)*sin(vehicle_state_->heading* M_PI / 180));
    }
 
}

void HighwayPlanner::setVehicleState(double x, double y, double s, double d, double heading, double speed)
{
    vehicle_state_->position_x = x;
    vehicle_state_->position_y = y;
    vehicle_state_->position_s = s;
    vehicle_state_->position_d = d;
    vehicle_state_->heading = heading;
    vehicle_state_->speed = speed;
    std::cout << "Current position: x-> " << vehicle_state_->position_x << " y-> " << vehicle_state_->position_y << '\n';
    std::cout << "Current heading: " << vehicle_state_->heading << '\n';
    std::cout << "Current speed: " << vehicle_state_->speed << '\n';
    
}

void HighwayPlanner::setObjectToIndex(int index, unsigned int id, double pos_x, double pos_y, double vx, double vy, double s, double d)
{
    Object new_object{id,pos_x,pos_y,vx,vy,s,d};
    object_list_->objects.at(index) = new_object;
}

void HighwayPlanner::clearObjectList()
{
    Object empty_object{255,0.0,0.0,0.0,0.0,0.0,0.0};
    
    for(Object& obj : object_list_->objects)
    {
        obj = empty_object;
    }
}


GoalCoordianteList HighwayPlanner::GetOutputPath()
{
    return output_path_->planned_coordinates_cartesian;
}
