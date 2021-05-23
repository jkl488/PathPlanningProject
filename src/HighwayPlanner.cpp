//
//  HighwayPlanner.cpp
//  path_planning
//
//  Created by Jakob Klein on 23.05.21.
//

#include <iostream>

#include "HighwayPlanner.hpp"
#include <vector>

HighwayPlanner::HighwayPlanner()
{
    vehicle_state_ = std::make_shared<VehicleState>();
    object_list_ = std::make_shared<ObjectList>();
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

void HighwayPlanner::setObjectList()
{
    
}
