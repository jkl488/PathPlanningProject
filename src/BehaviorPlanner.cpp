//
//  BehaviorPlanner.cpp
//  path_planning
//
//  Created by Jakob Klein on 23.05.21.
//

#include "BehaviorPlanner.hpp"
#include <iostream>
BehaviorPlanner::BehaviorPlanner()
{}
BehaviorPlanner::~BehaviorPlanner()
{}

void BehaviorPlanner::init(std::shared_ptr<VehiclePose> vehicle_pose, std::shared_ptr<ObjectList> object_list , std::shared_ptr<WaypointMap> global_map)
{
    vehicle_pose_ptr_ = vehicle_pose;
    object_list_ptr_ = object_list;
    global_map_ptr_ = global_map;
    
    current_state_ = ManeuverKeepLane; //could also be init, but not yet needed
}

void BehaviorPlanner::step()
{
    runStateMachine();
    
}

void BehaviorPlanner::runStateMachine()
{
    
    
    switch (current_state_) {
        case ManeuverKeepLane:
            std::cout << "ManeuverKeepLane" << '\n';
            //if follow
            //if slower then our speed we would like do a lane change (preferably left (as long as we are not on th leftmost lane, else right)
            
            //if freedrive
            
            //if stop
            
            //POSSIBLE TRANSITION: ManeuverPrepareLaneChangeLeft, ManeuverLaneChangeRight
            break;
        case ManeuverPrepareLaneChangeLeft:
            std::cout << "ManeuverPrepareLaneChangeLeft" << '\n';
            //follow until LC possible
            
            //POSSIBLE TRANSITION: ManeuverLaneChangeLeft, ManeuverKeepLane
            break;
        case ManeuverPrepareLaneChangeRight:
            std::cout << "ManeuverPrepareLaneChangeRight" << '\n';
            //follow until LC possible
            
            //POSSIBLE TRANSITION: ManeuverLaneChangeLeft, ManeuverKeepLane
            break;
        case ManeuverLaneChangeLeft:
            std::cout << "ManeuverLaneChangeLeft" << '\n';
            //until we are fully in that lane
            
            //POSSIBLE TRANSITION: ManeuverKeepLane
            break;
        case ManeuverLaneChangeRight:
            std::cout << "ManeuverLaneChangeRight" << '\n';
            //until we are fully in that lane
            
            //POSSIBLE TRANSITION: ManeuverKeepLane
            break;
        
            
        default:
            break;
    }
}

void BehaviorPlanner::runManeuverKeepLane()
{
    //check if there are objects in out lane and get the closest in front of us
    //check if it is moving (TODO: or static->avoid or brake)
    //check if we already should follow
    //Not close enough->free drive
    //if we are not in lane 2(right) and we where enough steps in keep lane, prepare to go to right
    
    //Close enough, FOLLOW (TODO: technically we could only overtake left)
    //If speed is above our target speed, keep following
    //Else, prepare lane change, if we where enough steps in Keep lane
}
