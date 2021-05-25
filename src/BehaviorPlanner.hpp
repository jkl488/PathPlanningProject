//
//  BehaviorPlanner.hpp
//  path_planning
//
//  Created by Jakob Klein on 23.05.21.
//

#ifndef BehaviorPlanner_hpp
#define BehaviorPlanner_hpp

#include <stdio.h>
#include "DataTypes.hpp"

class BehaviorPlanner
{
public:
    BehaviorPlanner();
    ~BehaviorPlanner();
    void init(std::shared_ptr<VehicleState> vehicle_state, std::shared_ptr<ObjectList> object_list, std::shared_ptr<WaypointMap> global_map);
    void step();
    
private:
    bool is_initialized_ = false;
    std::shared_ptr<VehicleState> vehicle_state_ = nullptr;
    std::shared_ptr<ObjectList> object_list_ = nullptr;
    std::shared_ptr<OutputPath> output_path_ = nullptr;
    std::shared_ptr<WaypointMap> global_map_ = nullptr;
    
    //Defining the states the state machine can be in
    enum Maneuver : unsigned int
    {
        ManeuverInit = 0U,
        ManeuverKeepLane = 1U,
        ManeuverPrepareLaneChangeLeft = 2U,
        ManeuverLaneChangeLeft = 3U,
        ManeuverPrepareLaneChangeRight = 4U,
        ManeuverLaneChangeRight = 5U
    };
    GoalPosition current_goal_;
    Maneuver current_state_;
    //counts how long we are in the current state
    unsigned int state_counter_;
    //flag to check if lane change to the right is allowed (in leftmost lane and if we aborted LC left)
    bool right_lane_change_allowed = false;
    
    void runStateMachine();
    
};

#endif /* BehaviorPlanner_hpp */
