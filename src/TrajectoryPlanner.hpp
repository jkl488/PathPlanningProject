//
//  TrajectoryPlanner.hpp
//  path_planning
//
//  Created by Jakob Klein on 23.05.21.
//

#ifndef TrajectoryPlanner_hpp
#define TrajectoryPlanner_hpp

#include <stdio.h>
#include "DataTypes.hpp"

class TrajectoryPlanner
{
    
public:
    void init(std::shared_ptr<VehiclePose> vehicle_pose, std::shared_ptr<ObjectList> object_list , std::shared_ptr<WaypointMap> global_map,std::shared_ptr<OutputPath> output_path);
    void step();
    //Set the maneuver which the trajectories should be planned for
    void SetManeuverRequest(Maneuver requested_maneuver);
private:
    //Current pose of the vehicle in the world -INPUT
    std::shared_ptr<VehiclePose> vehicle_pose_ptr_ = nullptr;
    //List of the objects in the environment -INPUT
    std::shared_ptr<ObjectList> object_list_ptr_ = nullptr;
    //Path given to the simulator, -OUTPUT
    std::shared_ptr<OutputPath> output_path_ptr_ = nullptr;
    //Global mal waypoints. needed for the conversion of the frames
    std::shared_ptr<WaypointMap> global_map_ptr_ = nullptr;
    
    //Max time span we plan a quintic trajectoty currently
    double current_quintic_planning_time_ = 0.0;
    //time we are in the current planning mode
    int current_planning_timestep_ = -1;
    //Maneuver we should currently plan for
    Maneuver current_maneuver_;
    

    //For now we sty tightly to the papaer, to be able to play stupid
    //For velocity keeping or stopping
    std::vector<DiscretizedTrajectory> GenerateVelocityKeepingInLaneTrajectory(double goal_s_dot , double goal_d, double planning_time, double offset);
    //follow a vehicle with a constant time gap
    std::vector<DiscretizedTrajectory> GenerateFollowingTrajectory(PlanningState start_state, PlanningState final_state, double planning_time, double offset);
    //Position the vehicle optimal to perform lane change
    std::vector<DiscretizedTrajectory> GenerateMergeTrajectory(PlanningState start_state, PlanningState final_state, double planning_time, double offset);
    //Generate a trajectory for safe transfer between lanes
    std::vector<DiscretizedTrajectory> GenerateLaneChangeTrajectory();
    //Update the planning timestep
    void IncrementPlanningTimeStep();



    
    
};

#endif /* TrajectoryPlanner_hpp */
