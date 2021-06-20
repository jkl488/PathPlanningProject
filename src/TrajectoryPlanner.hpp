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
#include "Map.hpp"

class TrajectoryPlanner
{
    
public:
    TrajectoryPlanner() = default;
    ~TrajectoryPlanner() = default;
    TrajectoryPlanner(const TrajectoryPlanner& rhs) = default;
    TrajectoryPlanner& operator= (const TrajectoryPlanner& rhs) = default;
    TrajectoryPlanner(TrajectoryPlanner&& rhs) = default;
    TrajectoryPlanner& operator= (TrajectoryPlanner&& rhs) = default;
    
    void Init(std::shared_ptr<VehiclePose> vehicle_pose, std::shared_ptr<ObjectList> object_list , std::shared_ptr<Map> global_map,std::shared_ptr<OutputPath> output_path, std::shared_ptr<OutputPath> previous_path);
    void Step();
    //Set the maneuver which the trajectories should be planned for
    void SetManeuverRequest(TrajectoryType requested_trajectory, GoalState requested_state);
    DiscretizedTrajectory GetLongitudinalTrajectory();
    DiscretizedTrajectory GetLateralTrajectory();
    
    
private:
    //Current pose of the vehicle in the world -INPUT
    std::shared_ptr<VehiclePose> vehicle_pose_ptr_ = nullptr;
    //List of the objects in the environment -INPUT
    std::shared_ptr<ObjectList> object_list_ptr_ = nullptr;
    //Path given to the simulator, -OUTPUT
    std::shared_ptr<OutputPath> output_path_ptr_ = nullptr;
    //Path received to the simulator, with unused points -INPUT
    std::shared_ptr<OutputPath> previous_path_ptr_ = nullptr;
    //Global mal waypoints. needed for the conversion of the frames
    std::shared_ptr<Map> global_map_ptr_ = nullptr;
    
    DiscretizedTrajectory trajectory_longitudinal_;
    DiscretizedTrajectory trajectory_lateral_;
    
    //Max time span we plan a quintic trajectoty currently
//    double current_quintic_planning_time_ = 0.0;
    //time we are in the current planning mode
    int current_planning_timestep_ = -1;
    //Trajectory we should currently generate
    TrajectoryType current_trajectory_type_;
    //Current ideal goal we want to reach
    GoalState current_goal_state_;
    

    //For now we sty tightly to the papaer, to be able to play stupid
    //For velocity keeping or stopping
    void GenerateVelocityKeepingInLaneTrajectory(double goal_s_dot , double goal_d, double planning_time, double offset);
    //follow a vehicle with a constant time gap
    void GenerateVehicleFollowingTrajectory(double goal_s,double goal_s_dot,double goal_s_dot_dot, double goal_d, double planning_time, double offset);
    //Position the vehicle optimal to perform lane change
    void GenerateMergeTrajectory(PlanningState start_state, PlanningState final_state, double planning_time, double offset);
    //Generate a trajectory for safe transfer between lanes
    void GenerateLaneChangeTrajectory();
    //Update the planning timestep
    void IncrementPlanningTimeStep();
    //Returns the trajectory with the lowest score
    DiscretizedTrajectory SelectTrajectory(std::vector<DiscretizedTrajectory>& trajectory_candidates);



    
    
};

#endif /* TrajectoryPlanner_hpp */
