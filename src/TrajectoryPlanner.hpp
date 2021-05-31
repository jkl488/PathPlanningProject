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
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

class TrajectoryPlanner
{
    
public:
    void init(std::shared_ptr<VehiclePose> vehicle_pose, std::shared_ptr<ObjectList> object_list , std::shared_ptr<WaypointMap> global_map,std::shared_ptr<OutputPath> output_path);
    void step();
private:
    //Current pose of the vehicle in the world -INPUT
    std::shared_ptr<VehiclePose> vehicle_pose_ptr_ = nullptr;
    //List of the objects in the environment -INPUT
    std::shared_ptr<ObjectList> object_list_ptr_ = nullptr;
    //Path given to the simulator, -OUTPUT
    std::shared_ptr<OutputPath> output_path_ptr_ = nullptr;
    //Global mal waypoints. needed for the conversion of the frames
    std::shared_ptr<WaypointMap> global_map_ptr_ = nullptr;
    //Matrix to solve the linear equation system in the Polynomial calculation
    Eigen::MatrixXd quintic_time_matrix_;
    //Inverse of the Matrix to solve the linear equation system in the Polynomial calculation
    Eigen::MatrixXd quintic_time_matrix_inverse_;
    //Max time span we plan a quintic trajectoty currently
    double current_quintic_planning_time_ = 0.0;
    //Calculate the coefficients of a fifth order polynomial for a given start and end state
    QuinticPolynomial CalculateQuinticPolynomialCoefficients(PlanningState start_state, PlanningState final_state, double planning_time);
    //Set up the inverse of the time matrix to avoid some matrix invertations
    void SetQuinticTimeMatrix_inverse(double planning_time);
    //Calculate the coefficients of a fourth order polynomial for a given start and end state
    QuarticPolynomial CalculateQuarticPolynomialCoefficients(PlanningState start_state, PlanningState final_state, double planning_time);
    //For now we sty tightly to the papaer, to be able to play stupid
    //For velocity keeping or stopping
    std::vector<DiscretizedTrajectory> GenerateVelocityKeepingInLaneTrajectory(double goal_s_dot , double goal_d, double planning_time, double offset);
    //follow a vehicle with a constant time gap
    std::vector<DiscretizedTrajectory> GenerateFollowingTrajectory(PlanningState start_state, PlanningState final_state, double planning_time, double offset);

    std::vector<DiscretizedTrajectory> GenerateMergeTrajectory(PlanningState start_state, PlanningState final_state, double planning_time, double offset);


    
    
};

#endif /* TrajectoryPlanner_hpp */
