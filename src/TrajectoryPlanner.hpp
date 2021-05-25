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
    void init();
    void step();
private:
    Eigen::MatrixXd time_matrix_;
    Eigen::MatrixXd time_matrix_inverse_;
    int current_planning_time_ = 0;
    void GenerateLateralTrajectory();
    FifthOrderPolynomial CalculateFithOrderPolynomialCoefficients(PlanningState start_state, PlanningState final_state, int planning_time);
    void SetTimeMatrix_inverse(int planning_time);
    
    
};

#endif /* TrajectoryPlanner_hpp */
