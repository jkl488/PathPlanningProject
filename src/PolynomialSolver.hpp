//
//  PolynomialSolver.hpp
//  path_planning
//
//  Created by Jakob Klein on 01.06.21.
//

#ifndef PolynomialSolver_hpp
#define PolynomialSolver_hpp

#include <stdio.h>
#include "DataTypes.hpp"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

class PolynomialSolver
{
public:
    //Calculate the coefficients of a fifth order polynomial for a given start and end state
    static QuinticPolynomial CalculateQuinticPolynomialCoefficients(PlanningState start_state, PlanningState final_state, double planning_time);
    //Calculate the coefficients of a fourth order polynomial for a given start and end state
    static QuarticPolynomial CalculateQuarticPolynomialCoefficients(PlanningState start_state, PlanningState final_state, double planning_time);
    
private:
    
};

#endif /* PolynomialSolver_hpp */
