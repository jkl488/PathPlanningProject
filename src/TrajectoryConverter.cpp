//
//  TrajectoryConverter.cpp
//  path_planning
//
//  Created by Jakob Klein on 01.06.21.
//

#include "TrajectoryConverter.hpp"
#include "config.hpp"




DiscretizedTrajectory TrajectoryConverter::DiscretizeQuinticPolynomialTrajectory(const QuinticPolynomialTrajectory& polynomial_trajectory)
{

    
}

DiscretizedTrajectory TrajectoryConverter::DiscretizeQuarticLongitudinalPolynomialTrajectory(const QuarticPolynomialTrajectory& polynomial_trajectory)
{
    DiscretizedTrajectory result_trajectory;
    
    timing passt nicht, zu wenig delta, außerdem wahrscheinlich polynom falsch wuartic
    
    //number of steps we follow the polynomial
    //int poly_steps = polynomial_trajectory.max_planning_time/CONFIGURATION::delta_t_trajectory_points;
    for(int i = 0; i < CONFIGURATION::num_trajectory_points; ++i)
    {
        TrajectoryPoint point;
        point.time = i*CONFIGURATION::delta_t_trajectory_points;
        point.position = polynomial_trajectory.GetPosition(point.time);
        point.position_dot = polynomial_trajectory.GetVelocity(point.time);
        point.position_dot_dot = polynomial_trajectory.GetAcceleration(point.time);
        point.position_dot_dot_dot = polynomial_trajectory.GetJerk(point.time);
        result_trajectory.points.push_back(point);
    }
    

    return result_trajectory;
}
