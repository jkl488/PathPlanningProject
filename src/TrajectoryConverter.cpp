//
//  TrajectoryConverter.cpp
//  path_planning
//
//  Created by Jakob Klein on 01.06.21.
//

#include "TrajectoryConverter.hpp"
#include "config.hpp"
#include <iostream>




DiscretizedTrajectory TrajectoryConverter::DiscretizeQuinticPolynomialTrajectory(const QuinticPolynomialTrajectory& polynomial_trajectory)
{

    
}


//DiscretizedTrajectory TrajectoryConverter::DiscretizeQuarticLongitudinalPolynomialTrajectory(const QuarticPolynomialTrajectory& polynomial_trajectory)
//{
//    DiscretizedTrajectory result_trajectory;
//    result_trajectory.trajectory_type = polynomial_trajectory.trajectory_type;
//    int num_poly_step = polynomial_trajectory.max_planning_time/CONFIGURATION::delta_t_trajectory_points;
//    bool goal_reached = false;
//    for(int i = 0; i <= CONFIGURATION::num_trajectory_points; ++i)
//    {
//        TrajectoryPoint point;
//        point.time = i*CONFIGURATION::delta_t_trajectory_points;
//        //Use polynomial until we reached the goal velocity
//        double velo;
//        if(!goal_reached)
//        {
//            velo = polynomial_trajectory.GetVelocity(point.time);
//        }
//        else
//        {
//            velo = CONFIGURATION::speed_limit_m_s;
//        }
//
//
//        //TODO: CHANGE THAT TO TRAJECTORY GOAL
//
//        if(std::abs(velo - CONFIGURATION::speed_limit_m_s)<0.00001)
//        {
//            std::cout << "lin"<<"\n";
//            point.position = polynomial_trajectory.GetPosition(0.0) + ( i * CONFIGURATION::delta_t_trajectory_points) * CONFIGURATION::speed_limit_m_s;
//            point.position_dot = velo;
//            point.position_dot_dot = 0.0;
//            goal_reached = true;
//        }
//        else
//        {
//            std::cout << "poly"<<"\n";
//            point.position = polynomial_trajectory.GetPosition(point.time);
//            point.position_dot = velo;
//            point.position_dot_dot = polynomial_trajectory.GetAcceleration(point.time);
//        }
//        if(point.position_dot_dot >= CONFIGURATION::maximum_acceleration)
//        {
//            result_trajectory.exceeds_acceleration_limit = true;
//        }
//        point.position_dot_dot_dot = polynomial_trajectory.GetJerk(point.time);
//        if(point.position_dot_dot_dot >= CONFIGURATION::max_jerk)
//        {
//            result_trajectory.exceeds_jerk_limit = true;
//        }
//        std::cout << "number: " << i << ",position: " << point.position << ",position_dot: " << point.position_dot << ",position_dot_dot: " << point.position_dot_dot <<std::endl;
//        result_trajectory.points.push_back(point);
//    }
//    return result_trajectory;
//}


DiscretizedTrajectory TrajectoryConverter::DiscretizeQuarticLongitudinalPolynomialTrajectory(const QuarticPolynomialTrajectory& polynomial_trajectory)
{
    DiscretizedTrajectory result_trajectory;
    result_trajectory.trajectory_type = polynomial_trajectory.trajectory_type;
    int num_poly_step = polynomial_trajectory.max_planning_time/CONFIGURATION::delta_t_trajectory_points;
    for(int i = 0; i <= num_poly_step; ++i)
    {
        TrajectoryPoint point;
        point.time = i*CONFIGURATION::delta_t_trajectory_points;
        point.position = polynomial_trajectory.GetPosition(point.time);

        point.position_dot = polynomial_trajectory.GetVelocity(point.time);
        point.position_dot_dot = polynomial_trajectory.GetAcceleration(point.time);
        if(point.position_dot_dot >= CONFIGURATION::maximum_acceleration)
        {
            result_trajectory.exceeds_acceleration_limit = true;
        }
        point.position_dot_dot_dot = polynomial_trajectory.GetJerk(point.time);
        if(point.position_dot_dot_dot >= CONFIGURATION::max_jerk)
        {
            result_trajectory.exceeds_jerk_limit = true;
        }
        std::cout << "number: " << i << ",position: " << point.position << ",position_dot: " << point.position_dot << ",position_dot_dot: " << point.position_dot_dot <<std::endl;
        result_trajectory.points.push_back(point);
    }
    for (int u = num_poly_step+1; u <= CONFIGURATION::num_trajectory_points; ++u) {
        TrajectoryPoint point;
        point.time = u * CONFIGURATION::delta_t_trajectory_points;
        //constamt velocity movement
        point.position = result_trajectory.points.at(u-1).position + CONFIGURATION::delta_t_trajectory_points * result_trajectory.points.at(num_poly_step).position_dot;

        point.position_dot = result_trajectory.points.at(num_poly_step).position_dot;
        point.position_dot_dot = result_trajectory.points.at(num_poly_step).position_dot_dot;
        if(point.position_dot_dot >= CONFIGURATION::maximum_acceleration)
        {
            result_trajectory.exceeds_acceleration_limit = true;
        }
        point.position_dot_dot_dot = result_trajectory.points.at(num_poly_step).position_dot_dot_dot;
        if(point.position_dot_dot_dot >= CONFIGURATION::max_jerk)
        {
            result_trajectory.exceeds_jerk_limit = true;
        }
        std::cout << "number: " << u << ",position: " << point.position << ",position_dot: " << point.position_dot << ",position_dot_dot: " << point.position_dot_dot <<std::endl;
        result_trajectory.points.push_back(point);

    }
    return result_trajectory;
}

double TrajectoryConverter::ScoreTrajectory(const DiscretizedTrajectory& trajectory, const QuarticPolynomialTrajectory& polynomial_trajectory, const PlanningState& goal)
{
    double score{std::numeric_limits<double>::max()};
    switch (trajectory.trajectory_type) {
        case TrajectoryType::VelocityKeeping:
        {
            //Score to punish high jerk
            const double& t = polynomial_trajectory.max_planning_time;
            const double& a3 = polynomial_trajectory.polynomial.a3;
            const double& a4 = polynomial_trajectory.polynomial.a4;
            double jerk_score = ((36*a3*a3*t + 144*a3*a4*t*t + 192*a4*a4*t*t*t));
            //score to punish slow convergence
            double time_score = polynomial_trajectory.max_planning_time;
            //score to punish big deviation from goal velocity
            double deviation_score = trajectory.points.back().position_dot - goal.position_dot;
            score = CONFIGURATION::jerk_cost_weight * jerk_score + CONFIGURATION::time_cost_weight * time_score + CONFIGURATION::s_deviation_weight * deviation_score;
            break;
        }
        default:
        {
            // Intentionally blank
            break;
        }
    }
    return score;
}


