//
//  config.hpp
//  path_planning
//
//  Created by Jakob Klein on 25.05.21.
//

#ifndef config_hpp
#define config_hpp

namespace CONFIGURATION {

//>>>>>TRAJECTORY PLANNING<<<<<
constexpr unsigned int num_trajectory_points = 50;
constexpr double delta_t_trajectory_points = 0.02;
constexpr double trajectory_planning_time_total = 0.02;
constexpr double speed_limit_m_s = 49.5 * (0,44704); // 50 is the real speed limit, the factor converts to m/s
constexpr double max_jerk = 9.25;//10.0; //m/s^3
constexpr double maximum_accelearion = 9.25;// = 10; //m/s^2
constexpr double lateral_goal_shift = 0.2;
constexpr double longitudinal_velocity_goal_shift = 2.0;



//>>>>>LANE ASSIGNMENT<<<<<
//Lane limits in frenet frame
constexpr double left_lane_lower_limit = 0.0;
constexpr double left_lane_upper_limit = 4.0;

constexpr double middle_lane_lower_limit = 4.0;
constexpr double middle_lane_upper_limit = 8.0;

constexpr double right_lane_lower_limit = 8.0;
constexpr double right_lane_upper_limit = 12.0;

//>>>>>OTHER<<<<<<
constexpr double object_bounding_circle_radius = 3.0;



}

#endif /* config_hpp */
