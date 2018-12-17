#ifndef __MDOF_WALKING_FOOT_H__
#define __MDOF_WALKING_FOOT_H__

#include <eigen3/Eigen/Dense>

namespace mdof 
{
    Eigen::Vector3d compute_swing_trajectory(const Eigen::Vector3d& start, 
                                             const Eigen::Vector3d& end, 
                                             double clearance,
                                             double t_start, 
                                             double t_end,
                                             double time,
                                             double warp_factor = 1.0,
                                             Eigen::Vector3d * vel = nullptr,
                                             Eigen::Vector3d * acc = nullptr
                                             );
    
    double compute_swing_trajectory_normalized_xy(double tau, double* dx = 0, double* ddx = 0);
    double compute_swing_trajectory_normalized_z(double final_height, 
                                                 double tau, 
                                                 double* dx = 0, 
                                                 double* ddx = 0);
    double time_warp(double tau, double beta);
    double time_warp_tanh(double tau, double beta);
}

#endif