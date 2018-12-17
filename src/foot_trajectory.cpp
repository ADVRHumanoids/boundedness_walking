#include <multidof_walking/trajectory/foot_trajectory.h>
#include <XBotInterface/Utils.h>

namespace mdof {
double time_warp(double tau, double beta)
{
    const double m = beta;
    const double b = m*m/(2*m - 1);
    const double a =  m*(-m + 1)/(2*m - 1);
    const double x0 = 1/(2*m);
    
    return b*tau - a*(std::log(std::cosh(30.*x0))/30. - std::log(std::cosh(30.*tau - 30.*x0))/30.);
}

double compute_swing_trajectory_normalized_xy(double tau, double* __dx, double* __ddx)
{
    double x, dx, ddx;
    XBot::Utils::FifthOrderPlanning(0, 0, 0, 1, 0, 1, tau, x, dx, ddx);
//     if(__dx) *__dx = dx;
//     if(__ddx) *__ddx = ddx;
    return x;
}

double compute_swing_trajectory_normalized_z(double final_height, double tau, double* dx, double* ddx)
{
    
    
    double x = std::pow(tau, 3)*std::pow(1-tau, 3);
    double x_max = 1./64.;
    x = x/x_max;
    
//     if(dx) *dx = powdx.dot(avec);
//     if(ddx) *ddx = powddx.dot(avec);
    
    return x;
    
}


Eigen::Vector3d compute_swing_trajectory(const Eigen::Vector3d& start, 
                                                    const Eigen::Vector3d& end, 
                                                    double clearance,
                                                    double t_start, 
                                                    double t_end, 
                                                    double time, 
                                                    double warp_factor,
                                                    Eigen::Vector3d* vel,
                                                    Eigen::Vector3d* acc)
{
    Eigen::Vector3d ret;
    
    double tau = std::min(std::max((time - t_start)/(t_end - t_start), 0.0), 1.0);
    double alpha = compute_swing_trajectory_normalized_xy(time_warp(tau, warp_factor));
    
    ret.head<2>() = (1-alpha)*start.head<2>() + alpha*end.head<2>();
    ret.z() = compute_swing_trajectory_normalized_z(end.z()/clearance, time_warp(tau, warp_factor))*clearance;
    
    
    return ret;
}


}
