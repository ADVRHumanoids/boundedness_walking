#ifndef __MOMENTUM_CONTROLLER_H__
#define __MOMENTUM_CONTROLLER_H__

#include <eigen3/Eigen/Dense>
#include <XBotInterface/ModelInterface.h>

namespace mdof
{
    
    class MomentumController 
    {
      
    public:
        
        typedef std::shared_ptr<MomentumController> Ptr;
        
        MomentumController(XBot::ImuSensor::ConstPtr imu,
                           const double dt);
        
        void setGains(double kp, double kd);
        
        void update();
        
        Eigen::Vector3d getOmega() const;
        
        
    private:
        
        Eigen::Vector3d _omega;
        double _omega_max;
        double _kp, _kd;
        double _dt;
        
        XBot::ImuSensor::ConstPtr _imu;
        
        XBot::MatLogger::Ptr _logger;
    };
    
}

mdof::MomentumController::MomentumController(XBot::ImuSensor::ConstPtr imu,
                                             const double dt):
    _imu(imu),
    _dt(dt),
    _omega_max(1.0),
    _kp(1.0),
    _kd(0.1)
{
    _logger = XBot::MatLogger::getLogger("/tmp/momentum_controller_log");
    _omega.setZero();
}

void mdof::MomentumController::setGains(double kp, double kd)
{
    _kp = kp;
    _kd = kd;
}

Eigen::Vector3d mdof::MomentumController::getOmega() const
{
    return _omega;
}

void mdof::MomentumController::update()
{
    // get world z-axis w.r.t. imu frame
    Eigen::Matrix3d w_R_imu;
    _imu->getOrientation(w_R_imu);
    Eigen::Vector3d imu_z_world = w_R_imu.transpose().col(2);
    
    // waist z-axis is 0 0 1
    Eigen::Vector3d imu_z_waist(0.0, 0.0, 1.0);
    
    // get angular vel from imu
    Eigen::Vector3d omega_imu;
    _imu->getAngularVelocity(omega_imu);
    
    // control law
    Eigen::Vector3d omega_dot = _kp * imu_z_waist.cross(imu_z_world) + _kd * omega_imu;
    omega_dot.z() = 0.0;
    
    // integrate
    _omega += omega_dot * _dt;
    
    // anti windup
    _omega = _omega.array().min(_omega_max).max(-_omega_max);
    
    // log 
    _logger->add("imu_z_world", imu_z_world);
    _logger->add("imu_z_waist", imu_z_waist);
    _logger->add("omega_imu", omega_imu);
    _logger->add("omega_dot", omega_imu);
    _logger->add("_omega", omega_imu);
    
    
}

#endif

