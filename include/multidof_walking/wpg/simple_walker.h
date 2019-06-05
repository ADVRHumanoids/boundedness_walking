#ifndef __MDOF_WALKER_H__
#define __MDOF_WALKER_H__

#include <OpenMpC/dynamics/LtiDynamics.h>

#include <multidof_walking/robot/robot_state.h>



namespace mdof {
 
    class Walker 
    {
      
    public:
        
        typedef std::function<void(const Eigen::Vector3d&, const Eigen::Vector3d&, double, double)> StepCallback;
        
        typedef boost::shared_ptr<Walker> Ptr;
        
        struct Options
        {
            Options();
            
            double com_height = 1.0;
            double step_duration = 1.0;
            int horizon_length = 5;
            
            double step_weight = 0.1;
            
        };
        
        Walker(double control_dt, Options opt = Options());
        void setStepCallback(StepCallback f);
        void start(double time, const RobotState& state);
        void stop();
        void set_vref(double vref_x);
        
        bool run(double time, const RobotState& state, RobotState& ref);
        
    private:
        
        enum class State { Idle, Walking, Stopping };
        
        void setup();
        
        OpenMpC::dynamics::LtiDynamics::ConstPtr _lipm;
        
        const Options _opt;
        const double GRAVITY;
        const double OMEGA;
        const double _dt;
        
        Eigen::MatrixXd _Kx, _Kv;
        
        Eigen::Vector2d _vref;
        
        State _current_state;
        double _last_step_time; // last time a step was triggered
        Eigen::Vector2d _next_step, _current_step;
        StepCallback _step_callback;
        int _swing_foot;
        
        
    };
    
}

#include <multidof_walking/lti/lipm.h>
#include <multidof_walking/trajectory/foot_trajectory.h>
#include <OpenMpC/solver/UnconstrainedMpc.h>
#include <chrono>
#include <iostream>
#include <XBotInterface/RtLog.hpp>

using XBot::Logger;

mdof::Walker::Walker(double control_dt, mdof::Walker::Options opt):
    GRAVITY(9.81),
    OMEGA(std::sqrt(GRAVITY/opt.com_height)),
    _dt(control_dt),
    _opt(opt),
    _vref(0, 0),
    _current_state(State::Idle),
    _current_step(0, 0),
    _next_step(0, 0)
{
    /* Sagittal plane controller setup */
    _lipm = mdof::MakeLipmContinuousTime(OMEGA);
    auto lipm_ext = mdof::MakeLipmExtended(OMEGA, _opt.step_duration);
    
    OpenMpC::UnconstrainedMpc lqr(lipm_ext, _opt.step_duration, _opt.horizon_length);
    lqr.addOutputTask("vel", Eigen::MatrixXd::Identity(2,2) * 1.0);
    lqr.addInputTask(Eigen::MatrixXd::Identity(2,2) * _opt.step_weight);
    
    auto tic = std::chrono::high_resolution_clock::now();
    lqr.compute();
    auto toc = std::chrono::high_resolution_clock::now();
    Logger::info() << "Gains computed in " << 
        std::chrono::duration_cast<std::chrono::microseconds>(toc-tic).count() / 1000.0 << " ms" << Logger::endl();
    
    _Kx = lqr.getStateFeedbackGain();
    _Kv = lqr.getOutputFeedforwardGain("vel");
    
    Logger::info(Logger::Severity::HIGH) << "Feedback gain is: " << _Kx << "\n";
    Logger::log() <<  "Vel ffw gain plane is : " << _Kv << Logger::endl();
}

bool mdof::Walker::run(double time, 
                       const mdof::RobotState& state, 
                       mdof::RobotState& ref)
{
    /* Construct extended state (x, v, p) */
    Eigen::Matrix<double, 6, 1> x_ext;
    x_ext << state.com_pos.head<2>(),
             state.com_vel.head<2>(),
             _current_step;
    
    /* If idle, do nothing */ 
    if(_current_state == State::Idle)
    {
        return true;
    }
    
    /* Compute new step with state feedback */
    if(time - _last_step_time >= _opt.step_duration)
    {
        _current_step = _next_step;
        x_ext.tail<2>() = _current_step;
        
        _next_step = _current_step + _Kx * x_ext + _Kv * _vref;
        
        
        ref.foot_pos_start[_swing_foot].head<2>() = state.foot_pos[_swing_foot].head<2>();
        ref.foot_pos_goal[_swing_foot].head<2>() = _next_step.head<2>();
        ref.t_start[_swing_foot] = time;
        ref.t_goal[_swing_foot] = time + _opt.step_duration * 0.8;
        
        _last_step_time = time;
        _swing_foot = 1 - _swing_foot;
    }
    
    Eigen::Vector4d x = x_ext.head<4>(), x_new;
    _lipm->integrate(x, _current_step, _dt, x_new);
    
    ref.com_pos.head<2>() = x_new.head<2>();
    ref.com_vel.head<2>() = x_new.segment<2>(2);
    ref.zmp.head<2>() = _current_step.head<2>();
    
    return true;
}

mdof::Walker::Options::Options()
{
}

void mdof::Walker::set_vref(double vref_x)
{
    _vref.x() = vref_x;
}


void mdof::Walker::start(double time, const mdof::RobotState& state)
{
    _swing_foot = 0;
    _current_step = _next_step = state.foot_pos[_swing_foot].head<2>();
    _last_step_time = time - _opt.step_duration * 2;
    _current_state = State::Walking;
}

void mdof::Walker::setStepCallback(mdof::Walker::StepCallback f)
{
    _step_callback = f;
}


#endif
