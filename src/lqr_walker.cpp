#include <multidof_walking/wpg/lqr_walker.h>

#include <multidof_walking/lti/lipm.h>
#include <multidof_walking/trajectory/foot_trajectory.h>
#include <OpenMpC/solver/UnconstrainedMpc.h>
#include <chrono>
#include <iostream>
#include <XBotInterface/RtLog.hpp>

using XBot::Logger;

mdof::Walker::Options::Options()
{

}


mdof::Walker::Walker(double control_dt, mdof::Walker::Options opt):
    _opt(opt), 
    GRAVITY(9.81),
    OMEGA(std::sqrt(GRAVITY / opt.com_height)),
    _dt(control_dt),
    _last_step_time(0.0),
    _current_zmp(0, 0),
    _current_swing_foot_target(0, 0),
    _vref_x(0.0),
    _current_swing_leg(0),
    _ext_state(0,0,0),
    _current_state(State::Idle)
{
    sagittal_setup();
    lateral_setup();
    
}


void mdof::Walker::sagittal_setup()
{
    /* Sagittal plane controller setup */
    _sag_lipm = mdof::MakeLipmContinuousTime(OMEGA);
    auto sag_lipm_ext = mdof::MakeLipmExtended(OMEGA, _opt.step_duration);
    
    OpenMpC::UnconstrainedMpc sag_lqr(sag_lipm_ext, _opt.step_duration, _opt.horizon_length);
    sag_lqr.addOutputTask("vel", Eigen::MatrixXd::Identity(1,1) * 1.0);
    sag_lqr.addInputTask(Eigen::MatrixXd::Identity(1,1) * _opt.sagittal_step_weight);
    
    auto tic = std::chrono::high_resolution_clock::now();
    sag_lqr.compute();
    auto toc = std::chrono::high_resolution_clock::now();
    Logger::info() << "Mpc law for sagittal plane computed in " << 
        std::chrono::duration_cast<std::chrono::microseconds>(toc-tic).count() / 1000.0 << " ms" << Logger::endl();
    
    _sag_Kx = sag_lqr.getStateFeedbackGain();
    _sag_Kv = sag_lqr.getOutputFeedforwardGain("vel");
    
    Logger::info(Logger::Severity::HIGH) << "Feedback gain for sagittal plane is: " << _sag_Kx << "\n";
    Logger::log() <<  "Vel ffw gain for sagittal plane is : " << _sag_Kv << Logger::endl();
}


void mdof::Walker::lateral_setup()
{
    /* Lateral plane controller setup */
    _lat_lipm = mdof::MakeLipmContinuousTime(OMEGA);
    auto lat_lipm_ext = mdof::MakeLipmExtended(OMEGA, _opt.step_duration);
    
    OpenMpC::UnconstrainedMpc lat_lqr(lat_lipm_ext, _opt.step_duration, _opt.horizon_length);
    lat_lqr.addOutputTask("zmp", Eigen::MatrixXd::Identity(1,1) * _opt.lateral_step_weight);
    lat_lqr.addOutputTask("vel", Eigen::MatrixXd::Identity(1,1) * 1.0);
    lat_lqr.addInputTask(Eigen::MatrixXd::Identity(1,1) * 1.0);
    
    auto tic = std::chrono::high_resolution_clock::now();
    lat_lqr.compute();
    auto toc = std::chrono::high_resolution_clock::now();
    Logger::info() << "Mpc law for lateral plane computed in " << 
        std::chrono::duration_cast<std::chrono::microseconds>(toc-tic).count() / 1000.0 << " ms" << Logger::endl();
    
    _lat_Kx = lat_lqr.getStateFeedbackGain();
    _lat_Kv = lat_lqr.getOutputFeedforwardGain("vel");
    
    Eigen::VectorXd dstep_ffwd(_opt.horizon_length);
    for(int i = 0; i < _opt.horizon_length; i++)
    {
        dstep_ffwd[i] = std::pow(-1, i);
    }
    
    _lat_Kstep = lat_lqr.getOutputFeedforwardGainPreview("zmp")*dstep_ffwd;
    
    Logger::info(Logger::Severity::HIGH) << "Feedback gain for lateral plane is: " << _lat_Kx << "\n";
    Logger::log() <<  "Vel ffw gain for lateral plane is : " << _lat_Kv << "\n";
    Logger::log() <<  "Step ffw gain for lateral plane is: " << _lat_Kstep << Logger::endl();
    
}

void mdof::Walker::set_vref(double vref_x)
{
    _vref_x = vref_x;
}

void mdof::Walker::start(double time, const RobotState& state)
{
    _current_zmp = state.zmp.head<2>();
    _last_step_time = time;
    _current_state = State::Walking;
}

void mdof::Walker::stop()
{
    _current_state = State::Stopping;
}


bool mdof::Walker::run(double time, const mdof::RobotState& state, mdof::RobotState& ref)
{
    if(_current_state == State::Idle)
    {
        return true;
    }
    
    if(time - _last_step_time >= _opt.step_duration)
    {
        
        /* Manage foot landing */
        _current_zmp = state.eq_foot_pos.head<2>();
        ref.foot_contact[_current_swing_leg] = true;
        
        /* Switch swing leg */
        _current_swing_leg = 1 - _current_swing_leg;
        
        /* Compute next sagittal step */
        Eigen::Vector3d sag_extended_state;
        sag_extended_state << state.com_pos.x(), 
                              state.com_vel.x(),
                              _current_zmp.x();
        
        Logger::info() << "Current extend sagittal state is: " << sag_extended_state.transpose() << Logger::endl();
                              
        Scalar sag_next_step = _sag_Kx * sag_extended_state + _sag_Kv * _vref_x;
        Scalar sag_u_nom = _sag_Kx * _ext_state + _sag_Kv * _vref_x;
        
        /* Compute next lateral step */
        Eigen::Vector3d lat_extended_state;
        lat_extended_state << state.com_pos.y(), 
                              state.com_vel.y(),
                              _current_zmp.y();
                              
        Logger::info() << "Current extend lateral state is: " << lat_extended_state.transpose() << Logger::endl();
                              
        double lat_next_step_ref = _opt.step_width / 2.0 * (_current_swing_leg == 0 ? 1 : -1);
                              
        Scalar lat_next_step = _lat_Kx * lat_extended_state + 
                               _lat_Kv * 0.0 + 
                               _lat_Kstep * lat_next_step_ref;
                               
        /* Next overall step */
        Eigen::Vector2d computed_step;
        computed_step << sag_next_step, lat_next_step;
        _current_swing_foot_target = computed_step + _current_zmp;
        _last_step_time = time;
        ref.eq_foot_pos.head<2>() = _current_swing_foot_target;
        if(computed_step.norm() > 0.1)
        {
            ref.foot_contact[_current_swing_leg] = false;
            ref.foot_pos_start[_current_swing_leg] = ref.foot_pos[_current_swing_leg];
            ref.foot_pos_goal[_current_swing_leg].head<2>() = _current_swing_foot_target;
            ref.t_start[_current_swing_leg] = time + _opt.step_duration * 0.1;
            ref.t_goal[_current_swing_leg]  = time + _opt.step_duration * 0.9; // HACK HARDCODED
            
        }
        else
        {
            Logger::info(Logger::Severity::HIGH, "Skipping step..\n");
        }
        
        Logger::info() << "Computed step  : " << computed_step.transpose() << Logger::endl();
        Logger::info() << "Target foot pos: " << _current_swing_foot_target.transpose() << Logger::endl();
        
    }
    
    /* Sagittal update */
    Eigen::Vector2d sag_lipm_state, sag_next_lipm_state;
    sag_lipm_state << state.com_pos.x(), state.com_vel.x();
    
    Scalar sag_zmp;
    sag_zmp << _current_zmp.x();
    
    _sag_lipm->integrate(sag_lipm_state, sag_zmp, _dt, sag_next_lipm_state);
    
    /* Lateral update */
    Eigen::Vector2d lat_lipm_state, lat_next_lipm_state;
    lat_lipm_state << state.com_pos.y(), state.com_vel.y();
    
    Scalar lat_zmp;
    lat_zmp << _current_zmp.y();
    
    _lat_lipm->integrate(lat_lipm_state, lat_zmp, _dt, lat_next_lipm_state);
    
    /* Foot trajectory */
    for(int i : {0, 1})
    {
        ref.foot_pos[i] = mdof::compute_swing_trajectory(ref.foot_pos_start[i], ref.foot_pos_goal[i],
                                       0.05, ref.t_start[i], ref.t_goal[i], time);
    }
    
    /* Fill next state */
    ref.com_pos.x() = sag_next_lipm_state[0];
    ref.com_pos.y() = lat_next_lipm_state[0];
    ref.com_vel.x() = sag_next_lipm_state[1];
    ref.com_vel.y() = lat_next_lipm_state[1];
    ref.zmp.head<2>() << sag_zmp, lat_zmp;
    
    return true;
}


