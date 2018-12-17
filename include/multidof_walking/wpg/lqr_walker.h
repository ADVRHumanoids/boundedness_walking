#ifndef __MDOF_WALKER_H__
#define __MDOF_WALKER_H__

#include <OpenMpC/dynamics/LtiDynamics.h>

#include <multidof_walking/robot/robot_state.h>



namespace mdof {
 
    class Walker 
    {
      
    public:
        
        typedef boost::shared_ptr<Walker> Ptr;
        
        struct Options
        {
            Options();
            
            double com_height = 1.0;
            double step_duration = 1.0;
            double step_width = 0.2;
            int horizon_length = 5;
            
            double sagittal_step_weight = 1e-1;
            double lateral_step_weight = 1e2;
            
        };
        
        Walker(double control_dt, Options opt = Options());
        void start(double time, const RobotState& state);
        void stop();
        void set_vref(double vref_x);
        
        bool run(double time, const RobotState& state, RobotState& ref);
        
    private:
        
        typedef Eigen::Matrix<double, 1, 1> Scalar;
        
        enum class State { Idle, Walking, Stopping };
        
        void sagittal_setup();
        void lateral_setup();
        
        OpenMpC::dynamics::LtiDynamics::ConstPtr _lat_lipm, _sag_lipm;
        
        const Options _opt;
        const double GRAVITY;
        const double OMEGA;
        const double _dt;
        
        Eigen::MatrixXd _lat_Kx, _sag_Kx;
        Eigen::MatrixXd _lat_Kv, _sag_Kv;
        Eigen::MatrixXd _lat_Kstep;
        
        double _vref_x;
        
        
        State _current_state;
        double _last_step_time; // last time a step was triggered
        Eigen::Vector2d _current_zmp; // current stance foot position
        Eigen::Vector2d _current_swing_foot_target;
        int _current_swing_leg;
        
        Eigen::Vector3d _ext_state;
        
        
    };
    
}



#endif