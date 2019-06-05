/*
 * Copyright (C) 2017 IIT-ADVR
 * Author:
 * email:
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <mdof_walking_plugin_plugin.h>

/* Specify that the class XBotPlugin::mdof_walking_plugin is a XBot RT plugin with name "mdof_walking_plugin" */
REGISTER_XBOT_PLUGIN_(XBotPlugin::mdof_walking_plugin)

using namespace XBot::Cartesian;
using XBot::Logger;

namespace XBotPlugin {

bool mdof_walking_plugin::init_control_plugin(XBot::Handle::Ptr handle)
{
    /* This function is called outside the real time loop, so we can
     * allocate memory on the heap, print stuff, ...
     * The RT plugin will be executed only if this init function returns true. */


    /* Save robot to a private member. */
    _robot = handle->getRobotInterface();
    _model = XBot::ModelInterface::getModel(handle->getPathToConfigFile());
    
    Eigen::VectorXd qhome;
    _model->getRobotState("home", qhome);
    _model->setJointPosition(qhome);
    _model->update();
    
    _q.resize(_model->getJointNum());
    _qdot = _q;
    
    _period = 0.002;
    
    YAML::Node yaml_file = YAML::LoadFile(handle->getPathToConfigFile());
    ProblemDescription ik_problem(yaml_file["WalkingStackCI"]["problem_description"], _model);
    std::string impl_name = "OpenSot";
    _ci = SoLib::getFactoryWithArgs<CartesianInterfaceImpl>("Cartesian" + impl_name + ".so", 
                                                            impl_name + "Impl", 
                                                            _model, ik_problem);
    _ci->enableOtg(_period);

    /* Initialize a logger which saves to the specified file. Remember that
     * the current date/time is always appended to the provided filename,
     * so that logs do not overwrite each other. */

    _logger = XBot::MatLogger::getLogger("/tmp/mdof_walking_log");
    
    
    mdof::Walker::Options opt;
    opt.com_height = 0.55;
    _walker = boost::make_shared<mdof::Walker>(_period, opt);
    
    _feet_links = {"l_sole", "r_sole"};
    
    _ci->update(0, 0);

    return true;


}

void mdof_walking_plugin::on_start(double time)
{
    /* This function is called on plugin start, i.e. when the start command
     * is sent over the plugin switch port (e.g. 'rosservice call /mdof_walking_plugin_switch true').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* Save the robot starting config to a class member */
    _model->syncFrom(*_robot, XBot::Sync::Position, XBot::Sync::MotorSide);
    
//     Eigen::VectorXd q;
//     _model->getJointPosition(q);
//     q += Eigen::VectorXd::Random(q.size())/100;
//     _model->setJointPosition(q);
//     _model->update();
    
    set_world_pose();    
    _ci->reset(time);

    /* Save the plugin starting time to a class member */
    _time = _start_time = time;
}

void mdof_walking_plugin::on_stop(double time)
{
    /* This function is called on plugin stop, i.e. when the stop command
     * is sent over the plugin switch port (e.g. 'rosservice call /mdof_walking_plugin_switch false').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
}


void mdof_walking_plugin::control_loop(double, double)
{
    
    double time = _time;
    double period = _period;
        
    switch(_current_state)
    {
        case State::IDLE:
        { 
            run_idle(time, period);
            break;
        }
        
        case State::HOMING:
        {
            run_homing(time, period);
            break;
        }
        
        case State::WALKING :
        {
            run_walking(time, period);
            break;
        }
    }
    
    _ref.log(_logger);

    /* Run CI loop */
    if(!_ci->update(time, period))
    {
        return;
    }
    
    /* Integrate solution */
    _model->getJointPosition(_q);
    _model->getJointVelocity(_qdot);
    
    _q += period * _qdot;
    
    _model->setJointPosition(_q);
    _model->update();
    
    _robot->setReferenceFrom(*_model, XBot::Sync::Position);
    _robot->move();

    _time += period;

}

void mdof_walking_plugin::run_idle(double time, double period)
{
    // prepare transition to homing
    // com must be on top of the stance foot centre

    Eigen::Vector3d com_ref = get_lfoot_pos();
    com_ref.z() += 0.55;

    Logger::info() << "Setting com ref to: " << com_ref.transpose() << Logger::endl();

    _ci->setTargetComPosition(com_ref, 5.0);

    _current_state = State::HOMING;
}

void mdof_walking_plugin::run_homing(double time, double period)
{
    // do nothing as the CI is updated externally

    if(_ci->getTaskState("com") == XBot::Cartesian::State::Online)
    {
        // homing finished, prepare transition to walking

        _model->getCOM(_state.com_pos);
        _state.com_vel.setZero();
        _state.foot_pos[0] = get_lfoot_pos();
        _state.foot_pos[1] = get_rfoot_pos();
        _ref = _state;

        Eigen::Affine3d w_T_f1, w_T_f2, w_T_f3, w_T_f4;
        _model->getPose("wheel_1", w_T_f1);
        _model->getPose("wheel_2", w_T_f2);
        _model->getPose("wheel_3", w_T_f3);
        _model->getPose("wheel_4", w_T_f4);

        _delta_rfoot_1 = w_T_f1.translation() - get_rfoot_pos();
        _delta_rfoot_4 = w_T_f4.translation() - get_rfoot_pos();
        _delta_lfoot_2 = w_T_f2.translation() - get_lfoot_pos();
        _delta_lfoot_3 = w_T_f3.translation() - get_lfoot_pos();

        Logger::info() << "Delta foot: " <<  _delta_rfoot_1.transpose() << Logger::endl();
        Logger::info() << "Delta foot: " <<  _delta_rfoot_4.transpose() << Logger::endl();
        Logger::info() << "Delta foot: " <<  _delta_lfoot_2.transpose() << Logger::endl();
        Logger::info() << "Delta foot: " <<  _delta_lfoot_3.transpose() << Logger::endl();

        _walker->start(time, _state);

        _current_state = State::WALKING;
    }
}

void mdof_walking_plugin::run_walking(double time, double period)
{
    _state = _ref;

    _walker->set_vref(0.1);
    _walker->run(time, _state, _ref);

    for(uint i : {0, 1})
    {
        _ref.foot_pos[i] = mdof::compute_swing_trajectory(
            _ref.foot_pos_start[i],
            _ref.foot_pos_goal[i],
            0.10,
            _ref.t_start[i],
            _ref.t_goal[i],
            time,
            1.5);
    }

    Eigen::Affine3d w_T_f;

    _ci->getPoseReferenceRaw("wheel_2", w_T_f);
    w_T_f.translation() = _ref.foot_pos[0] + _delta_lfoot_2;
    _ci->setPoseReference("wheel_2", w_T_f);

    _ci->getPoseReferenceRaw("wheel_3", w_T_f);
    w_T_f.translation() = _ref.foot_pos[0] + _delta_lfoot_3;
    _ci->setPoseReference("wheel_3", w_T_f);

    _ci->getPoseReferenceRaw("wheel_1", w_T_f);
    w_T_f.translation() = _ref.foot_pos[1] + _delta_rfoot_1;
    _ci->setPoseReference("wheel_1", w_T_f);

    _ci->getPoseReferenceRaw("wheel_4", w_T_f);
    w_T_f.translation() = _ref.foot_pos[1] + _delta_rfoot_4;
    _ci->setPoseReference("wheel_4", w_T_f);

    _ci->setComPositionReference(_ref.com_pos);
}

void mdof_walking_plugin::set_world_pose()
{
    Eigen::Affine3d w_T_l;
    w_T_l.setIdentity();
    w_T_l.translation() = get_lfoot_pos();

    Eigen::Affine3d w_T_pelvis;
    _model->getFloatingBasePose(w_T_pelvis);

    _model->setFloatingBasePose(w_T_pelvis*w_T_l.inverse());
    _model->update();
}

Eigen::Vector3d mdof_walking_plugin::get_rfoot_pos() const
{
    Eigen::Affine3d w_T_f1;
    _model->getPose("wheel_1", w_T_f1);

    Eigen::Affine3d w_T_f4;
    _model->getPose("wheel_4", w_T_f4);

    return 0.5*(w_T_f1.translation() + w_T_f4.translation());
}

Eigen::Vector3d mdof_walking_plugin::get_lfoot_pos() const
{
    Eigen::Affine3d w_T_f2;
    _model->getPose("wheel_2", w_T_f2);

    Eigen::Affine3d w_T_f3;
    _model->getPose("wheel_3", w_T_f3);

    return 0.5*(w_T_f2.translation() + w_T_f3.translation());
}


bool mdof_walking_plugin::close()
{
    /* This function is called exactly once, at the end of the experiment.
     * It can be used to do some clean-up, or to save logging data to disk. */

    /* Save logged data to disk */
    _logger->flush();

    return true;
}

mdof_walking_plugin::~mdof_walking_plugin()
{
  
}

}
