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
    
    YAML::Node yaml_file = YAML::LoadFile(handle->getPathToConfigFile());
    ProblemDescription ik_problem(yaml_file["CartesianInterface"]["problem_description"], _model);
    std::string impl_name = "OpenSot";
    _ci = SoLib::getFactoryWithArgs<CartesianInterfaceImpl>("Cartesian" + impl_name + ".so", 
                                                            impl_name + "Impl", 
                                                            _model, ik_problem);
    _ci->enableOtg(0.002);

    /* Initialize a logger which saves to the specified file. Remember that
     * the current date/time is always appended to the provided filename,
     * so that logs do not overwrite each other. */

    _logger = XBot::MatLogger::getLogger("/tmp/mdof_walking_log");
    
    
    mdof::Walker::Options opt;
    _walker = boost::make_shared<mdof::Walker>(0.002, opt);
    
    _feet_links = {"l_sole", "r_sole"};

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
    set_world_pose();
    
    _model->getCOM(_state.com_pos);
    _state.zmp = _state.com_pos;
    _state.zmp.z() = 0.0;
    _state.eq_foot_pos = _state.zmp;
    _model->getPose(_feet_links[0], _T_lsole);
    _model->getPose(_feet_links[1], _T_rsole);
    _state.foot_pos[0] = _T_lsole.translation();
    _state.foot_pos[1] = _T_rsole.translation();
    _state.foot_pos_goal = _state.foot_pos_start = _state.foot_pos;
    _state.foot_contact.fill(true);
    _ref = _state;
    
    _ci->reset(time);

    /* Save the plugin starting time to a class member */
    _start_time = time;
}

void mdof_walking_plugin::on_stop(double time)
{
    /* This function is called on plugin stop, i.e. when the stop command
     * is sent over the plugin switch port (e.g. 'rosservice call /mdof_walking_plugin_switch false').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
}


void mdof_walking_plugin::control_loop(double time, double period)
{
    /* This function is called on every control loop from when the plugin is start until
     * it is stopped.
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* The following code checks if any command was received from the plugin standard port
     * (e.g. from ROS you can send commands with
     *         rosservice call /mdof_walking_plugin_cmd "cmd: 'MY_COMMAND_1'"
     * If any command was received, the code inside the if statement is then executed. */
	 
	static bool started = false; 
	if(!started && time - _start_time > 2.0)
	{
		_walker->set_vref(0.1);
	    _walker->start(time, _state);
		started = true;
	}

    if(!current_command.str().empty()){

        if(current_command.str() == "WALK"){
            
        }

        if(current_command.str() == "STOP"){
            _walker->set_vref(0.0);
            _walker->stop();
        }

    }
    
    _state = _ref;
    _walker->run(time, _state, _ref);
    
    _ref.log(_logger);
    
    _ci->setComPositionReference(_ref.com_pos);
    
    _T_lsole.translation() = _ref.foot_pos[0];
    _ci->setPoseReference(_feet_links[0], _T_lsole);
    
    _T_rsole.translation() = _ref.foot_pos[1];
    _ci->setPoseReference(_feet_links[1], _T_rsole);
    
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

}

void mdof_walking_plugin::set_world_pose()
{
    Eigen::Affine3d Tlsole;
    _model->getPose("l_sole", Tlsole);
    
    Eigen::Affine3d Trsole;
    _model->getPose("r_sole", Trsole);
    
    auto Tmid = Tlsole;
    Tmid.translation() = (Tlsole.translation() + Trsole.translation()) / 2.0;
    
    _model->setFloatingBasePose(Tmid.inverse());
    _model->update();
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
