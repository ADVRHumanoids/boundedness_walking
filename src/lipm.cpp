#include <multidof_walking/lti/lipm.h>

namespace mdof {
    
OpenMpC::dynamics::LtiDynamics::Ptr MakeLipm(double omega)
{
    Eigen::Matrix2d Alipm;
    Eigen::Vector2d Blipm;
    
    Alipm <<      0     , 1,
             omega*omega, 0;
             
    Blipm <<      0      , 
             -omega*omega;
             
    auto lipm = boost::make_shared<OpenMpC::dynamics::LtiDynamics>(Alipm, Blipm);
    
    Eigen::MatrixXd Cpos(1,2);
    Eigen::MatrixXd Cvel(1,2);
    Cpos << 1, 0;
    Cvel << 0, 1;
    
    lipm->addOutput("pos", Cpos);
    lipm->addOutput("vel", Cvel);
    
    return lipm;
}

OpenMpC::dynamics::LtiDynamics::Ptr MakeLipmExtended(double omega, double dt)
{
    auto lipm_dt = mdof::MakeLipm(omega)->makeDiscreteTime(dt);
    
    Eigen::MatrixXd Alipm_dt_delay_incr(4,4);
    Alipm_dt_delay_incr << lipm_dt->getA(), Eigen::Vector3d::Zero(), lipm_dt->getB(),
                                  0  ,  0 ,            0           ,          0     ,
                                  0  ,  0 ,            0           ,          1     ;
     
    Eigen::Vector4d Blipm_dt_delay_incr(0, 0, 0, 1);
    
    auto ext =  boost::make_shared<OpenMpC::dynamics::LtiDynamics>(Alipm_dt_delay_incr, 
                                                               Blipm_dt_delay_incr, 
                                                               true);
    
    Eigen::MatrixXd Cpos(1,4);
    Eigen::MatrixXd Cvel(1,4);
    Eigen::MatrixXd Cstep(1,4);
    Eigen::MatrixXd Czmp(1,4);
    Cpos  << 1, 0, 0, 0;
    Cvel  << 0, 1, 0, 0;
    Cstep << 0, 0, 1, 0;
    Czmp  << 0, 0, 0, 1;
    
    ext->addOutput("pos", Cpos);
    ext->addOutput("vel", Cvel);
    ext->addOutput("step", Cstep);
    ext->addOutput("zmp", Czmp);
    
    return ext;
    
}

}