#include <multidof_walking/lti/lipm.h>

namespace mdof {
    
OpenMpC::dynamics::LtiDynamics::Ptr MakeLipmContinuousTime(double omega)
{
    Eigen::Matrix4d Alipm;
    Eigen::Matrix<double, 4, 2> Blipm;
    auto eye2 = Eigen::Matrix2d::Identity();
    auto zero2 = Eigen::Matrix2d::Zero();
    
    Alipm <<   zero2     , eye2,
             omega*omega*eye2, zero2;
             
    Blipm <<      zero2       , 
             -omega*omega*eye2;
             
    auto lipm = boost::make_shared<OpenMpC::dynamics::LtiDynamics>(Alipm, Blipm);
    
    Eigen::MatrixXd Cpos(2,4);
    Eigen::MatrixXd Cvel(2,4);
    Cpos << eye2, zero2;
    Cvel << zero2, eye2;
    
    lipm->addOutput("pos", Cpos);
    lipm->addOutput("vel", Cvel);
    
    return lipm;
}

OpenMpC::dynamics::LtiDynamics::Ptr MakeLipmExtended(double omega, double dt)
{
    auto lipm_dt = mdof::MakeLipmContinuousTime(omega)->makeDiscreteTime(dt);
    auto eye2 = Eigen::Matrix2d::Identity();
    auto zero2 = Eigen::Matrix2d::Zero();
    
    Eigen::MatrixXd Alipm_dt_delay_incr(6,6);
    Alipm_dt_delay_incr << lipm_dt->getA(), lipm_dt->getB(),
                            zero2    ,   zero2  ,        eye2       ;
     
    Eigen::MatrixXd Blipm_dt_delay_incr(6,2);
    Blipm_dt_delay_incr << zero2, zero2, eye2;
    
    auto ext =  boost::make_shared<OpenMpC::dynamics::LtiDynamics>(Alipm_dt_delay_incr, 
                                                               Blipm_dt_delay_incr, 
                                                               true);
    
    Eigen::MatrixXd Cpos(2,6);
    Eigen::MatrixXd Cvel(2,6);
    Eigen::MatrixXd Czmp(2,6);
    Eigen::MatrixXd Cdeltafoot(2,6);
    Cpos  << eye2, zero2, zero2;
    Cvel  << zero2, eye2, zero2;
    Czmp  << zero2, zero2, eye2;
    Cdeltafoot << -eye2, zero2, eye2;
    
    ext->addOutput("pos", Cpos);
    ext->addOutput("vel", Cvel);
    ext->addOutput("zmp", Czmp);
    ext->addOutput("delta_foot", Cdeltafoot);
    
    return ext;
    
}

}
