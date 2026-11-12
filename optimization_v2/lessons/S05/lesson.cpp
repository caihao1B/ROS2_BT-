// Teaching/probe: calls the installed robot_localization C++ EKF, not a copied filter.
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <robot_localization/ekf.hpp>
double update(double variance,double gate,std::ostream &debug,double *position=nullptr){
 robot_localization::Ekf filter;
 Eigen::VectorXd state=Eigen::VectorXd::Zero(15);state[6]=.5;
 filter.setState(state);Eigen::MatrixXd covariance=Eigen::MatrixXd::Identity(15,15)*.04;
 if(position){covariance(0,0)=1.;covariance(0,6)=covariance(6,0)=.15;}
 filter.setEstimateErrorCovariance(covariance);filter.setDebug(true,&debug);
 robot_localization::Measurement m;m.topic_name_="bounded_actual_ekf_probe";m.time_=rclcpp::Time(1000000000LL,RCL_ROS_TIME);m.measurement_=Eigen::VectorXd::Zero(15);m.measurement_[6]=.8;m.covariance_=Eigen::MatrixXd::Identity(15,15)*variance;m.update_vector_=std::vector<bool>(15,false);m.update_vector_[6]=true;m.mahalanobis_thresh_=gate;
 filter.correct(m);if(position)*position=filter.getState()[0];return filter.getState()[6];
}
int main(int argc,char **argv){
 std::ostringstream trace;double strong=update(.0025,6.,trace),weak=update(4.,6.,trace),rejected=update(.0025,.2,trace);
 if(!(strong>weak&&weak>.5&&std::abs(rejected-.5)<1e-12&&trace.str().find("Innovation is:")!=std::string::npos))return 1;
 double position=0.;update(.0025,6.,trace,&position);if(position<1.)return 2;
 if(argc>1){std::ofstream out(argv[1]);out<<trace.str();}
 std::cout<<"PASS actual robot_localization EKF: prior=0.5 measurement=0.8 strong="<<strong<<" inflated_covariance="<<weak<<" gated="<<rejected<<" position_correction_from_velocity_only="<<position<<"; internal innovation trace bytes="<<trace.str().size()<<"\n";
}
