// Teaching subset of production innovation gate, not a ROS node.
#include <Eigen/Dense>
#include <cassert>
#include <iostream>
bool admissible(Eigen::Vector2d innovation,Eigen::Matrix2d covariance){if(!covariance.allFinite()||covariance.determinant()<=0||covariance.diagonal().minCoeff()<=0)return false;double nis=innovation.dot(covariance.ldlt().solve(innovation));std::cout<<"innovation="<<innovation.norm()<<" m NIS="<<nis<<"\n";return innovation.norm()<=.15&&nis<=9.21;}
int main(){auto S=Eigen::Matrix2d::Identity()*.0034;assert(admissible({.03,.02},S));assert(!admissible({1.2,-.8},S));assert(!admissible({0,0},-S));std::cout<<"PASS: fixed label cannot override residual/covariance gate\n";}
