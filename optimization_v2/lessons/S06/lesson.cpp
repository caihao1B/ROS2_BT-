// Teaching only: linear 2D pose graph with fixed headings, not production SE(3).
#include <Eigen/Dense>
#include <iostream>
#include <stdexcept>
#include <vector>
struct Edge{int from,to;Eigen::Vector2d delta;double information;};
Eigen::MatrixXd solve(const std::vector<Edge> &edges){
  Eigen::MatrixXd H=Eigen::MatrixXd::Zero(5,5),b=Eigen::MatrixXd::Zero(5,2);
  H(0,0)=1e6;  // Anchor the coordinate gauge. No simulator truth enters here.
  for(const auto &e:edges){
    if(e.information<=0||!std::isfinite(e.information))throw std::invalid_argument("non-positive information");
    Eigen::VectorXd a=Eigen::VectorXd::Zero(5);a(e.from)=-1;a(e.to)=1;
    H+=e.information*a*a.transpose();b+=e.information*a*e.delta.transpose();
  }
  if(H.fullPivLu().rank()!=5)throw std::invalid_argument("disconnected unobservable graph");
  return H.ldlt().solve(b);
}
int main(){
  std::vector<Edge> e={{0,1,{2.1,0},100},{1,2,{0,2.1},100},{2,3,{-2,0},100},{3,4,{0,-2},100}};
  auto raw=solve(e);e.push_back({4,0,{0,0},400});auto optimized=solve(e);
  double before=(raw.row(4)-raw.row(0)).norm(),after=(optimized.row(4)-optimized.row(0)).norm();
  bool boundary=false;try{auto bad=e;bad[0].information=-1;solve(bad);}catch(const std::invalid_argument &){boundary=true;}
  bool disconnected=false;try{solve({e[0]});}catch(const std::invalid_argument &){disconnected=true;}
  std::cout<<"Teaching graph closure: "<<before<<" -> "<<after<<" m\ninvalid_information_rejected="<<boundary<<" disconnected_rejected="<<disconnected<<"\n";
  return after<before&&boundary&&disconnected?0:1;
}
