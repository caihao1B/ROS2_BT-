// Real Fields2Cover calls; this small example does not establish collision freedom.
#include <fields2cover.h>
#include <cmath>
#include <iostream>
#include <stdexcept>
void width_ok(double w){if(!std::isfinite(w)||w<=0)throw std::invalid_argument("positive width required");}
int main(){
  double width=.65;width_ok(width);
  F2CCell field(F2CLinearRing({F2CPoint(0,0),F2CPoint(12,0),F2CPoint(12,9),F2CPoint(0,9),F2CPoint(0,0)}));
  f2c::hg::ConstHL h;auto inner=h.generateHeadlands(F2CCells(field),1.);
  f2c::sg::BruteForce sg;auto rows=sg.generateSwaths(0,width,inner.getGeometry(0));
  F2CRobot robot(1.04,.9);robot.setMinTurningRadius(2.11);robot.setMaxDiffCurv(.25);
  f2c::pp::DubinsCurvesCC cc;cc.setDiscretization(.04);auto turn=cc.createTurn(robot,F2CPoint(0,0),0,F2CPoint(0,3),M_PI);
  if(rows.size()==0||turn.size()<3)throw std::runtime_error("missing real library output");
  bool rejected=false;try{width_ok(0);}catch(const std::invalid_argument&){rejected=true;}if(!rejected)return 2;
  std::cout<<"PASS: "<<rows.size()<<" F2C swaths; CC turn "<<turn.length()<<" m. Vehicle envelope still requires independent verification.\n";
}
