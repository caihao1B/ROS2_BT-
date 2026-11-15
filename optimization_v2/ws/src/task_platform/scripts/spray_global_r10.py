"""Spray-only native recovery: evaluate independent fixes against the candidate.

The moving jump/innovation guards and every numeric threshold remain unchanged.
The original R10 binary remains available and bound to the mowing edition.
"""
from common import *
from global_filter_r10 import derive

def recovery_source():
    source=(PKG/'r10_backend/stopped_revalidation.inc').read_text()
    begin=source.index('bool recoveryConsistent(');end=source.index('void observeRecovery(',begin)
    helper='''
struct RecoveryFix {double t;T odom;V antenna,lever;Eigen::Matrix2d covariance;};
std::deque<RecoveryFix> recovery_fixes_;bool recovery_gnss_valid_=false;
void recordRecoveryGNSS(const J&e,double t,const T&o,const Eigen::Matrix3d&C){
 V antenna(e["antenna"][0],e["antenna"][1],e["antenna"][2]);
 V lever(e["antenna_base_m"][0],e["antenna_base_m"][1],e["antenna_base_m"][2]);
 if(!antenna.allFinite()||!lever.allFinite()){recovery_gnss_valid_=false;return;}
 recovery_fixes_.push_back({t,o,antenna,lever,C.topLeftCorner<2,2>()});
 while(!recovery_fixes_.empty()&&t-recovery_fixes_.front().t>c("gnss_age_s"))recovery_fixes_.pop_front();
 recovery_gnss_valid_=true;
}
bool recoveryConsistent(const T& next){
 if(!recovery_gnss_valid_||recovery_fixes_.size()<3)return false;
 for(auto it=recovery_fixes_.end()-3;it!=recovery_fixes_.end();++it){
  if(!fresh(it->t,c("gnss_age_s")))return false;
  Eigen::Vector2d innovation=(it->antenna-next*it->odom*it->lever).head<2>();
  Eigen::Matrix2d S=it->covariance+Eigen::Matrix2d::Identity()*c("reference_variance_m2");
  double nis=innovation.dot(S.ldlt().solve(innovation));
  if(!innovation.allFinite()||!std::isfinite(nis)||innovation.norm()>c("innovation_m")||nis>c("nis_limit"))return false;
 }
 return true;
}
'''
    return source[:begin]+helper+source[end:]

def build():
    out=APP/'build/r10_spray_global_source';out.mkdir(parents=True,exist_ok=True)
    include=out/'stopped_revalidation.inc';recovery=recovery_source()
    if not include.exists() or include.read_text()!=recovery:include.write_text(recovery)
    code=derive((APP/'build/r08_source/global_manager.cpp').read_text())
    old=str(PKG/'r10_backend/stopped_revalidation.inc');assert code.count(old)==1
    code=code.replace(old,str(include))
    old='T o;if(!lookup(t,o))return;T reference='
    assert code.count(old)==1
    code=code.replace(old,'T o;if(!lookup(t,o))return;recordRecoveryGNSS(e,t,o,C);T reference=')
    old='if(!e.at("valid").get<bool>()||e.at("mode")!="fixed"){'
    assert code.count(old)==1;code=code.replace(old,old+'recovery_gnss_valid_=false;')
    old='gps_reason_="COVARIANCE_REJECTED";'
    assert code.count(old)==1;code=code.replace(old,'recovery_gnss_valid_=false;'+old)
    old='gps_reason_="MALFORMED_GNSS";'
    assert code.count(old)==1;code=code.replace(old,'recovery_gnss_valid_=false;'+old)
    code=code.replace('{"recovery_count",recovery_count_}', '{"recovery_count",recovery_count_},{"recovery_gnss_samples",recovery_fixes_.size()},{"recovery_gnss_reference","independent raw fixed samples tested against stopped candidate"}')
    target=out/'global_manager.cpp'
    if not target.exists() or target.read_text()!=code:target.write_text(code)
    cmake=(APP/'build/r10_global_source/CMakeLists.txt').read_text().replace('project(r10_global)','project(r10_spray_global)')
    cmake_path=out/'CMakeLists.txt'
    if not cmake_path.exists() or cmake_path.read_text()!=cmake:cmake_path.write_text(cmake)
    binary=APP/'build/r10_spray_global';subprocess.run(['cmake','-S',str(out),'-B',str(binary)],check=True)
    subprocess.run(['cmake','--build',str(binary),'--parallel','1'],check=True)
    return binary/'r10_global_manager'

if __name__=='__main__':build()
