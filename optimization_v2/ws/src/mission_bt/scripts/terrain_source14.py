"""Reviewed S13 adapters plus ground-plane and vehicle-volume projection."""
import sys
from pathlib import Path
p=Path(__file__).resolve().parents[1];nav=p.parent/'navigation_adapter'
s=(p.parent/'terrain_perception/src/terrain_node.cpp').read_text()
patches=[
 ('if(v.x()>-.65&&v.x()<1.55&&std::abs(v.y())<.7&&v.z()<1.1)continue;','if(s13::selfPoint(v.x(),v.y(),v.z(),self_box_))continue;'),
 ('J cfg_;','J cfg_;std::array<double,4> self_box_;'),
 ('f>>cfg_;','f>>cfg_;self_box_=cfg_.at("self_mask").get<std::array<double,4>>();'),
 ('std::array<int,2>cost','std::array<int,2>confirm{{0,0}};std::array<int,2>cost'),
 ('Cell&c=cells_[i];c.t=t;','Cell&c=cells_[i];const double observation_gap=t-c.t;c.t=t;'),
 ('if(s09::clear(c.cost[k]','c.confirm[k]=s13::confirmLethal(c.confirm[k],cost==100,observation_gap);if(s09::clear(c.cost[k]'),
 ('SharedPtr,2>grids_;','SharedPtr,2>grids_,confirmed_grids_;'),
 ('grids_[k++]=create_publisher','confirmed_grids_[k]=create_publisher<nav_msgs::msg::OccupancyGrid>("/s13/terrain/confirmed/"+name,2);grids_[k++]=create_publisher'),
 ('grids_[k]->publish(grid);','grids_[k]->publish(grid);for(size_t i=0;i<cells_.size();++i)grid.data[i]=(t-cells_[i].t<.15&&cells_[i].confirm[k]>=3)?100:-1;confirmed_grids_[k]->publish(grid);'),
 ('h[i]-(normal.x()*(p.x()-pos.x())+normal.y()*(p.y()-pos.y()))/normal.z()','s14::planeHeight(mean,normal,p)'),
 ('if(i>=0)bins[i].push_back(p.z);','if(i>=0&&s14::inCollisionVolume(p.z-floor(p.x,p.y),val("collision_ceiling_m")))bins[i].push_back(p.z);'),
 ('if(ht>.16)elevated->push_back(p);','if(ht>.16&&s14::inCollisionVolume(ht,val("collision_ceiling_m")))elevated->push_back(p);'),
 ('{"filtered_points",filtered->size()}','{"filtered_points",filtered->size()},{"collision_ceiling_m",val("collision_ceiling_m")}')]
patches += [
 ('bool planar=false;', 'bool planar=false;double slope_sigma=90.;bool slope_observed=false;'),
 ('Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> e(C/near.size());', 'slope_sigma=s14::slopeSigma(C,val("terrain_height_sigma_m"));slope_observed=slope_sigma<=val("terrain_slope_sigma_deg");Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> e(C/near.size());'),
 ('int cost=s09::risk(slope,step,rough,height,planar,profiles_[k]);', 'double reference_slope=std::acos(std::clamp(double(plane.values[2]),0.,1.))*180/M_PI;int cost=slope_observed?s09::risk(slope,step,rough,height,planar,profiles_[k]):s14::unobservableSlopeRisk(height,reference_slope,profiles_[k]);if(cost<0){c.confirm[k]=0;c.clear[k]=0;if(c.cost[k]!=100)c.cost[k]=-1;continue;}'),
 ('c.points,c.cost[0],c.cost[1]});', 'c.points,c.cost[0],c.cost[1],slope_sigma,slope_observed});'),
 ('"points","diff_cost","ackermann_cost"}', '"points","diff_cost","ackermann_cost","slope_sigma_deg","slope_observed"}')]
patches += [
 ('std::array<int,2>confirm{{0,0}};', 'std::array<double,2>risk_t{{-100.,-100.}};std::array<int,2>confirm{{0,0}};'),
 ('if(cost<0){c.confirm[k]=0;c.clear[k]=0;if(c.cost[k]!=100)c.cost[k]=-1;continue;}c.confirm[k]=s13::confirmLethal(c.confirm[k],cost==100,observation_gap);if(s09::clear(c.cost[k],cost,c.clear[k],cfg_["clear_frames"].get<int>()))c.cost[k]=cost;', 'c.confirm[k]=s13::confirmLethal(c.confirm[k],cost==100,observation_gap);s14::observeRisk(cost,t,cfg_["clear_frames"].get<int>(),c.cost[k],c.clear[k],c.risk_t[k]);'),
 ('if(c.cost[0]==100){lethal++;', 'if(s14::riskAt(c.cost[0],c.risk_t[0],t,val("unknown_after_s"))==100){lethal++;'),
 ('(t-cells_[i].t>val("unknown_after_s"))?-1:cells_[i].cost[k]', 's14::riskAt(cells_[i].cost[k],cells_[i].risk_t[k],t,val("unknown_after_s"))'),
 ('q.r=i>=0&&cells_[i].cost[0]==100?240:30;q.g=i>=0&&cells_[i].cost[0]<100?190:70;', 'int active_cost=i>=0?s14::riskAt(cells_[i].cost[0],cells_[i].risk_t[0],t,val("unknown_after_s")):-1;q.r=active_cost==100?240:30;q.g=active_cost>=0&&active_cost<100?190:70;'),
 ('c.points,c.cost[0],c.cost[1],slope_sigma,slope_observed});', 'c.points,s14::riskAt(c.cost[0],c.risk_t[0],t,val("unknown_after_s")),s14::riskAt(c.cost[1],c.risk_t[1],t,val("unknown_after_s")),slope_sigma,slope_observed,t-c.risk_t[0],t-c.risk_t[1],c.cost[0],c.cost[1]});'),
 ('"slope_sigma_deg","slope_observed"}', '"slope_sigma_deg","slope_observed","diff_risk_age_s","ackermann_risk_age_s","diff_stored_cost","ackermann_stored_cost"}')]
# Robust ground support, followed by geometry belonging to the current cell.
# Do not spread a neighbor's obstacle residual into an otherwise flat cell.
upstream=(p.parent/'terrain_perception/src/terrain_node.cpp').read_text()
start=upstream.index('   if(near.size()>=8)')
end=upstream.index('   // A positive',start)
old=upstream[start:end]
for before,after in patches:
    if before in old:old=old.replace(before,after)
new="""   auto local=s14::localSurface(near,center(i,h[i]),val("ground_distance_m"),val("terrain_height_sigma_m"));
   if(local.valid){mean=local.mean;normal=local.normal;rough=local.rough;slope=local.slope;slope_sigma=local.sigma;slope_observed=slope_sigma<=val("terrain_slope_sigma_deg");planar=rough<.02&&slope<15.;step=std::max(local.step,std::max(0.,height-(h[i]-ground)));}
   else{rough=.01;step=std::abs(height);}
   // A measured depression remains hazardous even with a planar bottom.
   if(height<-val("obstacle_min_height_m"))step=std::max(step,std::abs(height));
"""
patches.append((old,new))
for old,new in patches:
    if s.count(old)!=1:raise ValueError('TERRAIN_UPSTREAM_CHANGED '+old)
    s=s.replace(old,new)
prefix=''.join('#include "'+str(x)+'"\n' for x in [nav/'src/self_mask.hpp',nav/'src/observation_confirmation.hpp',p/'include/step_reference.hpp',p/'include/collision_volume.hpp',p/'include/slope_observability.hpp',p/'include/risk_evidence.hpp',p/'include/local_surface.hpp'])
Path(sys.argv[1]).write_text(prefix+s)
