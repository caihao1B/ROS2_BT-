#pragma once
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav_msgs/msg/path.hpp>
#include <cmath>
#include <string>
namespace s14 {
inline double heading(const geometry_msgs::msg::Pose & p) {
 const auto&q=p.orientation;return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
}
inline double angle(double x){return std::atan2(std::sin(x),std::cos(x));}
struct Retention {bool valid=false;size_t first=0,checked=0;double distance=1e9,yaw_error=1e9;std::string reason="NO_PREVIOUS_PATH";};
inline bool bodyClear(nav2_costmap_2d::Costmap2D & map,
 const std::vector<geometry_msgs::msg::Point> & footprint,double x,double y,double h) {
 std::vector<nav2_costmap_2d::MapLocation> polygon,cells;
 for(const auto&p:footprint){nav2_costmap_2d::MapLocation q;
  if(!map.worldToMap(x+std::cos(h)*p.x-std::sin(h)*p.y,y+std::sin(h)*p.x+std::cos(h)*p.y,q.x,q.y))return false;
  polygon.push_back(q);
 }
 map.convexFillCells(polygon,cells);
 for(const auto&q:cells)if(map.getCost(q.x,q.y)>=254)return false; // includes unknown 255
 return !cells.empty();
}
inline Retention retain(const nav_msgs::msg::Path & path,
 const geometry_msgs::msg::Pose & start,const geometry_msgs::msg::Pose & goal,
 nav2_costmap_2d::Costmap2D & map,std::vector<geometry_msgs::msg::Point> footprint,bool current) {
 Retention r;if(path.poses.empty())return r;
 if(!current){r.reason="MAP_NOT_CURRENT";return r;}
 if(path.header.frame_id!="map"){r.reason="PATH_FRAME_CHANGED";return r;}
 const auto & end=path.poses.back().pose;
 if(std::hypot(end.position.x-goal.position.x,end.position.y-goal.position.y)>.05||std::abs(angle(heading(end)-heading(goal)))>.1){r.reason="GOAL_CHANGED";return r;}
 for(size_t i=0;i<path.poses.size();++i){const auto&p=path.poses[i].pose;
  double d=std::hypot(p.position.x-start.position.x,p.position.y-start.position.y);
  if(d<r.distance){r.distance=d;r.first=i;}
 }
 r.yaw_error=std::abs(angle(heading(start)-heading(path.poses[r.first].pose)));
 if(r.distance>.25||r.yaw_error>.1){r.reason="TRACKING_DEPARTURE";return r;}
 // Densify so the corner motion between samples is <= 2cm. A 1cm
 // footprint pad conservatively covers the intervening swept rectangle.
 double radius=0;for(const auto&p:footprint)radius=std::max(radius,std::hypot(p.x,p.y));
 nav2_costmap_2d::padFootprint(footprint,.01);
 if(!bodyClear(map,footprint,start.position.x,start.position.y,heading(start))){r.reason="CURRENT_BODY_OCCUPIED_OR_UNKNOWN";return r;}
 for(size_t i=r.first;i<path.poses.size();++i){
  const auto&a=path.poses[i].pose;const auto&b=path.poses[std::min(i+1,path.poses.size()-1)].pose;
  double dx=b.position.x-a.position.x,dy=b.position.y-a.position.y,dh=angle(heading(b)-heading(a));
  int count=std::max(1,int(std::ceil((std::hypot(dx,dy)+radius*std::abs(dh))/.02)));
  for(int j=0;j<=count;++j){double u=double(j)/count;++r.checked;
   if(!bodyClear(map,footprint,a.position.x+u*dx,a.position.y+u*dy,heading(a)+u*dh)){r.reason="PATH_OCCUPIED_UNKNOWN_OR_OUTSIDE";return r;}
  }
 }
 r.valid=true;r.reason="CURRENT_PATH_VALID_RETAINED";return r;
}
}
