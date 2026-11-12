#include "path_retention.hpp"
#include <cassert>
#include <iostream>
int main(){
 nav2_costmap_2d::Costmap2D map(300,200,.05,-5,-5,0);
 std::vector<geometry_msgs::msg::Point> fp;for(auto p:std::vector<std::pair<double,double>>{{-.3,-.52},{1.3,-.52},{1.3,.52},{-.3,.52}}){geometry_msgs::msg::Point q;q.x=p.first;q.y=p.second;fp.push_back(q);}
 nav_msgs::msg::Path path;path.header.frame_id="map";
 for(int i=0;i<=100;i++){geometry_msgs::msg::PoseStamped p;p.pose.position.x=i*.05;p.pose.orientation.w=1;path.poses.push_back(p);}
 auto start=path.poses[20].pose,goal=path.poses.back().pose;
 auto check=[&](){return s14::retain(path,start,goal,map,fp,true);};
 auto good=check();assert(good.valid&&good.first==20&&good.checked>100);
 auto cell=[&](double x,double y,int v){unsigned mx,my;assert(map.worldToMap(x,y,mx,my));map.setCost(mx,my,v);};
 // A tiny obstacle entirely inside the body must be caught, as must unknown.
 cell(3.5,.2,254);assert(!check().valid);cell(3.5,.2,255);assert(!check().valid);cell(3.5,.2,0);
 cell(-.1,0,254);assert(check().valid); // passed part is not future occupancy
 assert(!s14::retain(path,start,goal,map,fp,false).valid);
 goal.position.x+=.2;assert(check().reason=="GOAL_CHANGED");goal.position.x-=.2;
 start.position.y=.3;assert(check().reason=="TRACKING_DEPARTURE");start.position.y=0;
 start.orientation.z=std::sin(.15);start.orientation.w=std::cos(.15);assert(check().reason=="TRACKING_DEPARTURE");start.orientation.z=0;start.orientation.w=1;
 path.header.frame_id="odom";assert(check().reason=="PATH_FRAME_CHANGED");path.header.frame_id="map";
 path.poses.back().pose.position.x=20;goal=path.poses.back().pose;assert(!check().valid);
 std::cout<<"RETAIN_CURRENT_KNOWN_PATH; REJECT_INTERIOR_OBSTACLE_UNKNOWN_STALE_DEPARTURE_NEW_GOAL_AND_OUTSIDE; PAST_PATH_EXCLUDED\n";
}
