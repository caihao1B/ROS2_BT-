// Offline diagnostics with the installed Nav2 Hybrid-A*, never a mission PASS.
#include <nav2_smac_planner/a_star.hpp>
#include <nav2_smac_planner/node_hybrid.hpp>
#include <nav2_smac_planner/collision_checker.hpp>
#include <nav2_smac_planner/utils.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
using J=nlohmann::json;
int main(int argc,char**argv){
 if(argc<3)return 2;
 J input;std::ifstream(argv[1])>>input;
 rclcpp::init(0,nullptr);
 {
 auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("s14_offline_planner");
 auto ros=std::make_shared<nav2_costmap_2d::Costmap2DROS>();
 ros->set_parameter(rclcpp::Parameter("plugins",std::vector<std::string>{}));
 ros->on_configure(rclcpp_lifecycle::State());auto map=ros->getCostmap();
 double res=input["resolution"],ox=input["origin"][0],oy=input["origin"][1];int w=input["width"],h=input["height"];
 map->resizeMap(w,h,res,ox,oy);
 for(int y=0;y<h;y++)for(int x=0;x<w;x++)map->setCost(x,y,input["costs"][y*w+x]);
 unsigned bins=input.value("bins",72);nav2_smac_planner::GridCollisionChecker checker(ros,bins,node);
 nav2_costmap_2d::Footprint footprint;for(auto&p:input["footprint"]){geometry_msgs::msg::Point v;v.x=p[0];v.y=p[1];footprint.push_back(v);}
 // Always inspect the real polygon in this diagnostic; no inflation shortcut.
 checker.setFootprint(footprint,false,0.);
 nav2_smac_planner::SearchInfo info;info.minimum_turning_radius=2.11/res;info.non_straight_penalty=1.2;
 info.analytic_expansion_max_length=input.value("analytic_length",10.55)/res;
 info.allow_primitive_interpolation=input.value("interpolation",false);
 info.downsample_obstacle_heuristic=input.value("downsample_heuristic",true);
 nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::NodeHybrid> astar(nav2_smac_planner::MotionModel::DUBIN,info);
 int max_iter=1000000,num=0;astar.initialize(false,max_iter,1000,5000,input.value("time_limit",2.),401.,bins);astar.setCollisionChecker(&checker);
 auto pose=[&](const char* key){auto p=input[key];float x,y;map->worldToMapContinuous(p[0],p[1],x,y);double angle=p[2];int theta=int(std::round(angle/(2*M_PI/bins)));theta=(theta%int(bins)+int(bins))%int(bins);return std::tuple<float,float,unsigned>(x,y,theta);};
 auto[sx,sy,st]=pose("start");auto[gx,gy,gt]=pose("goal");astar.setStart(sx,sy,st);astar.setGoal(gx,gy,gt);
 J output;nav2_smac_planner::NodeHybrid::CoordinateVector path;auto start=std::chrono::steady_clock::now();
 try{output["found"]=astar.createPath(path,num,.04/res,[](){return false;});}catch(std::exception&e){output["found"]=false;output["error"]=e.what();}
 output["wall_s"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();output["iterations"]=num;output["poses"]=J::array();
 for(auto it=path.rbegin();it!=path.rend();++it){auto pose=nav2_smac_planner::getWorldCoords(it->x,it->y,map);output["poses"].push_back({pose.position.x,pose.position.y,it->theta});}
 std::ofstream(argv[2])<<output.dump(2)<<'\n';std::cout<<"FOUND "<<output["found"]<<" ITERATIONS "<<num<<" SECONDS "<<output["wall_s"]<<std::endl;
 ros->on_cleanup(rclcpp_lifecycle::State());
 nav2_smac_planner::NodeHybrid::destroyStaticAssets();
 }
 rclcpp::shutdown();return 0;
}
