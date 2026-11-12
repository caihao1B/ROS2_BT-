#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <mutex>
namespace s09 {
class ObservedRiskLayer:public nav2_costmap_2d::CostmapLayer {
 rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid_;std::mutex mutex_;double age_=0.6;
public:
 void onInitialize() override {auto node=node_.lock();if(!node)throw std::runtime_error("costmap node expired");declareParameter("topic",rclcpp::ParameterValue("/s09/risk/diff"));declareParameter("max_age",rclcpp::ParameterValue(.6));std::string topic;node->get_parameter(name_+".topic",topic);node->get_parameter(name_+".max_age",age_);sub_=node->create_subscription<nav_msgs::msg::OccupancyGrid>(topic,2,[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){std::lock_guard<std::mutex>l(mutex_);grid_=m;});current_=false;enabled_=true;}
 void updateBounds(double,double,double,double*minx,double*miny,double*maxx,double*maxy) override{auto c=layered_costmap_->getCostmap();*minx=std::min(*minx,c->getOriginX());*miny=std::min(*miny,c->getOriginY());*maxx=std::max(*maxx,c->getOriginX()+c->getSizeInMetersX());*maxy=std::max(*maxy,c->getOriginY()+c->getSizeInMetersY());}
 void updateCosts(nav2_costmap_2d::Costmap2D&master,int min_i,int min_j,int max_i,int max_j) override{
  std::lock_guard<std::mutex>l(mutex_);auto node=node_.lock();double age=grid_?(node->now()-rclcpp::Time(grid_->header.stamp)).seconds():1e6;current_=grid_&&grid_->header.frame_id==layered_costmap_->getGlobalFrameID()&&age>=-.05&&age<age_;
  for(int y=min_j;y<max_j;y++)for(int x=min_i;x<max_i;x++){unsigned char cost=nav2_costmap_2d::NO_INFORMATION;if(current_){double wx,wy;master.mapToWorld(x,y,wx,wy);int gx=int(std::floor((wx-grid_->info.origin.position.x)/grid_->info.resolution)),gy=int(std::floor((wy-grid_->info.origin.position.y)/grid_->info.resolution));if(gx>=0&&gy>=0&&gx<int(grid_->info.width)&&gy<int(grid_->info.height)){int v=grid_->data[gy*grid_->info.width+gx];cost=v<0?255:v>=100?254:static_cast<unsigned char>(v*2.5);}}master.setCost(x,y,cost);}
 }
 void reset() override{std::lock_guard<std::mutex>l(mutex_);grid_.reset();current_=false;}
 bool isClearable() override{return false;}
};
}
PLUGINLIB_EXPORT_CLASS(s09::ObservedRiskLayer,nav2_costmap_2d::Layer)
