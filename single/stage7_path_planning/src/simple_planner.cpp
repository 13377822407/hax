#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <queue>
#include <vector>
#include <cmath>
#include <mutex>

// 简单A*路径规划器，输入地图、起点、终点，输出Path
class SimplePlanner : public rclcpp::Node {
public:
  SimplePlanner() : Node("simple_planner") {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 1, std::bind(&SimplePlanner::mapCallback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal", 1, std::bind(&SimplePlanner::goalCallback, this, std::placeholders::_1));
    start_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "start", 1, std::bind(&SimplePlanner::startCallback, this, std::placeholders::_1));
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 1);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("path_marker", 1);
    RCLCPP_INFO(this->get_logger(), "SimplePlanner started. 发布/goal /start /map 即可规划路径");
  }

private:
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  geometry_msgs::msg::PoseStamped start_, goal_;
  bool has_map_ = false, has_start_ = false, has_goal_ = false;
  std::mutex mutex_;

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    map_ = msg;
    has_map_ = true;
    tryPlan();
  }
  void startCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    start_ = *msg;
    has_start_ = true;
    tryPlan();
  }
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    goal_ = *msg;
    has_goal_ = true;
    tryPlan();
  }

  void tryPlan() {
    if (!(has_map_ && has_start_ && has_goal_)) return;
    std::vector<std::pair<int,int>> path;
    if (!aStar(path)) {
      RCLCPP_WARN(this->get_logger(), "A* 规划失败，可能无可行路径");
      return;
    }
    publishPath(path);
  }

  bool aStar(std::vector<std::pair<int,int>>& out_path) {
    // 地图参数
    int w = map_->info.width, h = map_->info.height;
    double res = map_->info.resolution;
    double ox = map_->info.origin.position.x, oy = map_->info.origin.position.y;
    // 起终点转栅格
    int sx = (int)((start_.pose.position.x - ox) / res);
    int sy = (int)((start_.pose.position.y - oy) / res);
    int gx = (int)((goal_.pose.position.x - ox) / res);
    int gy = (int)((goal_.pose.position.y - oy) / res);
    if (!inMap(sx,sy,w,h) || !inMap(gx,gy,w,h)) return false;
    auto idx = [w](int x,int y){return y*w+x;};
    if (map_->data[idx(sx,sy)] > 50 || map_->data[idx(gx,gy)] > 50) return false;
    // A* open/closed
    struct Node {int x,y;double g,h;int px,py;};
    auto cmp = [](const Node&a,const Node&b){return a.g+a.h > b.g+b.h;};
    std::priority_queue<Node,std::vector<Node>,decltype(cmp)> open(cmp);
    std::vector<std::vector<bool>> closed(w,std::vector<bool>(h,false));
    std::vector<std::vector<std::pair<int,int>>> parent(w,std::vector<std::pair<int,int>>(h,{-1,-1}));
    open.push({sx,sy,0,hypot(gx-sx,gy-sy),-1,-1});
    while(!open.empty()){
      Node n=open.top();open.pop();
      if(n.x==gx&&n.y==gy){
        // 回溯路径
        int cx=gx,cy=gy;
        while(cx!=sx||cy!=sy){
          out_path.push_back({cx,cy});
          auto p=parent[cx][cy];cx=p.first;cy=p.second;
        }
        out_path.push_back({sx,sy});
        std::reverse(out_path.begin(),out_path.end());
        return true;
      }
      if(closed[n.x][n.y])continue;
      closed[n.x][n.y]=true;
      for(int dx=-1;dx<=1;dx++)for(int dy=-1;dy<=1;dy++)if(dx||dy){
        int nx=n.x+dx,ny=n.y+dy;
        if(!inMap(nx,ny,w,h))continue;
        if(map_->data[idx(nx,ny)]>50)continue;
        if(closed[nx][ny])continue;
        double g2=n.g+hypot(dx,dy);
        double h2=hypot(gx-nx,gy-ny);
        open.push({nx,ny,g2,h2,n.x,n.y});
        parent[nx][ny]={n.x,n.y};
      }
    }
    return false;
  }
  bool inMap(int x,int y,int w,int h){return x>=0&&x<w&&y>=0&&y<h;}

  void publishPath(const std::vector<std::pair<int,int>>& path) {
    nav_msgs::msg::Path msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = map_ ? map_->header.frame_id : "map";
    double res = map_->info.resolution;
    double ox = map_->info.origin.position.x, oy = map_->info.origin.position.y;
    for (auto& p : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position.x = ox + p.first * res + res/2.0;
      pose.pose.position.y = oy + p.second * res + res/2.0;
      pose.pose.orientation.w = 1.0;
      msg.poses.push_back(pose;
    }
    path_pub_->publish(msg);
    // 可视化Marker
    visualization_msgs::msg::Marker marker;
    marker.header = msg.header;
    marker.ns = "path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.2;
    marker.color.b = 0.2;
    for (auto& p : path) {
      geometry_msgs::msg::Point pt;
      pt.x = ox + p.first * res + res/2.0;
      pt.y = oy + p.second * res + res/2.0;
      pt.z = 0.05;
      marker.points.push_back(pt);
    }
    marker_pub_->publish(marker);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePlanner>());
  rclcpp::shutdown();
  return 0;
}
