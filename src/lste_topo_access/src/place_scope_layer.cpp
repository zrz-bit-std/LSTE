// A physical same-frame exclusion raster, independent of master-grid resizing.
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/cost_values.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <mutex>

namespace lste_topo_access {
class PlaceScopeLayer : public costmap_2d::CostmapLayer {
 public:
  void onInitialize() override {
    ros::NodeHandle nh("~/" + name_);
    std::string topic;
    nh.param("map_topic", topic, std::string("/lste/place_navigation_mask"));
    enabled_ = true;
    current_ = false;
    subscriber_ = nh.subscribe(topic, 1, &PlaceScopeLayer::incoming, this);
  }
  // The original immutable raster remains in world coordinates. A SLAM layer
  // may resize the master at any time, including before controller release ACK.
  void matchSize() override {
    // resizeMap clears the master before the next update cycle. Reapply within
    // the resize hook itself, before its mutex is released to the publisher.
    auto* master = layered_costmap_->getCostmap();
    updateCosts(*master, 0, 0, master->getSizeInCellsX(), master->getSizeInCellsY());
  }
  void reset() override { matchSize(); }
  void updateBounds(double, double, double, double* min_x, double* min_y,
                    double* max_x, double* max_y) override {
    auto* master = layered_costmap_->getCostmap();
    *min_x = std::min(*min_x, master->getOriginX());
    *min_y = std::min(*min_y, master->getOriginY());
    *max_x = std::max(*max_x, master->getOriginX() + master->getSizeInCellsX()*master->getResolution());
    *max_y = std::max(*max_y, master->getOriginY() + master->getSizeInCellsY()*master->getResolution());
  }
  void updateCosts(costmap_2d::Costmap2D& master, int min_i, int min_j,
                   int max_i, int max_j) override {
    nav_msgs::OccupancyGridConstPtr mask;
    bool bounded;
    { std::lock_guard<std::mutex> lock(mutex_); mask = mask_; bounded = bounded_; }
    if (!mask) { current_ = false; return; }
    if (!bounded) { current_ = true; return; }
    const auto& info = mask->info;
    const double yaw = std::atan2(2*info.origin.orientation.w*info.origin.orientation.z,
        1-2*info.origin.orientation.z*info.origin.orientation.z);
    const double cs = std::cos(yaw), sn = std::sin(yaw);
    for (int y = min_j; y < max_j; ++y) for (int x = min_i; x < max_i; ++x) {
      double wx, wy; master.mapToWorld(x,y,wx,wy);
      const double dx=wx-info.origin.position.x, dy=wy-info.origin.position.y;
      const int col=std::floor((cs*dx+sn*dy)/info.resolution);
      const int row=std::floor((-sn*dx+cs*dy)/info.resolution);
      // New master area outside the last committed bounded raster is excluded
      // until an acknowledged new snapshot permits it. Never lower obstacles.
      if (col<0 || row<0 || col>=int(info.width) || row>=int(info.height) ||
          mask->data[row*info.width+col]==100)
        master.setCost(x,y,costmap_2d::LETHAL_OBSTACLE);
    }
    current_ = true;
  }
 private:
  void incoming(const nav_msgs::OccupancyGridConstPtr& m) {
    const auto& i=m->info; const auto& q=i.origin.orientation;
    const bool valid=m->header.frame_id==layered_costmap_->getGlobalFrameID() &&
      i.width>0 && i.height>0 && uint64_t(i.width)*i.height<=4000000 &&
      m->data.size()==uint64_t(i.width)*i.height && std::isfinite(i.resolution) && i.resolution>0 &&
      std::isfinite(i.origin.position.x) && std::isfinite(i.origin.position.y) &&
      std::isfinite(q.z) && std::isfinite(q.w) && std::abs(q.x)<1e-9 && std::abs(q.y)<1e-9 &&
      std::abs(q.z*q.z+q.w*q.w-1)<1e-6 &&
      std::all_of(m->data.begin(),m->data.end(),[](int8_t v){return v==-1 || v==100;});
    if (!valid) { ROS_ERROR_THROTTLE(2,"Rejected invalid physical Place mask; retaining previous raster"); return; }
    const bool bounded=std::find(m->data.begin(),m->data.end(),100)!=m->data.end();
    std::lock_guard<std::mutex> lock(mutex_); mask_=m; bounded_=bounded;
  }
  ros::Subscriber subscriber_;
  std::mutex mutex_;
  nav_msgs::OccupancyGridConstPtr mask_;
  bool bounded_=false;
};
}
PLUGINLIB_EXPORT_CLASS(lste_topo_access::PlaceScopeLayer, costmap_2d::Layer)
