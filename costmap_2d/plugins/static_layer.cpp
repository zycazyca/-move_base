/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2015, Fetch Robotics, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <costmap_2d/static_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::StaticLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;		// 255
using costmap_2d::LETHAL_OBSTACLE;		// 254
using costmap_2d::FREE_SPACE;			// 0

namespace costmap_2d
{

StaticLayer::StaticLayer() : dsrv_(NULL) {}

StaticLayer::~StaticLayer()
{
  if (dsrv_)
    delete dsrv_;
}

// onInitialize() 是 Layer 中定义的虚函数，实际上是对 Layer 默认构造函数的一个补充
void StaticLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;						// current_ 变量一劳永逸

  global_frame_ = layered_costmap_->getGlobalFrameID();		// return global_frame_; global_frame 是 LayeredCostmap 的成员变量

  std::string map_topic;
  nh.param("map_topic", map_topic, std::string("map"));
  nh.param("first_map_only", first_map_only_, false);
  nh.param("subscribe_to_updates", subscribe_to_updates_, false);

  nh.param("track_unknown_space", track_unknown_space_, true);
  nh.param("use_maximum", use_maximum_, false);

  int temp_lethal_threshold, temp_unknown_cost_value;
  nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
  nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));
  nh.param("trinary_costmap", trinary_costmap_, true);

  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);	// lethal_threshold_ 不会超过100
  unknown_cost_value_ = temp_unknown_cost_value;

  // Only resubscribe if topic has changed	如果话题改变了就重新订阅
  if (map_sub_.getTopic() != ros::names::resolve(map_topic))
  {
    // we'll subscribe to the latched topic that the map server uses
    ROS_INFO("Requesting the map...");
    map_sub_ = g_nh.subscribe(map_topic, 1, &StaticLayer::incomingMap, this);	// 订阅地图话题的回调
    map_received_ = false;
    has_updated_data_ = false;

    ros::Rate r(10);
    while (!map_received_ && g_nh.ok())
    {
      ros::spinOnce();				// 使用循环函数等待接收地图话题
      r.sleep();
    }

    ROS_INFO("Received a %d X %d map at %f m/pix", getSizeInCellsX(), getSizeInCellsY(), getResolution());

    if (subscribe_to_updates_)			// 是否需要根据订阅 更新静态地图 默认是 false
    {
      ROS_INFO("Subscribing to updates");
// 更新静态地图来自话题 “map_updates”
      map_update_sub_ = g_nh.subscribe(map_topic + "_updates", 10, &StaticLayer::incomingUpdate, this);

    }
  }
  else
  {
    has_updated_data_ = true;
  }

  if (dsrv_)
  {
    delete dsrv_;
  }

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &StaticLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void StaticLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  if (config.enabled != enabled_)
  {
    enabled_ = config.enabled;
    has_updated_data_ = true;
    x_ = y_ = 0;
    width_ = size_x_;
    height_ = size_y_;
  }
}

void StaticLayer::matchSize()
{
  // If we are using rolling costmap, the static map size is
  //   unrelated to the size of the layered costmap		如果我们正在使用滚动地图，尺寸就和 layered costmap 无关
  if (!layered_costmap_->isRolling())			// 如果不是 rolling，这是 layered_costmap 中的参数，相当于是全局层的参数
  {

// 如果不是滚动窗口，就把当前的 costmap_ 初始化成与 master_grid 相同大小，并赋初值
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
  }
}

unsigned char StaticLayer::interpretValue(unsigned char value)			// 把 0-255 的value变为 0 或 254 或 255
{
  // check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_)			// unknown_cost_value_ = int(-1) -1,unknown_cost_value_ 默认是 true
    return NO_INFORMATION;
  else if (!track_unknown_space_ && value == unknown_cost_value_)
    return FREE_SPACE;
  else if (value >= lethal_threshold_)			// 100
    return LETHAL_OBSTACLE;				// 254
  else if (trinary_costmap_)				// 默认是 true，只会返回3个值中的一个
    return FREE_SPACE;

  double scale = (double) value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

// 当第一张地图达到时还进行了一些初始化的工作
// isRolling 是 true 就更新当前 costmap_ 大小
// isRolling 是 false 就更新LayeredCostmap中所有层大小 
void StaticLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

  ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

  // resize costmap if size, resolution or origin do not match	先 resize 再 赋值
  Costmap2D* master = layered_costmap_->getCostmap();

// 如果不是滚动窗口，判断master map的尺寸是否和获取到的static map一致，如果不一致，则应该修改master map（更新所有地图尺寸）
  if (!layered_costmap_->isRolling() && (master->getSizeInCellsX() != size_x ||		// isRolling 是 false 才行
      master->getSizeInCellsY() != size_y ||
      master->getResolution() != new_map->info.resolution ||
      master->getOriginX() != new_map->info.origin.position.x ||
      master->getOriginY() != new_map->info.origin.position.y ||
      !layered_costmap_->isSizeLocked()))						// isSizeLocked 是 false 则后面括号中一定为 1
  {
    // Update the size of the layered costmap (and all layers, including this one) 更新所有的地图尺寸
    ROS_INFO("Resizing costmap to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x,
                                new_map->info.origin.position.y, true);
  }
//如果本层的数据和订阅到的map尺寸不一致，则更新本层的尺寸, isRolling 是 true
// 更新本层的 Costmap2D 类的参数
  else if (size_x_ != size_x || size_y_ != size_y ||
           resolution_ != new_map->info.resolution ||
           origin_x_ != new_map->info.origin.position.x ||
           origin_y_ != new_map->info.origin.position.y)
  {
    // only update the size of the costmap stored locally in this layer  只更新当前这一个 地图尺寸
    ROS_INFO("Resizing static layer to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    resizeMap(size_x, size_y, new_map->info.resolution,
              new_map->info.origin.position.x, new_map->info.origin.position.y);
  }

  unsigned int index = 0;

// 赋值
  // initialize the costmap with static data
  for (unsigned int i = 0; i < size_y; ++i)
  {
    for (unsigned int j = 0; j < size_x; ++j)
    {
      unsigned char value = new_map->data[index];
      costmap_[index] = interpretValue(value);		// 使用函数进行赋值，变成 3 种 cost 值
      ++index;
    }
  }
  map_frame_ = new_map->header.frame_id;

  // we have a new map, update full size of map
  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  map_received_ = true;
  has_updated_data_ = true;

  // shutdown the map subscrber if firt_map_only_ flag is on  如果是 first_map_only 就只接收一次 map话题
  if (first_map_only_)
  {
    ROS_INFO("Shutting down the map subscriber. first_map_only flag is on");
    map_sub_.shutdown();
  }
}

void StaticLayer::incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update)
{
  unsigned int di = 0;
  for (unsigned int y = 0; y < update->height ; y++)
  {
    unsigned int index_base = (update->y + y) * size_x_;
    for (unsigned int x = 0; x < update->width ; x++)
    {
      unsigned int index = index_base + x + update->x;
      costmap_[index] = interpretValue(update->data[di++]);
    }
  }
  x_ = update->x;
  y_ = update->y;
  width_ = update->width;
  height_ = update->height;
  has_updated_data_ = true;
}

void StaticLayer::activate()
{
  onInitialize();
}

void StaticLayer::deactivate()
{
  map_sub_.shutdown();
  if (subscribe_to_updates_)
    map_update_sub_.shutdown();
}

void StaticLayer::reset()
{
  if (first_map_only_)
  {
    has_updated_data_ = true;
  }
  else
  {
    onInitialize();
  }
}

// 函数 updateBounds 这里设定为整张static map的大小：
void StaticLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                               double* max_x, double* max_y)
// min_x,min_y,max_x,max_y 变成 static map 四个角的世界坐标
{

  if( !layered_costmap_->isRolling() ){					// 如果 isRolling 是 false
    if (!map_received_ || !(has_updated_data_ || has_extra_bounds_))	// 一定要先接收到地图
      return;
  }

//if (!has_extra_bounds_)	return	
  useExtraBounds(min_x, min_y, max_x, max_y);	// 运行函数要先有 has_extra_bounds_

  double wx, wy;

  mapToWorld(x_, y_, wx, wy);			// (0,0)变到世界坐标
  *min_x = std::min(wx, *min_x);		// wx
  *min_y = std::min(wy, *min_y);		// wy

  mapToWorld(x_ + width_, y_ + height_, wx, wy);	// 最右上角也变到世界坐标
  *max_x = std::max(wx, *max_x);		// wx
  *max_y = std::max(wy, *max_y);		// wy

  has_updated_data_ = false;
}

void StaticLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!map_received_)
    return;

  if (!enabled_)		// Layer 中默认是 false
    return;

  if (!layered_costmap_->isRolling())	// 如果 isRolling 是 false
  {
    // if not rolling, the layered costmap (master_grid) has same coordinates as this layer
    // 如果不是滚动窗口 ， layered costmap 和 当前 layer 使用同一个坐标系
    if (!use_maximum_)		// 默认是 false
      updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);		// 使用当前层完全覆盖 master_grid
    else
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);	// 使用两者中的较大值进行覆盖，master_grid 是 255也覆盖，costmap_是 255不操作
  }
  else
  {
    // If rolling window, the master_grid is unlikely to have same coordinates as this layer
// 如果是滚动窗口， master_grid 和 当前层 可能不是同一个坐标系
    unsigned int mx, my;
    double wx, wy;
    // Might even be in a different frame
    tf::StampedTransform transform;
    try
    {
// tf_ 是 Layer 中的成员变量
      tf_->lookupTransform(map_frame_, global_frame_, ros::Time(0), transform);	// 从 map_frame_ 坐标系变换到 global_frame_
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
    }
    // Copy map data given proper transformations
    for (unsigned int i = min_i; i < max_i; ++i)
    {
      for (unsigned int j = min_j; j < max_j; ++j)
      {
        // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
        layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);		// (wx,xy) 是 global_frame_ 下的
        // Transform from global_frame_ to map_frame_
        tf::Point p(wx, wy, 0);							// 变换到 map_frame_ 下
        p = transform(p);
        // Set master_grid with cell from map
        if (worldToMap(p.x(), p.y(), mx, my))
        {
          if (!use_maximum_)							// 如果 use_maximum 是 false 就直接赋值
            master_grid.setCost(i, j, getCost(mx, my));
          else
            master_grid.setCost(i, j, std::max(getCost(mx, my), master_grid.getCost(i, j)));
        }
      }
    }
  }
}

}  // namespace costmap_2d
