/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/footprint.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>

using std::vector;

namespace costmap_2d
{

LayeredCostmap::LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown) :
    costmap_(),
    global_frame_(global_frame),
    rolling_window_(rolling_window),
    current_(false),
    minx_(0.0),
    miny_(0.0),
    maxx_(0.0),
    maxy_(0.0),
    bx0_(0),
    bxn_(0),
    by0_(0),
    byn_(0),
    initialized_(false),
    size_locked_(false),
    circumscribed_radius_(1.0),
    inscribed_radius_(0.1)
{
  if (track_unknown)			// track_unknown 决定了 costmap 的默认值是 255还是0
    costmap_.setDefaultValue(255);	// 设置 Costmap2D 类保护成员变量 default_value_ 的值
  else
    costmap_.setDefaultValue(0);	// 设置 Costmap2D 类保护成员变量 default_value_ 的值
}

LayeredCostmap::~LayeredCostmap()
{
  while (plugins_.size() > 0)
  {
    plugins_.pop_back();
  }
}

/*
给class costmap_2d 的 costmap_ 成员的大小重新做分配。然后根据plugin对每一层的地图调用其父类Costmap2D成员的initial 方法，实际效果就是将plugin所指向的每一层地图的大小都设置为和LayeredCostmap::costmap_ 数据成员一样的空间大小。
*/
void LayeredCostmap::resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
                               double origin_y, bool size_locked)
{
  boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));
  size_locked_ = size_locked;
  costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);	// 先把自己的大小给调整了
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    (*plugin)->matchSize();		// 调用 Layer 派生类（CostmapLayer）的函数 matchSize(),使得每一个 plugin中的 Costmap2D 类型成员参数 和 LayeredCostmap 中 costmap_ 一样
  }
}

// 最主要的函数
/*
完成对每一层地图的更新，更新过程分为两步updateBounds和updateCosts
*/
// 如果是滚动窗口，会改变 master_grid 的原点位置，使机器人位于 master_grid 正中间
void LayeredCostmap::updateMap(double robot_x, double robot_y, double robot_yaw)	// 这里的参数都是世界坐标
{
  // Lock for the remainder of this function, some plugins (e.g. VoxelLayer)
  // implement thread unsafe updateBounds() functions.
  boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));		// 改变地图 cost 值的时候加锁

  // if we're using a rolling buffer costmap... we need to update the origin using the robot's position
  if (rolling_window_)
  {
    double new_origin_x = robot_x - costmap_.getSizeInMetersX() / 2;	// getSizeInMetersX()是从最左的边到最后一个单元格中心的距离
    double new_origin_y = robot_y - costmap_.getSizeInMetersY() / 2;
    costmap_.updateOrigin(new_origin_x, new_origin_y);			// 使机器人在新的 costmap 的中间，之前未知的区域现在都使用默认值初始化 
  }

  if (plugins_.size() == 0)
    return;

  minx_ = miny_ = 1e30;
  maxx_ = maxy_ = -1e30;

// 对于 plugins_ 中的每一个 Layer
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    double prev_minx = minx_;
    double prev_miny = miny_;
    double prev_maxx = maxx_;
    double prev_maxy = maxy_;
// 调用Layer的updateBounds 函数
    (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);	
    if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy)
    {
      ROS_WARN_THROTTLE(1.0, "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
                        "is now [tl: (%f, %f), br: (%f, %f)]. The offending layer is %s",
                        prev_minx, prev_miny, prev_maxx , prev_maxy,
                        minx_, miny_, maxx_ , maxy_,
                        (*plugin)->getName().c_str());
    }
  }

//强制转换成 map 边界内的坐标，如果世界坐标越界就转换为 map 最边界的坐标
  int x0, xn, y0, yn;
  costmap_.worldToMapEnforceBounds(minx_, miny_, x0, y0);	
  costmap_.worldToMapEnforceBounds(maxx_, maxy_, xn, yn);

  x0 = std::max(0, x0);							// 保证得到的map坐标 x0 xn y0 yn 是可行的
  xn = std::min(int(costmap_.getSizeInCellsX()), xn + 1);
  y0 = std::max(0, y0);
  yn = std::min(int(costmap_.getSizeInCellsY()), yn + 1);

  ROS_DEBUG("Updating area x: [%d, %d] y: [%d, %d]", x0, xn, y0, yn);

  if (xn < x0 || yn < y0)
    return;

  costmap_.resetMap(x0, y0, xn, yn);	// 把这个区域内的值赋成 default_value_
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
// 对于 static map 就是直接把那一层的值赋过来 ， 每一层的排列顺序很重要 ， 第一层必须是 static_map
    (*plugin)->updateCosts(costmap_, x0, y0, xn, yn);
  }

  bx0_ = x0;		// 这是更新地图区域的并集，map 坐标
  bxn_ = xn;
  by0_ = y0;
  byn_ = yn;

  initialized_ = true;
}

// 对操作的实时性提供保证，提供是否发生超时的信息。
bool LayeredCostmap::isCurrent()
{
  current_ = true;
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    current_ = current_ && (*plugin)->isCurrent();	// plugins_ 中所有 Layer 的 current_ 都是 true 才可以
  }
  return current_;
}

void LayeredCostmap::setFootprint(const std::vector<geometry_msgs::Point>& footprint_spec)
{
  footprint_ = footprint_spec;
  costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius_, circumscribed_radius_);	// 重新计算内切圆和外接圆半径，是命名空间中的函数，不是某一个类的函数，机器人中心点默认为（0,0），计算中心点到顶点和边的最短距离和最远距离

// 对于 plugins_ 中的每一个 Layer ，改变 footprint
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    (*plugin)->onFootprintChanged();
  }
}

}  // namespace costmap_2d
