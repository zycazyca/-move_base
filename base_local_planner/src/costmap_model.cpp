/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/
#include <base_local_planner/line_iterator.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/cost_values.h>

using namespace std;
using namespace costmap_2d;

namespace base_local_planner {
  CostmapModel::CostmapModel(const Costmap2D& ma) : costmap_(ma) {}

// 检查相邻 footprint 的连线是否经过 致命障碍物 或是 未知区域 。如果都是合法的，返回边界中最大可行点的cost，如果不行，返回 -1

  double CostmapModel::footprintCost(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint, 
      double inscribed_radius, double circumscribed_radius){

    //used to put things into grid coordinates  用于将内容放入网格坐标中
    unsigned int cell_x, cell_y;

    //get the cell coord of the center point of the robot   将机器人中心点的世界坐标转换为栅格坐标
    if(!costmap_.worldToMap(position.x, position.y, cell_x, cell_y))
      return -1.0;

    //if number of points in the footprint is less than 3, we'll just assume a circular robot
    // 如果足迹小于 3 个点，我们假设是一个圆形机器人
    if(footprint.size() < 3){
      unsigned char cost = costmap_.getCost(cell_x, cell_y);
      //if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE)
      if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE || cost == NO_INFORMATION)
        return -1.0;
      return cost;	// 返回中心点的 cost 值
    }

    //now we really have to lay down the footprint in the costmap grid
    // 现在我们真的要在Costmap网格中建立足迹
    unsigned int x0, x1, y0, y1;
    double line_cost = 0.0;
    double footprint_cost = 0.0;

    //we need to rasterize each line in the footprint 我们需要将足迹中的每一行光栅化
    for(unsigned int i = 0; i < footprint.size() - 1; ++i){
      //get the cell coord of the first point 获取第一个点的单元格坐标
      if(!costmap_.worldToMap(footprint[i].x, footprint[i].y, x0, y0))
        return -1.0;

      //get the cell coord of the second point 获取第二个点的单元格坐标
      if(!costmap_.worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1))
        return -1.0;

      line_cost = lineCost(x0, x1, y0, y1);
      footprint_cost = std::max(line_cost, footprint_cost);    // footprint_cost 是 line_cost 中的最大值

      //if there is an obstacle that hits the line... we know that we can return false right away 
      // 如果有障碍物撞到了线上…我们知道我们可以马上返回错误
      if(line_cost < 0)
        return -1.0;
    }

    //we also need to connect the first point in the footprint to the last point
    // 我们还需要将示意图中的第一个点连接到最后一个点
    //get the cell coord of the last point
    if(!costmap_.worldToMap(footprint.back().x, footprint.back().y, x0, y0))
      return -1.0;

    //get the cell coord of the first point
    if(!costmap_.worldToMap(footprint.front().x, footprint.front().y, x1, y1))
      return -1.0;

    line_cost = lineCost(x0, x1, y0, y1);
    footprint_cost = std::max(line_cost, footprint_cost);

    if(line_cost < 0)
      return -1.0;

    //if all line costs are legal... then we can return that the footprint is legal
    // 如果所有的生产线的 costs 都是合法的…然后我们可以返回，足迹是合法的
    return footprint_cost;

  }

  //calculate the cost of a ray-traced line 计算光线跟踪线的成本
  double CostmapModel::lineCost(int x0, int x1, int y0, int y1) const {
    double line_cost = 0.0;
    double point_cost = -1.0;

    for( LineIterator line( x0, y0, x1, y1 ); line.isValid(); line.advance() )
    {
      point_cost = pointCost( line.getX(), line.getY() ); //Score the current point 
      // if(cost == LETHAL_OBSTACLE || cost == NO_INFORMATION) 返回 -1 ，否则返回 cost

      if(point_cost < 0)
        return -1;

      if(line_cost < point_cost)
        line_cost = point_cost;
    }

    return line_cost;		// line_cost 是 point_cost 中的最大值
  }

  double CostmapModel::pointCost(int x, int y) const {
    unsigned char cost = costmap_.getCost(x, y);
    //if the cell is in an obstacle the path is invalid
    //if(cost == LETHAL_OBSTACLE){
    if(cost == LETHAL_OBSTACLE || cost == NO_INFORMATION){
      return -1;
    }

    return cost;
  }

};
