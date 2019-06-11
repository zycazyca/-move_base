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
 * Author: TKruse
 *********************************************************************/

#ifndef FOOTPRINT_HELPER_H_
#define FOOTPRINT_HELPER_H_

#include <vector>

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>			// x y z
#include <Eigen/Core>
#include <base_local_planner/Position2DInt.h>		// 比 Point 类型少一个变量
/*
int64 x
int64 y
*/

namespace base_local_planner {

class FootprintHelper {
public:
  FootprintHelper();
  virtual ~FootprintHelper();

  /**
   * @brief  Used to get the cells that make up the footprint of the robot
用于获取构成机器人足迹的单元格
   * @param x_i The x position of the robot
   * @param y_i The y position of the robot
   * @param theta_i The orientation of the robot
   * @param  fill If true: returns all cells in the footprint of the robot. If false: returns only the cells that make up the outline of the footprint.
如果为真：返回机器人足迹中的所有单元。如果为false：仅返回构成示意图轮廓的单元格。
   * @return The cells that make up either the outline or entire footprint of the robot depending on fill
   */
  std::vector<base_local_planner::Position2DInt> getFootprintCells(
      Eigen::Vector3f pos,
      std::vector<geometry_msgs::Point> footprint_spec,
      const costmap_2d::Costmap2D&,
      bool fill);

  /**
   * @brief  Use Bresenham's algorithm to trace a line between two points in a grid
使用 Bresenham 的原理去在网格中的两点间追踪一根线
   * @param  x0 The x coordinate of the first point
   * @param  x1 The x coordinate of the second point
   * @param  y0 The y coordinate of the first point
   * @param  y1 The y coordinate of the second point
   * @param  pts Will be filled with the cells that lie on the line in the grid
pts 将被在线上的单元格填充
   */
  void getLineCells(int x0, int x1, int y0, int y1, std::vector<base_local_planner::Position2DInt>& pts);

  /**
   * @brief Fill the outline of a polygon, in this case the robot footprint, in a grid
在网格中填充多边形的轮廓（在本例中是机器人足迹）
   * @param footprint The list of cells making up the footprint in the grid, will be modified to include all cells inside the footprint
   */
  void getFillCells(std::vector<base_local_planner::Position2DInt>& footprint);
};

} /* namespace base_local_planner */
#endif /* FOOTPRINT_HELPER_H_ */
