/*********************************************************************
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
*********************************************************************/

//
// Navigation function computation
// Uses Dijkstra's method
// Modified for Euclidean-distance computation
//

#ifndef _NAVFN_H
#define _NAVFN_H

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

// cost defs
#define COST_UNKNOWN_ROS 255		// 255 is unknown cost
#define COST_OBS 254		// 254 for forbidden regions
#define COST_OBS_ROS 253	// ROS values of 253 are obstacles

// navfn cost values are set to
// COST_NEUTRAL + COST_FACTOR * costmap_cost_value.
// Incoming costmap cost values are in the range 0 to 252.
// With COST_NEUTRAL of 50, the COST_FACTOR needs to be about 0.8 to
// ensure the input values are spread evenly over the output range, 50
// to 253.  If COST_FACTOR is higher, cost values will have a plateau
// around obstacles and the planner will then treat (for example) the
// whole width of a narrow hallway as equally undesirable and thus
// will not plan paths down the center.

/* navfn的代价值被设置为  COST_NEUTRAL + COST_FACTOR * costmap_cost_value
   输入进来的代价地图代价值在 0~252 之间，把 COST_NEUTRAL 设置为50，并把
   COST_FACTOR设置成0.8,可以确保将输入值均匀分布在 50~253 .
*/
#define COST_NEUTRAL 50		// Set this to "open space" value   设置这个作为开放空间的值
#define COST_FACTOR 0.8		// Used for translating costs in NavFn::setCostmap()   用于在 NavFn::setCostmap() 中转换代价值

// Define the cost type in the case that it is not set. However, this allows
// clients to modify it without changing the file. Arguably, it is better to require it to
// be defined by a user explicitly
// 定义未设置的成本类型。但是，这允许客户机在不更改文件的情况下修改它。可以说，最好要求用户明确定义它。
#ifndef COSTTYPE
#define COSTTYPE unsigned char	// Whatever is used...
#endif

// potential defs
#define POT_HIGH 1.0e10		// unassigned cell potential

// priority buffers
#define PRIORITYBUFSIZE 10000


namespace navfn {
  /**
    Navigation function call.
    \param costmap Cost map array, of type COSTTYPE; origin is upper left   原点在左上角
NOTE: will be modified to have a border of obstacle costs
\param nx Width of map in cells
\param ny Height of map in cells
\param goal X,Y position of goal cell
\param start X,Y position of start cell

Returns length of plan if found, and fills an array with x,y interpolated 
positions at about 1/2 cell resolution; else returns 0.

*/

// 如果找到，返回路径的长度，并用x,y差值填充数组，位置的分辨率约为1/2个单元格，否则返回0
  int create_nav_plan_astar(const COSTTYPE *costmap, int nx, int ny,       // COSTTYPE 是 unsigned char ，nx是单元格中地图宽度，ny是单元格中地图高度
      int* goal, int* start,
      float *plan, int nplan);



  /**
   * @class NavFn
   * @brief Navigation function class. Holds buffers for costmap, navfn map. Maps are pixel-based. Origin is upper left, x is right, y is down. 
   */
  class NavFn
  {
    public:
      /**
       * @brief  Constructs the planner
       * @param nx The x size of the map 
       * @param ny The y size of the map 
       */
      NavFn(int nx, int ny);	// size of map   地图大小

      ~NavFn();

      /**
       * @brief  Sets or resets the size of the map
       * @param nx The x size of the map 
       * @param ny The y size of the map 
       */
      void setNavArr(int nx, int ny); /**< sets or resets the size of the map */
      int nx, ny, ns;		// ns = nx * ny

      /**
       * @brief  Set up the cost array for the planner, usually from ROS    为规划器设置代价数组，通常来自 ROS
       * @param cmap The costmap   代价地图
       * @param isROS Whether or not the costmap is coming in in ROS format    代价地图是否以ROS格式传入
       * @param allow_unknown Whether or not the planner should be allowed to plan through unknown space  是否允许规划器通过未知空间进行计划
       */
      void setCostmap(const COSTTYPE *cmap, bool isROS=true, bool allow_unknown = true); /**< sets up the cost map */


      bool calcNavFnAstar();	 //使用A*启发式算法计算一条路径，如果找到了就返回 true


      bool calcNavFnDijkstra(bool atStart = false);	 //使用Dijkstra计算完整导航


      float *getPathX();		// 路径的x坐标    { return pathx; } 


      float *getPathY();		// 路径的y坐标    { return pathy; } 


      int   getPathLen();		// 获取路径长度，如果没找到就返回 0


      float getLastPathCost();      // 返回上次计算导航函数时找到的路径的开销

      /** 单元格数组 */
      COSTTYPE *costarr;		/**< cost array in 2D configuration space */
      float   *potarr;		/**< potential array, navigation function potential */
      bool    *pending;		/**< pending cells during propagation */   // 传播期间挂起的单元格
      int nobs;			// 障碍物单元数

      /** block priority buffers */
      int *pb1, *pb2, *pb3;		/**< storage buffers for priority blocks */
      int *curP, *nextP, *overP;	/**< priority buffer block ptrs */
      int curPe, nextPe, overPe; /**< end points of arrays */

      /** block priority thresholds */
      float curT;			/**< current threshold */
      float priInc;			/**< priority threshold increment */

      /** goal and start positions */
      /**
       * @brief  Sets the goal position for the planner. Note: the navigation cost field computed gives the cost to get to a given point from the goal, not from the start.
       * @param goal the goal position 
       */
      void setGoal(int *goal);	

      /**
       * @brief  Sets the start position for the planner. Note: the navigation cost field computed gives the cost to get to a given point from the goal, not from the start.
       * @param start the start position 
       */
      void setStart(int *start);	

      int goal[2];
      int start[2];

      void initCost(int k, float v); /**用代价 v 去初始化单元格 k */

      /** 用于测试的简单障碍物 */
      void setObs();               // 源文件中未定义这个函数

      /** 传播 */

      void updateCell(int n);	/**更新索引是 n 的单元格 */

      /**
       * @brief  Updates the cell at index n using the A* heuristic
       * @param n The index to update
       */
      void updateCellAstar(int n);	/**使用A*算法更新索引是 n 的单元格 */

      void setupNavFn(bool keepit = false); /**重置所有用于传播的NAV FN数组 */

      // 运行传播以进行<cycles>迭代，或使用宽度优先的dijkstra方法直到到达start。
      // cycles 是最大迭代次数     atStart 到达起点时是否停止
      bool propNavFnDijkstra(int cycles, bool atStart = false); /**返回true，如果start点被找到或者 full prop */

      bool propNavFnAstar(int cycles); /**如果start点被找到，返回 true */

      /** 梯度和路径 */
      float *gradx, *grady;		/**< gradient arrays, size of potential array */
      float *pathx, *pathy;		/** 路径点，作为亚像素单元坐标 */
      int npath;			/** 路径中点的数量 */
      int npathbuf;			/** PathX、PathY缓冲区的大小 */

      float last_path_cost_; /**保留上次调用A*时找到的路径的开销 */


      /**
       * @brief  Calculates the path for at mose <n> cycles
       * @param n The maximum number of cycles to run for 
       * @return The lenght of the path found
       */
      int calcPath(int n, int *st = NULL); /**< calculates path for at most <n> cycles, returns path length, 0 if none */

      float gradCell(int n);	/**< calculates gradient at cell <n>, returns norm */
      float pathStep;		/**< following梯度的步长 */

      /** display callback */
      void display(void fn(NavFn *nav), int n = 100); /**< <n> 是更新之间的循环数  */
      int displayInt;		/**保存上面display（）的第二个参数 */
      void (*displayFn)(NavFn *nav); /**< display function itself */

      /** save costmap */
      void savemap(const char *fname); /**< write out costmap and start/goal states as fname.pgm and fname.txt */

  };
};


#endif  // NAVFN
