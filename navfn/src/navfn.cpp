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
// 在相邻单元格中，当pot字段为max时，路径计算不使用插值。
// Path calculation uses no interpolation when pot field is at max in
//   nearby cells
// 路径计算已进行健全性检查，检查是否成功
// Path calc has sanity check that it succeeded
//


#include <navfn/navfn.h>
#include <ros/console.h>

namespace navfn {

  //
  // function to perform nav fn calculation
  // keeps track of internal buffers, will be more efficient
  //   if the size of the environment does not change
  //

  int
    create_nav_plan_astar(COSTTYPE *costmap, int nx, int ny,
        int* goal, int* start,
        float *plan, int nplan)
    {
      static NavFn *nav = NULL;

      if (nav == NULL)
        nav = new NavFn(nx,ny);

      if (nav->nx != nx || nav->ny != ny) // check for compatibility with previous call
      {
        delete nav;
        nav = new NavFn(nx,ny);      
      }

      nav->setGoal(goal);		// 这里的 start 和 goal 都是 map 坐标
      nav->setStart(start);

      nav->costarr = costmap;
      nav->setupNavFn(true);          //重置所有用于传播的 NavFn 数组

      // calculate the nav fn and path
      nav->priInc = 2*COST_NEUTRAL;                    // 优先级阈值增量 float   100
      nav->propNavFnAstar(std::max(nx*ny/20,nx+ny));   // 如果start点被找到，返回 true

      // path
      int len = nav->calcPath(nplan);

      if (len > 0)			// found plan
        ROS_DEBUG("[NavFn] Path found, %d steps\n", len);
      else
        ROS_DEBUG("[NavFn] No path found\n");

      if (len > 0)
      {
        for (int i=0; i<len; i++)
        {
          plan[i*2] = nav->pathx[i];
          plan[i*2+1] = nav->pathy[i];       // float *plan中存储着，x1，y1，x2，y2 ……
        }
      }

      return len;
    }




  //
  // 创建 NavFn 缓冲区 
  //

  NavFn::NavFn(int xs, int ys)
  {  
    // create cell arrays
    costarr = NULL;
    potarr = NULL;
    pending = NULL;
    gradx = grady = NULL;
    setNavArr(xs,ys);                          // 在这里为5个数组（costarr，potarr，pending，gradx，grady）开辟空间，并把costarr和pending初始化为0

    // priority buffers
    pb1 = new int[PRIORITYBUFSIZE];            // 10000  块优先级缓冲区
    pb2 = new int[PRIORITYBUFSIZE];
    pb3 = new int[PRIORITYBUFSIZE];

    // for Dijkstra (breadth-first), set to COST_NEUTRAL
    // for A* (best-first), set to COST_NEUTRAL
    priInc = 2*COST_NEUTRAL;	

    // goal and start
    goal[0] = goal[1] = 0;             
    start[0] = start[1] = 0;

    // display function
    displayFn = NULL;        // 这是一个函数指针变量
    displayInt = 0;	     // 更新之间的循环数

    // path buffers
    npathbuf = npath = 0;
    pathx = pathy = NULL;
    pathStep = 0.5;          // following梯度的步长
  }


  NavFn::~NavFn()
  {
    if(costarr)
      delete[] costarr;
    if(potarr)
      delete[] potarr;
    if(pending)
      delete[] pending;
    if(gradx)
      delete[] gradx;
    if(grady)
      delete[] grady;
    if(pathx)
      delete[] pathx;
    if(pathy)                 // 上面7个都不是在构造函数中开辟的空间
      delete[] pathy;
    if(pb1)
      delete[] pb1;
    if(pb2)
      delete[] pb2;
    if(pb3)
      delete[] pb3;
  }


  //
  // set goal, start positions for the nav fn
  //

  void
    NavFn::setGoal(int *g)
    {
      goal[0] = g[0];
      goal[1] = g[1];
      ROS_DEBUG("[NavFn] Setting goal to %d,%d\n", goal[0], goal[1]);
    }

  void
    NavFn::setStart(int *g)
    {
      start[0] = g[0];
      start[1] = g[1];
      ROS_DEBUG("[NavFn] Setting start to %d,%d\n", start[0], start[1]);
    }

  //
  // Set/Reset map size
  // 重置5个数组并开辟空间，costarr、potarr、pending、gradx、grady

  void
    NavFn::setNavArr(int xs, int ys)
    {
      ROS_DEBUG("[NavFn] Array is %d x %d\n", xs, ys);

      nx = xs;
      ny = ys;
      ns = nx*ny;

      if(costarr)
        delete[] costarr;
      if(potarr)
        delete[] potarr;
      if(pending)
        delete[] pending;

      if(gradx)
        delete[] gradx;
      if(grady)
        delete[] grady;

      costarr = new COSTTYPE[ns]; // cost array, 2d config space
      memset(costarr, 0, ns*sizeof(COSTTYPE));                      // C++初始化函数，将一块内存中的内容全部设置为指定的值
      potarr = new float[ns];	// navigation potential array 为什么这个数组不进行初始化？
      pending = new bool[ns];
      memset(pending, 0, ns*sizeof(bool));
      gradx = new float[ns];
      grady = new float[ns];
    }


  //
  // set up cost array, usually from ROS
  // 设置代价数组，通常来自 ROS
  
  // 实际上就是把参数数组通过公式拷贝到成员变量
  void
    NavFn::setCostmap(const COSTTYPE *cmap, bool isROS, bool allow_unknown)
    {
      COSTTYPE *cm = costarr;           // 先把成员变量 costarr 的地址提取出来 
      if (isROS)			// ROS-type cost array
      {
        for (int i=0; i<ny; i++)
        {
          int k=i*nx;                   // k = 当前列标号 * 总行数
          for (int j=0; j<nx; j++, k++, cmap++, cm++)
          {
            // This transforms the incoming cost values:
            // COST_OBS                 -> COST_OBS (incoming "lethal obstacle")  致死障碍物
            // COST_OBS_ROS             -> COST_OBS (incoming "inscribed inflated obstacle") 内接膨胀障碍物
            // values in range 0 to 252 -> values from COST_NEUTRAL to COST_OBS_ROS.
            *cm = COST_OBS;               // 254
            int v = *cmap;
            if (v < COST_OBS_ROS)	  // 如果 v<253 , 就带入公式重算
            {
              v = COST_NEUTRAL+COST_FACTOR*v;
              if (v >= COST_OBS)
                v = COST_OBS-1;           // v = 253
              *cm = v;
            }
            else if(v == COST_UNKNOWN_ROS && allow_unknown)
            {
              v = COST_OBS-1;
              *cm = v;			  // v = 253    把 unknown 也设置为253
            }
          }
        }
      }

      else				// not a ROS map, just a PGM
      {
        for (int i=0; i<ny; i++)
        {
          int k=i*nx;
          for (int j=0; j<nx; j++, k++, cmap++, cm++)
          {
            *cm = COST_OBS;
            if (i<7 || i > ny-8 || j<7 || j > nx-8)
              continue;	// don't do borders
            int v = *cmap;
            if (v < COST_OBS_ROS)
            {
              v = COST_NEUTRAL+COST_FACTOR*v;
              if (v >= COST_OBS)
                v = COST_OBS-1;
              *cm = v;
            }
            else if(v == COST_UNKNOWN_ROS)
            {
              v = COST_OBS-1;
              *cm = v;
            }
          }
        }

      }
    }

  bool
    NavFn::calcNavFnDijkstra(bool atStart)         // 使用 Dijkstra 计算完整导航
    {
#if 0
      static char costmap_filename[1000];
      static int file_number = 0;
      snprintf( costmap_filename, 1000, "navfn-dijkstra-costmap-%04d", file_number++ );
      savemap( costmap_filename );
#endif
      setupNavFn(true);

      // calculate the nav fn and path
      propNavFnDijkstra(std::max(nx*ny/20,nx+ny),atStart);

      // path
      int len = calcPath(nx*ny/2);

      if (len > 0)			// found plan
      {
        ROS_DEBUG("[NavFn] Path found, %d steps\n", len);
        return true;
      }
      else
      {
        ROS_DEBUG("[NavFn] No path found\n");
        return false;
      }

    }


  //
  // calculate navigation function, given a costmap, goal, and start
  //

  bool
    NavFn::calcNavFnAstar()
    {
      setupNavFn(true);

      // calculate the nav fn and path
      propNavFnAstar(std::max(nx*ny/20,nx+ny));

      // path
      int len = calcPath(nx*4);

      if (len > 0)			// found plan
      {
        ROS_DEBUG("[NavFn] Path found, %d steps\n", len);
        return true;
      }
      else
      {
        ROS_DEBUG("[NavFn] No path found\n");
        return false;
      }
    }


  //
  // returning values
  //

  float *NavFn::getPathX() { return pathx; }        // float *pathx, *pathy;
  float *NavFn::getPathY() { return pathy; }
  int    NavFn::getPathLen() { return npath; }      // 路径中点的数量


  //
  // simple obstacle setup for tests
  //

  void
    NavFn::setObs()                              // 手动在地图上设置障碍物，需要搞清楚costmap的存储顺序，x和y对应什么方向
    {
#if 0
      // set up a simple obstacle
      ROS_INFO("[NavFn] Setting simple obstacle\n");
      int xx = nx/3;
      for (int i=ny/3; i<ny; i++)
        costarr[i*nx + xx] = COST_OBS;
      xx = 2*nx/3;
      for (int i=ny/3; i<ny; i++)
        costarr[i*nx + xx] = COST_OBS;

      xx = nx/4;
      for (int i=20; i<ny-ny/3; i++)
        costarr[i*nx + xx] = COST_OBS;
      xx = nx/2;
      for (int i=20; i<ny-ny/3; i++)
        costarr[i*nx + xx] = COST_OBS;
      xx = 3*nx/4;
      for (int i=20; i<ny-ny/3; i++)
        costarr[i*nx + xx] = COST_OBS;
#endif
    }


  // inserting onto the priority blocks   插入优先级块
#define push_cur(n)  { if (n>=0 && n<ns && !pending[n] && \
    costarr[n]<COST_OBS && curPe<PRIORITYBUFSIZE) \
  { curP[curPe++]=n; pending[n]=true; }}
#define push_next(n) { if (n>=0 && n<ns && !pending[n] && \
    costarr[n]<COST_OBS && nextPe<PRIORITYBUFSIZE) \
  { nextP[nextPe++]=n; pending[n]=true; }}
#define push_over(n) { if (n>=0 && n<ns && !pending[n] && \
    costarr[n]<COST_OBS && overPe<PRIORITYBUFSIZE) \
  { overP[overPe++]=n; pending[n]=true; }}


  // Set up navigation potential arrays for new propagation
  // 为新的传播设置导航势能数组
  /* 函数的具体作用：
1、把每一个单元格的 potarr 设置成一个很大的值，gradx 和 grady 都设置为 0
2、如果是 false 就重置 costarr 数组，都变成 50 ，如果是 true 就不变
3、把地图外边界在代价数组（costarr）中对应的值都设置为 254
4、把 curP,nextP,overP初始化为 pb1,pb2,pb3 ,把 curPe,nextPe,overPe 初始化为0， pending 数组初始化为0
5、把 goal 的 potarr[k] 设置为 0 ,往 curP 中放了4个数
6、计算 nobs
*/
  void
    NavFn::setupNavFn(bool keepit)                  // 默认是 false ，一般设置为 true ，重置所有用于传播的 NavFn 数组
    {
      // reset values in propagation arrays
      for (int i=0; i<ns; i++)                      // 对于每一个单元格
      {
        potarr[i] = POT_HIGH;                       // 1.0e10，把 potarr 设为一个很大的值	
        if (!keepit) costarr[i] = COST_NEUTRAL;	    // 如果是 false 就重置 costarr 数组，都变成 50
        gradx[i] = grady[i] = 0.0;		    // x 和 y 的梯度数组都设置为0
      }

      // outer bounds of cost array   代价数组的外边界
      COSTTYPE *pc;
      pc = costarr;
      for (int i=0; i<nx; i++)               // 把 costarr 边界都设置成 254 
        *pc++ = COST_OBS;                    // 地图的上边界
      pc = costarr + (ny-1)*nx;
      for (int i=0; i<nx; i++)
        *pc++ = COST_OBS;		     // 地图的下边界
      pc = costarr;
      for (int i=0; i<ny; i++, pc+=nx)
        *pc = COST_OBS;			     // 地图左边界
      pc = costarr + nx - 1;
      for (int i=0; i<ny; i++, pc+=nx)
        *pc = COST_OBS;			     // 地图右边界

      // priority buffers
      curT = COST_OBS;                      // curT 是当前阈值，设置为254
      curP = pb1; 			    // pb1,pb2,pb3 在构造函数中被分配了 int[10000] 大小的空间
      curPe = 0;
      nextP = pb2;
      nextPe = 0;
      overP = pb3;
      overPe = 0;
      memset(pending, 0, ns*sizeof(bool));   // 初始化 pending 数组全部为 0

      // set goal  把 goal 的 potarr[k] 设置为0
      int k = goal[0] + goal[1]*nx;	     // 用 行编号+列编号×总行数
      initCost(k,0);                         // 用代价 0 去初始化单元格 k , potarr[k] = 0

      // find # of obstacle cells   计算障碍物占据的单元格数量
      pc = costarr;
      int ntot = 0;
      for (int i=0; i<ns; i++, pc++)
      {
        if (*pc >= COST_OBS)
          ntot++;			// number of cells that are obstacles
      }
      nobs = ntot;
    }


  // initialize a goal-type cost for starting propagation   初始化开始传播的目标类型代价

  void
    NavFn::initCost(int k, float v)
    {
      potarr[k] = v;
      push_cur(k+1);     // { curP[curPe++]=n; pending[n]=true; } 依次添加 curP ,并且把上下左右四个单元格的pending设置为true
      push_cur(k-1);
      push_cur(k-nx);
      push_cur(k+nx);
    }


  // 
  // Critical function: calculate updated potential value of a cell,
  //   given its neighbors' values
  // Planar-wave update calculation from two lowest neighbors in a 4-grid
  // Quadratic approximation to the interpolated value    插值的二次近似
  // No checking of bounds here, this function should be fast    这里不检查边界，这个函数应该很快
  // 临界函数：考虑到相邻单元格的值，计算一个单元的更新势能，4网格中两个最低邻域的平面波更新计算

#define INVSQRT2 0.707106781      // 二分之根号二

  inline void
    NavFn::updateCell(int n)       // 更新索引是 n 的单元格
    {
      // get neighbors
      float u,d,l,r;
      l = potarr[n-1];         // left    n-1
      r = potarr[n+1];	       // right   n+1
      u = potarr[n-nx];	       // up      n-nx       nx 代表一行有多少个，地图宽度
      d = potarr[n+nx];	       // down    n+nx		
      //  ROS_INFO("[Update] c: %0.1f  l: %0.1f  r: %0.1f  u: %0.1f  d: %0.1f\n", 
      //	 potarr[n], l, r, u, d);
      //  ROS_INFO("[Update] cost: %d\n", costarr[n]);

      // find lowest, and its lowest neighbor
      float ta, tc;
      if (l<r) tc=l; else tc=r;        // tc是左右中较小的
      if (u<d) ta=u; else ta=d;	       // ta是上下中较小的

      // do planar wave update
      if (costarr[n] < COST_OBS)	// 不要传播到障碍物中，其实这个判断正常情况下用不到，因为在加入到数组中时已经判断过一次是不是障碍物
      {
        float hf = (float)costarr[n]; // traversability factor  可穿越系数
        float dc = tc-ta;		// relative cost between ta,tc
        if (dc < 0) 		// 把四个格子中的最小值存进 ta
        {
          dc = -dc;
          ta = tc;
        }

        // 计算新的势能
        float pot;
        if (dc >= hf)		// if too large, use ta-only update
          pot = ta+hf;
        else			// two-neighbor interpolation update
        {
          // use quadratic approximation   使用二次近似
          // might speed this up through table lookup, but still have to 
          //   do the divide   可能通过查找表加快速度，但仍然必须进行划分
          float d = dc/hf;
          float v = -0.2301*d*d + 0.5307*d + 0.7040;     // 这个式子是什么意思？？ 好像近似为 二分之根号二
          pot = ta + hf*v;
        }

        //      ROS_INFO("[Update] new pot: %d\n", costarr[n]);

        // now add affected neighbors to priority blocks
        if (pot < potarr[n])                             // 如果 pot 比原来的 potarr[n] 要小 ，更新 potarr[n]
        {
          float le = INVSQRT2*(float)costarr[n-1];
          float re = INVSQRT2*(float)costarr[n+1];
          float ue = INVSQRT2*(float)costarr[n-nx];
          float de = INVSQRT2*(float)costarr[n+nx];
          potarr[n] = pot;
          if (pot < curT)	// low-cost buffer block   低代价缓冲块   如果 pot 比阈值要小，一开始是 254
          {
            if (l > pot+le) push_next(n-1);           // 只有 panding = 0 才会被放入
            if (r > pot+re) push_next(n+1);
            if (u > pot+ue) push_next(n-nx);
            if (d > pot+de) push_next(n+nx);
          }
          else			// overflow block
          {
            if (l > pot+le) push_over(n-1);
            if (r > pot+re) push_over(n+1);
            if (u > pot+ue) push_over(n-nx);
            if (d > pot+de) push_over(n+nx);
          }
        }

      }

    }


  //
  // Use A* method for setting priorities    使用 A* 方法设置优先级
  // Critical function: calculate updated potential value of a cell,   临界函数：根据相邻单元的值，计算单元的更新势能值。
  //   given its neighbors' values
  // Planar-wave update calculation from two lowest neighbors in a 4-grid
  // Quadratic approximation to the interpolated value 
  // No checking of bounds here, this function should be fast
  //

#define INVSQRT2 0.707106781

  inline void
    NavFn::updateCellAstar(int n)
    {
      // get neighbors
      float u,d,l,r;
      l = potarr[n-1];
      r = potarr[n+1];		
      u = potarr[n-nx];
      d = potarr[n+nx];
      //ROS_INFO("[Update] c: %0.1f  l: %0.1f  r: %0.1f  u: %0.1f  d: %0.1f\n", 
      //	 potarr[n], l, r, u, d);
      // ROS_INFO("[Update] cost of %d: %d\n", n, costarr[n]);

      // find lowest, and its lowest neighbor
      float ta, tc;
      if (l<r) tc=l; else tc=r;
      if (u<d) ta=u; else ta=d;

      // do planar wave update
      if (costarr[n] < COST_OBS)	// don't propagate into obstacles
      {
        float hf = (float)costarr[n]; // traversability factor
        float dc = tc-ta;		// relative cost between ta,tc
        if (dc < 0) 		// ta is lowest
        {
          dc = -dc;
          ta = tc;
        }

        // calculate new potential
        float pot;
        if (dc >= hf)		// if too large, use ta-only update
          pot = ta+hf;
        else			// two-neighbor interpolation update
        {
          // use quadratic approximation
          // might speed this up through table lookup, but still have to 
          //   do the divide
          float d = dc/hf;
          float v = -0.2301*d*d + 0.5307*d + 0.7040;     // 大约在 0.7 到 1 之间
          pot = ta + hf*v;
        }

        //ROS_INFO("[Update] new pot: %d\n", costarr[n]);

        // now add affected neighbors to priority blocks
        if (pot < potarr[n])
        {
          float le = INVSQRT2*(float)costarr[n-1];
          float re = INVSQRT2*(float)costarr[n+1];
          float ue = INVSQRT2*(float)costarr[n-nx];
          float de = INVSQRT2*(float)costarr[n+nx];

          // calculate distance   计算到目标点的距离，底下五行中除了第四行都是多出来的
          int x = n%nx;
          int y = n/nx;
          float dist = hypot(x-start[0], y-start[1])*(float)COST_NEUTRAL;

          potarr[n] = pot;      
          pot += dist;		// pot = dist + pot     并不把 dist 加到更新的 potarr 中，只是作为选择待检测单元格的依据
          if (pot < curT)	// low-cost buffer block 
          {
            if (l > pot+le) push_next(n-1);      // { nextP[nextPe++]=n; pending[n]=true; }
            if (r > pot+re) push_next(n+1);
            if (u > pot+ue) push_next(n-nx);
            if (d > pot+de) push_next(n+nx);
          }
          else
          {
            if (l > pot+le) push_over(n-1);
            if (r > pot+re) push_over(n+1);
            if (u > pot+ue) push_over(n-nx);
            if (d > pot+de) push_over(n+nx);
          }
        }

      }

    }



  //
  // main propagation function                   主传播函数
  // Dijkstra method, breadth-first              Dijkstra 方法，广度优先
  // runs for a specified number of cycles,
  //   or until it runs out of cells to update,
  //   or until the Start cell is found (atStart = true)   
  //  运行指定的周期数，或者直到要更新的单元格用完，或者直到找到开始单元格（atstart=true）

  bool
    NavFn::propNavFnDijkstra(int cycles, bool atStart)	
    {
      int nwv = 0;			// max priority block size  最大优先级块大小
      int nc = 0;			// number of cells put into priority blocks  放入优先级块的单元格数
      int cycle = 0;		// which cycle we're on   我们在哪个周期

      // set up start cell  设置开始单元格
      int startCell = start[1]*nx + start[0];

      for (; cycle < cycles; cycle++) // go for this many cycles, unless interrupted
      {
        // 
        if (curPe == 0 && nextPe == 0) // priority blocks empty
          break;

        // stats
        nc += curPe;                   // 放入优先级块的单元格数   nc = nc+curPe
        if (curPe > nwv)
          nwv = curPe;

        // reset pending flags on current priority buffer
        int *pb = curP;
        int i = curPe;			
        while (i-- > 0)		
          pending[*(pb++)] = false;		// 这边为什么要把 pending 改成 false 。为了可以将列表中的单元格再次加入 nextP

        // process current priority buffer
        pb = curP; 
        i = curPe;
        while (i-- > 0)		
          updateCell(*pb++);

        if (displayInt > 0 &&  (cycle % displayInt) == 0)
          displayFn(this);

        // swap priority blocks curP <=> nextP
        curPe = nextPe;
        nextPe = 0;
        pb = curP;		// swap buffers
        curP = nextP;
        nextP = pb;

        // see if we're done with this priority level
        if (curPe == 0)
        {
          curT += priInc;	// increment priority threshold
          curPe = overPe;	// set current to overflow block
          overPe = 0;
          pb = curP;		// swap buffers
          curP = overP;
          overP = pb;
        }

        // check if we've hit the Start cell
        if (atStart)
          if (potarr[startCell] < POT_HIGH)
            break;
      }

      ROS_DEBUG("[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n", 
          cycle,nc,(int)((nc*100.0)/(ns-nobs)),nwv);

      if (cycle < cycles) return true; // finished up here
      else return false;
    }


  //
  // main propagation function
  // A* method, best-first
  // uses Euclidean distance heuristic
  // runs for a specified number of cycles,
  //   or until it runs out of cells to update,
  //   or until the Start cell is found (atStart = true)
  //

  bool
    NavFn::propNavFnAstar(int cycles)	
    {
      int nwv = 0;			// max priority block size      优先级块的最大尺寸
      int nc = 0;			// number of cells put into priority blocks  放进优先级块中的单元格数量
      int cycle = 0;		// which cycle we're on

      // set initial threshold, based on distance
      float dist = hypot(goal[0]-start[0], goal[1]-start[1])*(float)COST_NEUTRAL;    // 给定两个直角边，求斜边的长度  *50
      curT = dist + curT;                         // 当前的阈值   curT = dist + 254

      // set up start cell
      int startCell = start[1]*nx + start[0];

      // do main cycle
      for (; cycle < cycles; cycle++) // go for this many cycles, unless interrupted
      {
        // 
        if (curPe == 0 && nextPe == 0) // priority blocks empty
          break;

        // stats
        nc += curPe;				// nc = nc+curPe  记录一共处理（更新）了多少个单元格
        if (curPe > nwv)
          nwv = curPe;				// 记录优先级块的最大尺寸，方便之后输出

        // reset pending flags on current priority buffer   把当前 curP 中的 pending 都置为 0
        int *pb = curP;
        int i = curPe;			
        while (i-- > 0)		
          pending[*(pb++)] = false;

        // process current priority buffer		处理当前的优先级块  curP
        pb = curP; 
        i = curPe;
        while (i-- > 0)		
          updateCellAstar(*pb++);			// updateCellAstar   从头开始正向拿出

        if (displayInt > 0 &&  (cycle % displayInt) == 0)
          displayFn(this);

        // swap priority blocks curP <=> nextP
        curPe = nextPe;
        nextPe = 0;
        pb = curP;		// swap buffers
        curP = nextP;
        nextP = pb;

        // see if we're done with this priority level
        if (curPe == 0)
        {
          curT += priInc;	// increment priority threshold    提高优先级阈值
          curPe = overPe;	// set current to overflow block
          overPe = 0;
          pb = curP;		// swap buffers
          curP = overP;
          overP = pb;
        }

        // check if we've hit the Start cell
        if (potarr[startCell] < POT_HIGH)
          break;

      }

      last_path_cost_ = potarr[startCell];       // 路径 cost 直接读取 potarr[startCell] 就完事了

      ROS_DEBUG("[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n", 
          cycle,nc,(int)((nc*100.0)/(ns-nobs)),nwv);


      if (potarr[startCell] < POT_HIGH) return true; // finished up here
      else return false;
    }


  float NavFn::getLastPathCost()
  {
    return last_path_cost_;
  }


  //
  // Path construction		构造 pathx , pathy, npath
  // Find gradient at array points, interpolate path   在数组点处查找渐变，插值路径
  // Use step size of pathStep, usually 0.5 pixel      使用PathStep的步长，通常为0.5像素
  //
  // Some sanity checks:				一些健康检查：
  //  1. Stuck at same index position			卡在同一索引位置
  //  2. Doesn't get near goal				不接近目标
  //  3. Surrounded by high potentials			被高势能包围
  //

  int
    NavFn::calcPath(int n, int *st)
    {
      // test write
      //savemap("test");

      // check path arrays
      if (npathbuf < n)                     // npathbuf 一开始 = 0
      {
        if (pathx) delete [] pathx;
        if (pathy) delete [] pathy;
        pathx = new float[n];
        pathy = new float[n];
        npathbuf = n;
      }

      // set up start position at cell     在单元格处设置开始位置
      // st is always upper left corner for 4-point bilinear interpolation     对于4点双线性插值，st总是左上角
      if (st == NULL) st = start;
      int stc = st[1]*nx + st[0];     // stc 是开始点坐标

      // set up offset   设置偏移量
      float dx=0;
      float dy=0;
      npath = 0;

      // go for <n> cycles at most   最多进行<n>个循环
      for (int i=0; i<n; i++)
      {
        // check if near goal   先判断有没有到达目标点
        int nearest_point=std::max(0,std::min(nx*ny-1,stc+(int)round(dx)+(int)(nx*round(dy))));
        if (potarr[nearest_point] < COST_NEUTRAL)
        {
          pathx[npath] = (float)goal[0];
          pathy[npath] = (float)goal[1];
          return ++npath;	// done!
        }

        if (stc < nx || stc > ns-nx) // would be out of bounds   表明在边界之外
        {
          ROS_DEBUG("[PathCalc] Out of bounds");
          return 0;
        }

        // add to path    把上一个循环中计算得到的 stc 插入到 pathx 和 pathy
        pathx[npath] = stc%nx + dx;      // 梯度是有很多种可能的，个人感觉这边不需要加上 dx 和 dy
        pathy[npath] = stc/nx + dy;
        npath++;

        bool oscillation_detected = false;
        if( npath > 2 &&
            pathx[npath-1] == pathx[npath-3] &&                          // 路径中的上个点和上上上个点位置一样，则认为检测到摩擦
            pathy[npath-1] == pathy[npath-3] )
        {
          ROS_DEBUG("[PathCalc] oscillation detected, attempting fix.");
          oscillation_detected = true;
        }

        int stcnx = stc+nx;       // 下边的点
        int stcpx = stc-nx;	  // 上边的点

        // check for potentials at eight positions near cell
        if (potarr[stc] >= POT_HIGH ||                            // 如果自身或四周有障碍物或者检测到摩擦， stc变成周围八个中 potarr 较小的单元格
            potarr[stc+1] >= POT_HIGH ||
            potarr[stc-1] >= POT_HIGH ||
            potarr[stcnx] >= POT_HIGH ||
            potarr[stcnx+1] >= POT_HIGH ||
            potarr[stcnx-1] >= POT_HIGH ||
            potarr[stcpx] >= POT_HIGH ||
            potarr[stcpx+1] >= POT_HIGH ||
            potarr[stcpx-1] >= POT_HIGH ||
            oscillation_detected)
        {
          ROS_DEBUG("[Path] Pot fn boundary, following grid (%0.1f/%d)", potarr[stc], npath);
          // check eight neighbors to find the lowest
          int minc = stc;				// 用于记录相连的八个单元格中较小的那个格子
          int minp = potarr[stc];			// 用于记录较小的 arrpot 的值
          int st = stcpx - 1;           // 左上角的点
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          st++;
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          st++;
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          st = stc-1;
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          st = stc+1;
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          st = stcnx-1;
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          st++;
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          st++;
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          stc = minc;    // stc 变成 potential 较小的单元格
          dx = 0;
          dy = 0;

          ROS_DEBUG("[Path] Pot: %0.1f  pos: %0.1f,%0.1f",
              potarr[stc], pathx[npath-1], pathy[npath-1]);

          if (potarr[stc] >= POT_HIGH)     // 周围八个点全是墙壁
          {
            ROS_DEBUG("[PathCalc] No path found, high potential");
            //savemap("navfn_highpot");
            return 0;
          }
        }

        // have a good gradient here   在这里有较好的梯度
        else			
        {

          // get grad at four positions near cell   获取单元格附近的四个位置的梯度，自己、右、下、下右
          gradCell(stc);
          gradCell(stc+1);
          gradCell(stcnx);
          gradCell(stcnx+1);


          // get interpolated gradient    获取插值梯度
          float x1 = (1.0-dx)*gradx[stc] + dx*gradx[stc+1];
          float x2 = (1.0-dx)*gradx[stcnx] + dx*gradx[stcnx+1];   // 下边的点
          float x = (1.0-dy)*x1 + dy*x2; // interpolated x
          float y1 = (1.0-dx)*grady[stc] + dx*grady[stc+1];
          float y2 = (1.0-dx)*grady[stcnx] + dx*grady[stcnx+1];
          float y = (1.0-dy)*y1 + dy*y2; // interpolated y

          // show gradients
          ROS_DEBUG("[Path] %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f; final x=%.3f, y=%.3f\n",
                    gradx[stc], grady[stc], gradx[stc+1], grady[stc+1], 
                    gradx[stcnx], grady[stcnx], gradx[stcnx+1], grady[stcnx+1],
                    x, y);

          // check for zero gradient, failed
          if (x == 0.0 && y == 0.0)
          {
            ROS_DEBUG("[PathCalc] Zero gradient");	  
            return 0;
          }

          // move in the right direction
          float ss = pathStep/hypot(x, y);
          dx += x*ss;
          dy += y*ss;

          // check for overflow    决定往上下左右哪个方向走
          if (dx > 1.0) { stc++; dx -= 1.0; }
          if (dx < -1.0) { stc--; dx += 1.0; }
          if (dy > 1.0) { stc+=nx; dy -= 1.0; }
          if (dy < -1.0) { stc-=nx; dy += 1.0; }

        }

        //      ROS_INFO("[Path] Pot: %0.1f  grad: %0.1f,%0.1f  pos: %0.1f,%0.1f\n",
        //	     potarr[stc], x, y, pathx[npath-1], pathy[npath-1]);
      }

      //  return npath;			// out of cycles, return failure
      ROS_DEBUG("[PathCalc] No path found, path too long");
      //savemap("navfn_pathlong");
      return 0;			// out of cycles, return failure
    }


  //
  // gradient calculations  计算梯度
  //

  // calculate gradient at a cell
  // positive value are to the right and down
  float				
    NavFn::gradCell(int n)
    {
      if (gradx[n]+grady[n] > 0.0)	// check this cell
        return 1.0;			

      if (n < nx || n > ns-nx)	// would be out of bounds
        return 0.0;

      float cv = potarr[n];
      float dx = 0.0;
      float dy = 0.0;

      // check for in an obstacle
      if (cv >= POT_HIGH) 
      {
        if (potarr[n-1] < POT_HIGH)
          dx = -COST_OBS;
        else if (potarr[n+1] < POT_HIGH)
          dx = COST_OBS;

        if (potarr[n-nx] < POT_HIGH)
          dy = -COST_OBS;
        else if (potarr[n+nx] < POT_HIGH)
          dy = COST_OBS;
      }

      else				// not in an obstacle
      {
        // dx calc, average to sides
        if (potarr[n-1] < POT_HIGH)
          dx += potarr[n-1]- cv;	// 梯度计算用前面的 - 后面的
        if (potarr[n+1] < POT_HIGH)
          dx += cv - potarr[n+1]; 

        // dy calc, average to sides
        if (potarr[n-nx] < POT_HIGH)
          dy += potarr[n-nx]- cv;	
        if (potarr[n+nx] < POT_HIGH)
          dy += cv - potarr[n+nx]; 
      }

      // normalize
      float norm = hypot(dx, dy);    // 求斜边长度
      if (norm > 0)
      {
        norm = 1.0/norm;
        gradx[n] = norm*dx;	     // 归一化后的梯度
        grady[n] = norm*dy;
      }
      return norm;
    }


  //
  // display function setup
  // <n> is the number of cycles to wait before displaying,
  //     use 0 to turn it off

  void
    NavFn::display(void fn(NavFn *nav), int n)
    {
      displayFn = fn;
      displayInt = n;
    }


  //
  // debug writes
  // saves costmap and start/goal
  //

  void 
    NavFn::savemap(const char *fname)
    {
      char fn[4096];

      ROS_DEBUG("[NavFn] Saving costmap and start/goal points");
      // write start and goal points
      sprintf(fn,"%s.txt",fname);
      FILE *fp = fopen(fn,"w");
      if (!fp)
      {
        ROS_WARN("Can't open file %s", fn);
        return;
      }
      fprintf(fp,"Goal: %d %d\nStart: %d %d\n",goal[0],goal[1],start[0],start[1]);
      fclose(fp);

      // write cost array
      if (!costarr) return;
      sprintf(fn,"%s.pgm",fname);
      fp = fopen(fn,"wb");
      if (!fp)
      {
        ROS_WARN("Can't open file %s", fn);
        return;
      }
      fprintf(fp,"P5\n%d\n%d\n%d\n", nx, ny, 0xff);
      fwrite(costarr,1,nx*ny,fp);
      fclose(fp);
    }
};
