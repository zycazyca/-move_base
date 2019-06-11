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
#include <global_planner/gradient_path.h>
#include <algorithm>
#include <stdio.h>
#include <global_planner/planner_core.h>

namespace global_planner {

GradientPath::GradientPath(PotentialCalculator* p_calc) :
        Traceback(p_calc), pathStep_(0.5) {
    gradx_ = grady_ = NULL;
}

GradientPath::~GradientPath() {

    if (gradx_)
        delete[] gradx_;
    if (grady_)
        delete[] grady_;
}

void GradientPath::setSize(int xs, int ys) {
    Traceback::setSize(xs, ys);
    if (gradx_)
        delete[] gradx_;
    if (grady_)
        delete[] grady_;
    gradx_ = new float[xs * ys];
    grady_ = new float[xs * ys];
}

bool GradientPath::getPath(float* potential, double start_x, double start_y, double goal_x, double goal_y, std::vector<std::pair<float, float> >& path) {
    std::pair<float, float> current;
    int stc = getIndex(goal_x, goal_y);		// stc 是目标点，goal_x 和 goal_y 应该会先转换为 int

    // set up offset
    float dx = goal_x - (int)goal_x;		// stc 和目标点间的 x 方向的偏差
    float dy = goal_y - (int)goal_y;		// stc 和目标点间的 y 方向的偏差
    int ns = xs_ * ys_;
    memset(gradx_, 0, ns * sizeof(float));	// gradx_ 全部初始化为 0
    memset(grady_, 0, ns * sizeof(float));	// grady_ 全部初始化为 0

    int c = 0;
    while (c++<ns*4) {				// 限制下路线长度
        // check if near goal
        double nx = stc % xs_ + dx, ny = stc / xs_ + dy;	// 一开始就是 goal_x 和 goal_y

        if (fabs(nx - start_x) < .5 && fabs(ny - start_y) < .5) {	// float 类型的 abs
            current.first = start_x;		// 如果当前点和起始点的坐标距离足够小，就退出
            current.second = start_y;
            path.push_back(current);
            return true;
        }

        if (stc < xs_ || stc > xs_ * ys_ - xs_) // would be out of bounds
        {
            printf("[PathCalc] Out of bounds\n");	// 路线不可以在上边界和下边界，要在两条线中间
            return false;
        }

        current.first = nx;		// nx , ny 都是 float 类型
        current.second = ny;

        //ROS_INFO("%d %d | %f %f ", stc%xs_, stc/xs_, dx, dy);

        path.push_back(current);	// path 里面的路径点都是 float 类型

        bool oscillation_detected = false;	// 振荡检测
        int npath = path.size();
        if (npath > 2 && path[npath - 1].first == path[npath - 3].first		// path 中的最后一个值和倒数第三个值相同，即认为发生了振荡
                && path[npath - 1].second == path[npath - 3].second) {
            ROS_DEBUG("[PathCalc] oscillation detected, attempting fix.");
            oscillation_detected = true;	// 振荡检测设为 true
        }

        int stcnx = stc + xs_;			// 上方的单元格坐标
        int stcpx = stc - xs_;			// 下方的单元格坐标

        // check for potentials at eight positions near cell
	// 如果周围有 potential 是 POT_HIGH 的点，有可能是障碍物，也有可能是之前没有探索过的区域
        if (potential[stc] >= POT_HIGH || potential[stc + 1] >= POT_HIGH || potential[stc - 1] >= POT_HIGH
                || potential[stcnx] >= POT_HIGH || potential[stcnx + 1] >= POT_HIGH || potential[stcnx - 1] >= POT_HIGH
                || potential[stcpx] >= POT_HIGH || potential[stcpx + 1] >= POT_HIGH || potential[stcpx - 1] >= POT_HIGH
                || oscillation_detected) {
            ROS_DEBUG("[Path] Pot fn boundary, following grid (%0.1f/%d)", potential[stc], (int) path.size());
            // check eight neighbors to find the lowest
            int minc = stc;
            int minp = potential[stc];
            int st = stcpx - 1;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            st++;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            st++;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            st = stc - 1;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            st = stc + 1;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            st = stcnx - 1;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            st++;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            st++;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            stc = minc;			// stc 变为周围 9 个单元格中 potential 最小的那个的索引
            dx = 0;			// dx 和 dy 全部变为 0
            dy = 0;

            //ROS_DEBUG("[Path] Pot: %0.1f  pos: %0.1f,%0.1f",
            //    potential[stc], path[npath-1].first, path[npath-1].second);

            if (potential[stc] >= POT_HIGH) {
                ROS_DEBUG("[PathCalc] No path found, high potential");
                //savemap("navfn_highpot");
                return 0;
            }
        }

        // have a good gradient here
        else {				// 周围单元格的 potential 都是良好的，并且不处于振荡状态

            // get grad at four positions near cell
            gradCell(potential, stc);		// 自身 ， 右 ， 上 ， 上右
            gradCell(potential, stc + 1);
            gradCell(potential, stcnx);
            gradCell(potential, stcnx + 1);

            // get interpolated gradient	获取插值梯度
            float x1 = (1.0 - dx) * gradx_[stc] + dx * gradx_[stc + 1];
            float x2 = (1.0 - dx) * gradx_[stcnx] + dx * gradx_[stcnx + 1];
            float x = (1.0 - dy) * x1 + dy * x2; // interpolated x
            float y1 = (1.0 - dx) * grady_[stc] + dx * grady_[stc + 1];
            float y2 = (1.0 - dx) * grady_[stcnx] + dx * grady_[stcnx + 1];
            float y = (1.0 - dy) * y1 + dy * y2; // interpolated y

            // show gradients
            ROS_DEBUG(
                    "[Path] %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f; final x=%.3f, y=%.3f\n", gradx_[stc], grady_[stc], gradx_[stc+1], grady_[stc+1], gradx_[stcnx], grady_[stcnx], gradx_[stcnx+1], grady_[stcnx+1], x, y);

            // check for zero gradient, failed	梯度完全为 0 则故障退出
            if (x == 0.0 && y == 0.0) {
                ROS_DEBUG("[PathCalc] Zero gradient");
                return 0;
            }

            // move in the right direction	一次走半步，x 和 y 方向合起来只有 0.5 个单元格长度
            float ss = pathStep_ / hypot(x, y);
            dx += x * ss;
            dy += y * ss;

            // check for overflow	
            if (dx > 1.0) {
                stc++;
                dx -= 1.0;
            }
            if (dx < -1.0) {
                stc--;
                dx += 1.0;
            }
            if (dy > 1.0) {
                stc += xs_;
                dy -= 1.0;
            }
            if (dy < -1.0) {
                stc -= xs_;
                dy += 1.0;
            }

        }

        //printf("[Path] Pot: %0.1f  grad: %0.1f,%0.1f  pos: %0.1f,%0.1f\n",
        //         potential[stc], dx, dy, path[npath-1].first, path[npath-1].second);
    }

    return false;
}

/*
 int
 NavFn::calcPath(int n, int *st)
 {
 // set up start position at cell
 // st is always upper left corner for 4-point bilinear interpolation
 if (st == NULL) st = start;
 int stc = st[1]*nx + st[0];

 // go for <n> cycles at most
 for (int i=0; i<n; i++)
 {



 }

 //  return npath;            // out of cycles, return failure
 ROS_DEBUG("[PathCalc] No path found, path too long");
 //savemap("navfn_pathlong");
 return 0;            // out of cycles, return failure
 }
 */

//
// gradient calculations			计算梯度
//
// calculate gradient at a cell
// positive value are to the right and down	正值向右和向下
float GradientPath::gradCell(float* potential, int n) {
    if (gradx_[n] + grady_[n] > 0.0)    // check this cell	没看明白这是啥意思
        return 1.0;

    if (n < xs_ || n > xs_ * ys_ - xs_)    // would be out of bounds
        return 0.0;
    float cv = potential[n];
    float dx = 0.0;
    float dy = 0.0;

    // check for in an obstacle
    if (cv >= POT_HIGH) {				// 这里应该不存在这种情况，上面经过检查的
        if (potential[n - 1] < POT_HIGH)
            dx = -lethal_cost_;				// -253
        else if (potential[n + 1] < POT_HIGH)
            dx = lethal_cost_;

        if (potential[n - xs_] < POT_HIGH)
            dy = -lethal_cost_;
        else if (potential[n + xs_] < POT_HIGH)
            dy = lethal_cost_;
    }

    else                // not in an obstacle
    {
        // dx calc, average to sides
        if (potential[n - 1] < POT_HIGH)
            dx += potential[n - 1] - cv;	// 如果比左边的 potential 大，就是 负的，意味着路径向左
        if (potential[n + 1] < POT_HIGH)
            dx += cv - potential[n + 1];	// 如果比右边的 potential 大，就是 正的，意味着路径向右

        // dy calc, average to sides		上下方向同理
        if (potential[n - xs_] < POT_HIGH)
            dy += potential[n - xs_] - cv;
        if (potential[n + xs_] < POT_HIGH)
            dy += cv - potential[n + xs_];
    }

    // normalize
    float norm = hypot(dx, dy);		// 正则化，这是斜边长度
    if (norm > 0) {
        norm = 1.0 / norm;
        gradx_[n] = norm * dx;		// x 方向的梯度 （-1,1）之间  左右
        grady_[n] = norm * dy;		// x 方向的梯度 （1,1）之间  上下
    }
    return norm;
}

} //end namespace global_planner

