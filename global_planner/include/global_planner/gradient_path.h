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
#ifndef _GRADIENT_PATH_H
#define _GRADIENT_PATH_H

#include<global_planner/traceback.h>
#include <math.h>

namespace global_planner {

class GradientPath : public Traceback {
    public:
        GradientPath(PotentialCalculator* p_calc);
        ~GradientPath();

        void setSize(int xs, int ys);

        //
        // Path construction					路径构造器
        // Find gradient at array points, interpolate path	在数组点处查找渐变，插值路径
        // Use step size of pathStep, usually 0.5 pixel		使用PathStep的步长，通常为0.5像素
        //
        // Some sanity checks:					一些健康检查：
        //  1. Stuck at same index position			卡在同一索引位置
        //  2. Doesn't get near goal				不接近目标
        //  3. Surrounded by high potentials			被高potential包围
        //
        bool getPath(float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path);
    private:
        inline int getNearestPoint(int stc, float dx, float dy) {	// stc是当前索引，向右dx，向下dy
            int pt = stc + (int)round(dx) + (int)(xs_ * round(dy));	// round函数四舍五入到最接近的整数
            return std::max(0, std::min(xs_ * ys_ - 1, pt));		// 如果pt没有超出界就返回 pt
        }
        float gradCell(float* potential, int n);

        float *gradx_, *grady_; /**< gradient arrays, size of potential array */

        float pathStep_; 	// 0.5
};

} //end namespace global_planner
#endif
