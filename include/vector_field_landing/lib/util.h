/*********************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Woohyun Byun.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ********************************************************************************/
/**
 * @file util.h 
 * This file is used to convert the ROS message types to Eigen types.
 * 
 * @author Woohyun Byun - imbwh@cau.ac.kr
 * 
 * @date 2024.10.28
 */ 

#ifndef __UTIL_L__
#define __UTIL_L__

#include <iostream>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

using namespace std;
using namespace Eigen;

namespace aims_fly
{

inline Vector3d point2vec(const geometry_msgs::Point &p)
{
    Vector3d v3(p.x, p.y, p.z);
    return v3;
}

inline Vector4d quat2vec(const geometry_msgs::Quaternion &q)
{
    Vector4d v4(q.w, q.x, q.y, q.z);
    return v4;
}

inline Vector3d vec2vec(const geometry_msgs::Vector3 &v3)
{
    Vector3d ev3(v3.x, v3.y, v3.z);
    return ev3;
}


};

#endif