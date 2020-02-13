/*
 * BSD 2-Clause License
 * 
 * Copyright (c) 2020, Suhan Park
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <vector>

#include "fgpg/grap_data.h"

struct LineData
{
  Eigen::Vector3d approach_direction;
  std::pair<Eigen::Vector3d, Eigen::Vector3d> points;
  std::vector<GraspData> sampled_grasp_data;
  bool graspable {false};

  void calcGraspable()
  {
    if(sampled_grasp_data.size() == 0)
      return;
    
    graspable = true;

    for (auto& grasp : sampled_grasp_data)
      if(grasp.available == false)
      {
        graspable = false;
        return;
      }
  }
};

struct TrianglePlaneData
{
  Eigen::Vector3d normal;
  std::vector < Eigen::Vector3d > points {3};
  double area;
  Eigen::Vector3d incenter;

  std::vector < LineData > line_data {3};

  friend std::ostream & operator << (std::ostream &out, const TrianglePlaneData &d)
  {
    out << "normal: " << d.normal.transpose() << std::endl <<
     "area: " << d.area << std::endl <<
    "points: " << std::endl;
    for(const auto &point : d.points)
    {
      out << "        " << point.transpose() << std::endl;
    }
    
    return out;
  }
  void calculateIncenter()
  {
    if (points.size() != 3)
      return;
    
    auto &p1 = points[0];
    auto &p2 = points[1];
    auto &p3 = points[2];

    double a = (p1 - p2).norm();
    double b = (p1 - p3).norm();
    double c = (p3 - p2).norm();

    double r = a + b + c;

    for(int i=0; i<3; ++i)
    {
      incenter(i) = (a*p3(i) + b*p2(i) + c*p1(i)) / r;
    }
  }
  void calculateSmallTriangle(double len)
  {
    calculateIncenter();
  }
};
