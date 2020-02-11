 
/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 */

#include "fgpg/mesh_sampling.h"


double
uniform_deviate (int seed)
{
  double ran = seed * (1.0 / (RAND_MAX + 1.0));
  return ran;
}

void
randomPointTriangle (float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3,
                     float r1, float r2, Eigen::Vector3d& p)
{
  float r1sqr = std::sqrt (r1);
  float OneMinR1Sqr = (1 - r1sqr);
  float OneMinR2 = (1 - r2);
  a1 *= OneMinR1Sqr;
  a2 *= OneMinR1Sqr;
  a3 *= OneMinR1Sqr;
  b1 *= OneMinR2;
  b2 *= OneMinR2;
  b3 *= OneMinR2;
  c1 = r1sqr * (r2 * c1 + b1) + a1;
  c2 = r1sqr * (r2 * c2 + b2) + a2;
  c3 = r1sqr * (r2 * c3 + b3) + a3;
  p[0] = c1;
  p[1] = c2;
  p[2] = c3;
}

void
randPSurface (const std::vector <TrianglePlaneData>& planes, 
std::vector<double> &cumulativeAreas, double totalArea, 
Eigen::Vector3d& p, Eigen::Vector3d& n, 
double r, double r1, double r2)
{
  std::vector<double>::iterator low = std::lower_bound (cumulativeAreas.begin (), cumulativeAreas.end (), r);
  int el = int (low - cumulativeAreas.begin ());

  // OBJ: Vertices are stored in a counter-clockwise order by default
  Eigen::Vector3d v1 = planes[el].points[0] - planes[el].points[2];
  Eigen::Vector3d v2 = planes[el].points[1] - planes[el].points[2];
  n = v1.cross (v2);
  n.normalize ();

  randomPointTriangle (float (planes[el].points[0][0]), float (planes[el].points[0][1]), float (planes[el].points[0][2]),
                       float (planes[el].points[1][0]), float (planes[el].points[1][1]), float (planes[el].points[1][2]),
                       float (planes[el].points[2][0]), float (planes[el].points[2][1]), float (planes[el].points[2][2]), r1, r2, p);
}
