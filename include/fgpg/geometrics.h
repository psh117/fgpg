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
#include <pcl/common/intersections.h>
#include <vector>

#include "fgpg/triangle_plane_data.h"

/**
 p0, u: line
 plane: plane
 @see http://geomalgorithms.com/a05-_intersect-1.html
*/
bool calcLinePlaneIntersection(
  TrianglePlaneData plane, 
  Eigen::Vector3d p0, 
  Eigen::Vector3d u, ///< norm of point
  Eigen::Vector3d &p ///< result
  );

/// @see: http://blackpawn.com/texts/pointinpoly/default.html
bool sameSide(Eigen::Vector3d p1,Eigen::Vector3d p2, Eigen::Vector3d a, Eigen::Vector3d b);

bool pointInTriangle(Eigen::Vector3d p, TrianglePlaneData plane);

Eigen::Vector3d orthogonalVector3d(Eigen::Vector3d n, Eigen::Vector3d v0, double theta);
Eigen::Vector3d getOrthogonalVector(Eigen::Vector3d n);