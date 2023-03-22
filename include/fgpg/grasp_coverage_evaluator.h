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

#include <iostream>
#include <vector>
#include <limits>
#include <cmath>
#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

#include "fgpg/triangle_plane_data.h"

typedef Eigen::Array<size_t, 3, 1> Array3size_t;

class GraspCoverageEvaluator
{
public:

  GraspCoverageEvaluator();

  void setModel(const std::vector<TrianglePlaneData> & mesh_data);
  void setModel(const std::vector<Eigen::Vector3f> & mesh_points);

  void setGraspPoints(const std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f> > & grasp_data);

  void getMinMax3D();
  void getNumberOfBin();

  void setLeafSize(float leaf_size, int orientation_size);

  float getFullEntropy();
  float getPosEntropy();

private:
  std::vector<Eigen::Vector3f> mesh_points_;
  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > grasp_data_;
  Eigen::Array3f min_point_;
  Eigen::Array3f max_point_;

  Eigen::Vector3i min_bin_, max_bin_, div_bin_, divb_mul_;

  Eigen::Tensor<int, 6> num_points_tensor_full_;
  Eigen::Tensor<int, 3> num_points_tensor_pos_;

  float leaf_size_ {0.05}; // Cube shaped voxel (x=y=z)
  float inverse_leaf_size_ {20};
  int orientation_size_ {3};

  int total_num_points_ {1};
};