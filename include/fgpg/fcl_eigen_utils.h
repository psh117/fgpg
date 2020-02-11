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

#include <Eigen/Dense>
#include <fcl/shape/geometric_shapes.h>

namespace FCLEigenUtils
{

static void convertTransform(const fcl::Transform3f& fcl_input, Eigen::Affine3d & eigen_output)
{
  auto &trans = fcl_input.getTranslation();
  auto &rot = fcl_input.getRotation();

  eigen_output.linear() << rot(0, 0), rot(0, 1), rot(0, 2),
                           rot(1, 0), rot(1, 1), rot(1, 2),
                           rot(2, 0), rot(2, 1), rot(2, 2);
  eigen_output.translation() << trans[0], trans[1], trans[2];
}

static void convertTransform(const Eigen::Affine3d &eigen_input, fcl::Transform3f &fcl_output)
{
  fcl::Matrix3f rotation;
  fcl::Vec3f translation;

  auto &rot = eigen_input.linear();
  auto &trans = eigen_input.translation();

  rotation.setValue(rot(0,0), rot(0,1), rot(0,2),
                    rot(1,0), rot(1,1), rot(1,2),
                    rot(2,0), rot(2,1), rot(2,2));
  translation.setValue(trans(0), trans(1), trans(2));

  fcl_output.setTransform(rotation,translation);
}

}
