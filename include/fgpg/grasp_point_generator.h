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
#include <random>

#include <Eigen/Dense>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/features/integral_image_normal.h>

#include "fgpg/geometrics.h"
#include "fgpg/grap_data.h"
#include "fgpg/hsv2rgb.h"
#include "fgpg/fcl_utils.h"
#include "fgpg/mesh_sampling.h"
#include "fgpg/yaml_config.h"
#include "fgpg/calc_area.h"

typedef std::pair<Eigen::Vector3f, Eigen::Vector3f> Line;

class GraspPointGenerator
{
public:
  // Static functions
  static Eigen::Vector3f PCL2eigen(const PointT &pcl);
  static Eigen::Vector3f PCLNormal2eigen(const PointT &pcl);

  static void eigen2PCL(const Eigen::Vector3f &eig, PointT &pcl, int r = 128, int g = 128, int b = 128);
  static void eigen2PCL(const Eigen::Vector3f &eig, const Eigen::Vector3f &norm, PointT &pcl, int r = 128, int g = 128, int b = 128);

  const std::vector <TrianglePlaneData> & getTrianglePlaneData();
  const std::vector <GraspData> & getGraspData();

  void setConfig(const YAMLConfig &config);
  void setMesh(const std::vector <TrianglePlaneData> triangle_mesh);
  void generate();

  void findGraspableOutline();

  void display(pcl::PolygonMesh& mesh);
  void display(pcl::PolygonMesh& mesh, std::vector<Eigen::Isometry3f>& gripper_transforms, std::vector<float>& grasp_width);
  void displayOutline(pcl::PolygonMesh& mesh);
  void saveGraspCandidates(std::ofstream &of);
  void saveContGraspCandidates(std::ofstream &of);

  float getAverageDistance();

private:
  CollisionCheck collision_check_;

  std::vector <GraspData> grasps_;  ///< All generated grasp pose candidates
  std::vector <GraspData> grasp_cand_collision_free_;
  std::vector <GraspData> grasp_cand_in_collision_;

  std::vector <ContGraspPose> continuous_grasp_pose_;
  std::vector <ContGraspPose> continuous_grasp_pose_simplified_;

  std::vector <TrianglePlaneData> planes_;

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr candid_sample_cloud_ {new pcl::PointCloud<pcl::PointXYZRGBNormal>};
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr candid_result_cloud {new pcl::PointCloud<pcl::PointXYZRGBNormal>};

  YAMLConfig config_;

  void samplePointsInTriangle(TrianglePlaneData & plane);
  void samplePointsInLine(const Eigen::Vector3f &norm, Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f direction_vector, LineData & line_data);
  // void makePair
  void makePair(const Eigen::Vector3f &norm, Eigen::Vector3f new_p, Eigen::Vector3f direction_vector, LineData & line_data);

  void sample();
  void analyticSample ();
  void randomSample ();
  void collisionCheck();
  void collisionCheck(GraspData &grasp);
  void simplifyContGraspCandidates();
};
