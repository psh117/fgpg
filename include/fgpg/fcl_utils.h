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

#include "fgpg/fcl_eigen_utils.h"
#include "fgpg/geometrics.h"

#include <fcl/traversal/traversal_node_bvhs.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/collision_node.h>
#include <fcl/collision.h>
#include <fcl/BV/BV.h>
#include <fcl/BV/OBBRSS.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/narrowphase/narrowphase.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef fcl::OBBRSS BV;
typedef fcl::BVHModel<BV> BVHM;
typedef std::shared_ptr<BVHM> BVHMPtr;
using fcl::Box;
typedef std::shared_ptr<fcl::Box> BoxPtr;
using fcl::CollisionObject;
typedef std::shared_ptr<fcl::CollisionObject> CollisionObjectPtr;

typedef pcl::PointCloud<pcl::PointXYZRGBNormal> CloudT;
typedef pcl::PointXYZRGBNormal PointT;
typedef std::shared_ptr<PointT> PointTPtr;

using namespace pcl;

struct FCLGripper
{
  BoxPtr g[4];
  Eigen::Affine3d t[4];

  /// d: depth of the gripper
  /// h: width of the gripper
  /// l: total length of the gripper
  /// x1l: size of bottom gripper (width)
  /// y1l: size of bottom gripper (height)
  /// z2l: size of gripper finger (width)
  /// x4l: size of bar of the gripper (length)
  /// z4l: size of bar of the gripper (width)
  double l, h, d, x1l, y1l, z2l, x4l, z4l;

  void setParams(double id, double ih, double il, double ix1l, double iy1l, double iz2l, double ix4l= 0.08, double iz4l = 0.02)
  {
    l = il;
    h = ih;
    d = id;
    x1l = ix1l;
    y1l = iy1l;
    z2l = iz2l;
    x4l = ix4l;
    z4l = iz4l;

    makeModel();
  }
  void makeModel()
  {
    double z1l = 2*(h+z2l);
    g[0] = std::make_shared<Box>(x1l,y1l,z1l);
    g[1] = std::make_shared<Box>(l,y1l,z2l);
    g[2] = std::make_shared<Box>(l,y1l,z2l);
    g[3] = std::make_shared<Box>(x4l,y1l,z4l);

    t[0].linear().setIdentity();
    t[0].translation() << -d - x1l/2, 0, 0;

    t[1].linear().setIdentity();
    t[1].translation() << -d + l/2, 0, h+z2l/2;
    
    t[2].linear().setIdentity();
    t[2].translation() << -d + l/2, 0, -h-z2l/2;

    t[3].linear().setIdentity();
    t[3].translation() << -d - x1l - x4l/2, 0, 0;
  }

  void changeWidth(double new_h)
  {
    t[1].linear().setIdentity();
    t[1].translation() << -d + l/2, 0, new_h+z2l/2;
    
    t[2].linear().setIdentity();
    t[2].translation() << -d + l/2, 0, -new_h-z2l/2;
  }
  
  void drawGripper(pcl::visualization::PCLVisualizer & vis, 
  const Eigen::Affine3d gripper_transform,
  const std::string &id,
   double r, double g_c, double b, double opacity,
  double dist = -1.0)
  {
    if (dist < 0)
      changeWidth(h);
    else
      changeWidth(dist);

    for(int i=0; i<4; i++)
    {
      auto T = gripper_transform * t[i];
      Eigen::Vector3d position(T.translation());
      Eigen::Quaterniond quat(T.linear());
      Eigen::Vector3f posf;
      Eigen::Quaternionf quatf;

      posf = position.cast <float> ();
      quatf = quat.cast <float> ();
      std::string id_total = "cube" + id + std::to_string(i);
      vis.addCube(posf,quatf,g[i]->side[0],g[i]->side[1],g[i]->side[2], id_total);
      vis.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g_c, b, id_total);
      vis.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, id_total, 0);

      std::string id_total_line = "cube_line" + id + std::to_string(i);
      vis.addCube(posf,quatf,g[i]->side[0],g[i]->side[1],g[i]->side[2], id_total_line);
      vis.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, id_total_line);
      vis.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, id_total_line, 0);
      vis.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id_total_line, 0);
    }
  }
};

class CollisionCheck
{
  public:
  BVHMPtr mesh_model_;
  FCLGripper gripper_model_;
  
  void loadMesh(const std::vector <TrianglePlaneData> & mesh)
  {
    std::vector<fcl::Vec3f > points;
    std::vector<fcl::Triangle> triangles;
    mesh_model_ = std::make_shared<BVHM> ();

    for(const auto & tri_plane: mesh)
    {
      fcl::Triangle tri;

      for(int i=0; i<3; i++)
      {
        tri[i] = points.size();
        points.push_back(
          fcl::Vec3f(
            tri_plane.points[i](0), 
            tri_plane.points[i](1), 
            tri_plane.points[i](2)));
      }
      triangles.push_back(tri);
    }
    mesh_model_->beginModel();
    mesh_model_->addSubModel(points, triangles);
    mesh_model_->endModel();
  }

  bool isFeasible(Eigen::Affine3d gripper_transform, double distance)
  {
    // set the collision request structure, here we just use the default setting
    fcl::CollisionRequest request;
    // result will be returned via the collision result structure
    fcl::CollisionResult result[4];

    fcl::Transform3f init;
    init.setIdentity();
    
    gripper_model_.changeWidth(distance);

    bool is_collided = false;
    for (int i=0; i<4 ;++i)
    {
      Eigen::Affine3d cur_transform = gripper_transform * gripper_model_.t[i];
      fcl::Transform3f fcl_transform;
      FCLEigenUtils::convertTransform(cur_transform, fcl_transform);

      fcl::collide(mesh_model_.get(),init,gripper_model_.g[i].get(),
        fcl_transform, 
        request, result[i]);
      if (result[i].isCollision() == true)
      {
        is_collided = true;
        break;
      }
    }

    return !is_collided;
  }
};