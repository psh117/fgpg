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

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <vtkVersion.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/features/integral_image_normal.h>

#include "fgpg/triangle_plane_data.h"

static std::vector<TrianglePlaneData> buildTriangleData(pcl::PolygonMesh & mesh)
{
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New ();
  pcl::io::mesh2vtk (mesh, polydata);

  vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New ();
  triangleFilter->SetInputData (polydata);
  triangleFilter->Update ();

  vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
  triangleMapper->SetInputConnection (triangleFilter->GetOutputPort ());
  triangleMapper->Update ();
  polydata = triangleMapper->GetInput ();

  polydata->BuildCells ();
  vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys ();

  double p1[3], p2[3], p3[3];

  std::vector<TrianglePlaneData> triangles;
  
  vtkIdType npts = 0;
  vtkIdType *ptIds = nullptr;
  for (cells->InitTraversal (); cells->GetNextCell (npts, ptIds);)
  {
    TrianglePlaneData plane_data;
    polydata->GetPoint (ptIds[0], p1);
    polydata->GetPoint (ptIds[1], p2);
    polydata->GetPoint (ptIds[2], p3);
    plane_data.area = vtkTriangle::TriangleArea (p1, p2, p3);

    Eigen::Vector3f v1 = Eigen::Vector3f (p1[0], p1[1], p1[2]) - Eigen::Vector3f (p3[0], p3[1], p3[2]);
    Eigen::Vector3f v2 = Eigen::Vector3f (p2[0], p2[1], p2[2]) - Eigen::Vector3f (p3[0], p3[1], p3[2]);
    Eigen::Vector3f n = v1.cross (v2);
    n.normalize ();
    // std::cout << "n: " << n.transpose() << std::endl;

    plane_data.points.resize(3);
    
    float fp1[3], fp2[3], fp3[3];
    for (int i = 0; i < 3; i++)
    {
      fp1[i] = p1[i];
      fp2[i] = p2[i];
      fp3[i] = p3[i];
    }
    
    plane_data.points[0] = (Eigen::Map<const Eigen::Vector3f>(fp1));
    plane_data.points[1] = (Eigen::Map<const Eigen::Vector3f>(fp2));
    plane_data.points[2] = (Eigen::Map<const Eigen::Vector3f>(fp3));
    plane_data.normal = n;

    triangles.push_back(plane_data);
  }

  return triangles;
}