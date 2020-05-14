#pragma once

#include <vector>
#include <fgpg/triangle_plane_data.h>
#include <fgpg/fcl_utils.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>


double getGraspDistance(const Eigen::Isometry3d& transform, const FCLGripper& gripper, const std::vector<TrianglePlaneData>& planes)
{
  auto n = gripper.getPalmNormalVector(transform);
  auto o = gripper.getPalmOrigin(transform);
  double min_distance = std::numeric_limits<double>::infinity();
  int index = 0;
  for (auto it = planes.begin(); it != planes.end(); it++)
  {
    Eigen::Vector3d p;
    calcLinePlaneIntersection(*it, o, n, p);
    if (pointInTriangle(p, *it))
    {
      double dist = ((o-it->points[0]).transpose() * it->normal);
      if (dist < 0.0) continue;
      if (dist < min_distance)
      {
        min_distance = dist;
        index = std::distance(planes.begin(), it);
      }
    }
  }
  // std::cout << planes[index] << std::endl
  //           << min_distance << std::endl;
  // std::cout << "n: " << n.transpose() << std::endl
  //           << "o: " << o.transpose() << std::endl;
  return min_distance;
}


// class GraspAreaCalculator
// {
// public:
//   GraspAreaCalculator();
//   double calcArea(const Eigen::Isometry3d & transform)
//   {
//     auto f1 = gripper_.getFinger1PlanePoints(transform);
//     auto f2 = gripper_.getFinger2PlanePoints(transform);
//     auto p = gripper_.getPalmPlanePoints(transform);

//     std::vector<Eigen::Matrix<double, 3, 5> > planes;
//     planes.push_back(f1);
//     planes.push_back(f2);
//     planes.push_back(p);

//     typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > polygon;
//     typedef boost::geometry::model::d2::point_xy<double> point_type;

//     for (auto & plane : planes)
//     {
//       Eigen::Vector3d normal = plane.col(4);
//       Eigen::Vector3d v1 = plane.col(0) - plane.col(1);
//       Eigen::Vector3d v2 = plane.col(0) - plane.col(2);

//       polygon triangle_polygon, gripper_polygon;
//       // triangle.outer().

      
//     }

//   }
  

// private:
//   TrianglePlaneData triangle_;
//   const FCLGripper &gripper_;
// };