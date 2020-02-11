#include "fgpg/geometrics.h"

#include <iostream>
#include <Eigen/Dense>
#include <pcl/common/intersections.h>
#include <vector>

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
  )
{
  auto n = plane.normal;
  if(n.dot(u) == 0.0)
  {
    return false;
  }
  Eigen::Vector3d w = p0 - plane.points[0];
  double s = -n.dot(w) / n.dot(u);

  // inverse direction
  if(s<0)
    return false;

  p = p0 + s*u;

  return true;
}


/// @see: http://blackpawn.com/texts/pointinpoly/default.html
bool sameSide(Eigen::Vector3d p1,Eigen::Vector3d p2, Eigen::Vector3d a, Eigen::Vector3d b)
{
  auto cp1 = (b-a).cross(p1-a);
  auto cp2 = (b-a).cross(p2-a);
  if (cp1.dot(cp2) >= 0.0) return true;
  
  return false;
}

bool pointInTriangle(Eigen::Vector3d p, TrianglePlaneData plane)
{
  
  auto a = plane.points[0];
  auto b = plane.points[1];
  auto c = plane.points[2];
  
  if(abs(plane.normal.dot(p-a)) + abs(plane.normal.dot(p-b)) + abs(plane.normal.dot(p-c)) > 1e-6) // not on the plane
  {
    return false;
  }
  // Compute vectors        
  auto v0 = c - a;
  auto v1 = b - a;
  auto v2 = p - a;

  // Compute dot products
  double dot00 = v0.dot(v0);
  double dot01 = v0.dot(v1);
  double dot02 = v0.dot(v2);
  double dot11 = v1.dot(v1);
  double dot12 = v1.dot(v2);

  // Compute barycentric coordinates
  double inv_denom = 1 / (dot00 * dot11 - dot01 * dot01);
  double u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
  double v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

  // Check if point is in triangle
  return (u >= 0) && (v >= 0) && (u + v < 1);
}

Eigen::Vector3d orthogonalVector3d(Eigen::Vector3d n, Eigen::Vector3d v0, double theta)
{
  Eigen::Vector3d v;
  v.setZero();

  v = Eigen::AngleAxisd(theta, n).matrix() * v0;
  return v;
}


Eigen::Vector3d getOrthogonalVector(Eigen::Vector3d n)
{
  Eigen::Vector3d v;

  int max_index = 0;
  int new_index[3];
  double max = 0;
  for(int i=0; i<3; i++)
  {
    if(abs(n(i)) > max)
    {
      max = abs(n(i));
      max_index = i;
    }
  }

  int loc = 0;
  new_index[2] = max_index;
  for(int i=0; i<3; i++)
  {
    if(i == max_index) continue;

    new_index[loc] = i;
    loc++;
  }

  v(new_index[0]) = 1;
  v(new_index[1]) = 1;
  v(new_index[2]) = -(n(new_index[0])+n(new_index[1])) / n(new_index[2]);

  v = v.normalized();

  return v;
}

