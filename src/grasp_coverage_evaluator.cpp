
#include "fgpg/grasp_coverage_evaluator.h"

GraspCoverageEvaluator::GraspCoverageEvaluator() {}

void GraspCoverageEvaluator::setModel(const std::vector<TrianglePlaneData> & mesh_data)
{
  // Storage
  std::vector<Eigen::Vector3f> mesh_points;
  for (auto & triangle : mesh_data)
  {
    mesh_points.push_back(triangle.points[0]);
    mesh_points.push_back(triangle.points[1]);
    mesh_points.push_back(triangle.points[2]);
  }

  setModel(mesh_points);
  // getMinMax3D();
}
void GraspCoverageEvaluator::setModel(const std::vector<Eigen::Vector3f> & mesh_points)
{
  mesh_points_ = mesh_points;
  // getMinMax3D();
}

void GraspCoverageEvaluator::setGraspPoints(const std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f> > & grasp_data)
{
  grasp_data_ = grasp_data;
  total_num_points_ = grasp_data_.size();
}

void GraspCoverageEvaluator::getMinMax3D()
{
  // Eigen::Vector3f min_p, max_p;
  Eigen::Array3f min_p, max_p;
  min_p.setConstant(std::numeric_limits<float>::max());
  max_p.setConstant(std::numeric_limits<float>::min());

  for (const auto & point : mesh_points_)
  {
    Eigen::Array3f array_point = point;
    min_p = min_p.min(array_point);
    max_p = max_p.max(array_point);
  }

  min_point_ = min_p;
  max_point_ = max_p;  
}

void GraspCoverageEvaluator::getNumberOfBin()
{
  getMinMax3D();
  // Compute the minimum and maximum bounding box values
  min_bin_[0] = static_cast<int> ((min_point_[0] * inverse_leaf_size_));
  max_bin_[0] = static_cast<int> ((max_point_[0] * inverse_leaf_size_));
  min_bin_[1] = static_cast<int> ((min_point_[1] * inverse_leaf_size_));
  max_bin_[1] = static_cast<int> ((max_point_[1] * inverse_leaf_size_));
  min_bin_[2] = static_cast<int> ((min_point_[2] * inverse_leaf_size_));
  max_bin_[2] = static_cast<int> ((max_point_[2] * inverse_leaf_size_));


  // Compute the number of divisions needed along all axis
  div_bin_ = max_bin_ - min_bin_ + Eigen::Vector3i::Ones ();

  num_points_tensor_full_.resize(div_bin_(0), div_bin_(1), div_bin_(2), orientation_size_, orientation_size_, orientation_size_);
  num_points_tensor_full_.setZero();
  num_points_tensor_pos_.resize(div_bin_(0), div_bin_(1), div_bin_(2));
  num_points_tensor_pos_.setZero();

  for(const auto & data : grasp_data_)
  {
    const auto & pt = data.first;
    const auto & ap = data.second;
    int i = static_cast<int> ((pt[0] * inverse_leaf_size_) - min_bin_[0]);
    int j = static_cast<int> ((pt[1] * inverse_leaf_size_) - min_bin_[1]);
    int k = static_cast<int> ((pt[2] * inverse_leaf_size_) - min_bin_[2]);
    int l = static_cast<int> (((ap[0]*0.99+1)/2*orientation_size_));
    int m = static_cast<int> (((ap[1]*0.99+1)/2*orientation_size_));
    int n = static_cast<int> (((ap[2]*0.99+1)/2*orientation_size_));

    // Compute the centroid leaf index
    num_points_tensor_full_(i,j,k,l,m,n) += 1;
    num_points_tensor_pos_(i,j,k) += 1;
  }
}

void GraspCoverageEvaluator::setLeafSize(float leaf_size, int orientation_size)
{
  leaf_size_ = leaf_size;
  inverse_leaf_size_ = 1 / leaf_size;
  orientation_size_ = orientation_size;
}

float GraspCoverageEvaluator::getFullEntropy()
{
  float entropy = 0;
  for(int i=0; i<div_bin_(0); i++)
  {
    for(int j=0; j<div_bin_(1); j++)
    {
      for(int k=0; k<div_bin_(2); k++)
      {
        for(int l=0; l<orientation_size_; l++)
        {
          for(int n=0; n<orientation_size_; n++)
          {
            for(int m=0; m<orientation_size_; m++)
            {
              if(num_points_tensor_full_(i,j,k,l,n,m))
              {
                float p = num_points_tensor_full_(i,j,k,l,n,m) / (float)total_num_points_;
                entropy -= p * std::log(p);
              }
            }
          }
        }
      }
    }
  }
  return entropy;
}

float GraspCoverageEvaluator::getPosEntropy()
{
  float entropy = 0;
  for(int i=0; i<div_bin_(0); i++)
  {
    for(int j=0; j<div_bin_(1); j++)
    {
      for(int k=0; k<div_bin_(2); k++)
      {
        if(num_points_tensor_pos_(i,j,k))
        {
          float p = num_points_tensor_pos_(i,j,k) / (float)total_num_points_;
          entropy -= p * std::log(p);
        }
      }
    }
  }
  return entropy;
}
