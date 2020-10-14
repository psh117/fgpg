
#pragma once
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>


struct ContinuousGraspCandid
{
  std::vector< std::pair< std::pair<Eigen::Vector3d, Eigen::Vector3d>, Eigen::Quaterniond> > candids;

  void loadConfig(std::string file_name)
  {
    YAML::Node yamlnode;

    yamlnode = YAML::LoadFile(file_name);
    for (int i=0 ;i< yamlnode["grasp_points"].size(); i++)
    {
      std::pair< std::pair<Eigen::Vector3d, Eigen::Vector3d>, Eigen::Quaterniond> pose;
      pose.first.first = Eigen::Vector3d::Map(yamlnode["grasp_points"][i]["upper_bound"].as<std::vector<double> >().data());
      pose.first.second = Eigen::Vector3d::Map(yamlnode["grasp_points"][i]["lower_bound"].as<std::vector<double> >().data());
      pose.second.coeffs() = Eigen::Vector4d::Map(yamlnode["grasp_points"][i]["orientation"].as<std::vector<double> >().data());
      candids.push_back(pose);
    }
  }

  Eigen::Isometry3d getGrasp(int index, double ratio)
  {
    if (index >= candids.size())
      throw std::out_of_range("index ");

    const auto & ub = candids[index].first.first;
    const auto & lb = candids[index].first.second;

    const auto & quat = candids[index].second;

    Eigen::Isometry3d pose;
    pose.translation() = (ub-lb) * ratio + lb;
    pose.linear() = quat.matrix();

    return pose;
  }
};