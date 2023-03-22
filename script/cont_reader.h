
#pragma once
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>


struct ContinuousGraspCandid
{
  std::vector< std::pair< std::pair<Eigen::Vector3f, Eigen::Vector3f>, Eigen::Quaternionf> > candids;

  void loadConfig(std::string file_name)
  {
    YAML::Node yamlnode;

    yamlnode = YAML::LoadFile(file_name);
    for (int i=0 ;i< yamlnode["grasp_points"].size(); i++)
    {
      std::pair< std::pair<Eigen::Vector3f, Eigen::Vector3f>, Eigen::Quaternionf> pose;
      pose.first.first = Eigen::Vector3f::Map(yamlnode["grasp_points"][i]["upper_bound"].as<std::vector<float> >().data());
      pose.first.second = Eigen::Vector3f::Map(yamlnode["grasp_points"][i]["lower_bound"].as<std::vector<float> >().data());
      pose.second.coeffs() = Eigen::Vector4f::Map(yamlnode["grasp_points"][i]["orientation"].as<std::vector<float> >().data());
      candids.push_back(pose);
    }
  }

  Eigen::Isometry3f getGrasp(int index, float ratio)
  {
    if (index >= candids.size())
      throw std::out_of_range("index ");

    const auto & ub = candids[index].first.first;
    const auto & lb = candids[index].first.second;

    const auto & quat = candids[index].second;

    Eigen::Isometry3f pose;
    pose.translation() = (ub-lb) * ratio + lb;
    pose.linear() = quat.matrix();

    return pose;
  }
};