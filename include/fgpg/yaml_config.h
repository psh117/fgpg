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
#include <yaml-cpp/yaml.h>

struct YAMLConfig
{
  void loadConfig(std::string file_name)
  {
    YAML::Node yamlnode;

    yamlnode = YAML::LoadFile(file_name);
    point_generation_method = yamlnode["point_generation_method"].as<std::string>();

    gripper_params = yamlnode["gripper_params"].as<std::vector<double> >();
    gripper_depth_epsilon = yamlnode["gripper_depth_epsilon"].as<double> ();

    point_distance = yamlnode["point_distance"].as<double> ();
    random_point_num = yamlnode["random_point_num"].as<int>();

    remove_same_pose = yamlnode["remove_same_pose"].as<bool>();
    same_dist = yamlnode["same_dist"].as<double>();
    same_angle = yamlnode["same_angle"].as<double>();

    output_file_suffix = yamlnode["output_file_suffix"].as<std::string>();

    camera_position = yamlnode["camera_position"].as<std::vector<double> >();

    // Visualization
    attach_coordination = yamlnode["attach_coordination"].as<bool>();
    background_color = yamlnode["background_color"].as<std::vector<double> >();
    mesh_color = yamlnode["mesh_color"].as<std::vector<double> >();
    point_color = yamlnode["point_color"].as<std::vector<double> >();
    gripper_color = yamlnode["gripper_color"].as<std::vector<double> >();

    gripper_opacity = yamlnode["gripper_opacity"].as<double> ();
    point_opacity = yamlnode["point_opacity"].as<double> ();

    point_size = yamlnode["point_size"].as<int> ();

    display_figure = yamlnode["display_figure"].as<bool>();
    display_hand = yamlnode["display_hand"].as<bool>();
    display_collision = yamlnode["display_collision"].as<bool>();
    display_preliminary_points = yamlnode["display_preliminary_points"].as<bool>();

    leaf_size = yamlnode["leaf_size"].as<double> ();
    num_orientation_leaf = yamlnode["num_orientation_leaf"].as<int> ();
  }

  std::string point_generation_method;

  std::vector<double> gripper_params;
  double gripper_depth_epsilon;

  double point_distance;
  int random_point_num;

  bool remove_same_pose;
  double same_dist;
  double same_angle;

  std::string output_file_suffix;

  std::vector<double> camera_position;

  // Visualization
  bool attach_coordination;
  std::vector<double> background_color;
  std::vector<double> mesh_color;
  std::vector<double> point_color;
  std::vector<double> gripper_color;

  double gripper_opacity;
  double point_opacity;

  int point_size;

  bool display_figure;
  bool display_hand;
  bool display_collision;
  bool display_preliminary_points;

  double leaf_size;
  int num_orientation_leaf;
};