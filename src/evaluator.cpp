
#include <ros/ros.h>

#include <fstream>
#include "fgpg/grasp_point_generator.h"
#include "fgpg/fcl_utils.h"
#include "fgpg/hsv2rgb.h"
#include "fgpg/grasp_coverage_evaluator.h"
#include "fgpg/yaml_config.h"
#include "fgpg/vtk_mesh_utils.h"
#include "fgpg/calc_area.h"

void readVector(ifstream &stream, Eigen::Vector3f & vec)
{
  char g;
  stream >> vec(0) >> g >> vec(1)  >> g >> vec(2) >> g ;
}

int main(int argc, char** argv)
{
  if (argc < 4) 
  {
    std::cout << "Usage: "<< argv[0] << "config.yaml mesh_model.std result_file.txt" << std::endl;
    return -1;
  }

  YAMLConfig config;
  try
  {
    config.loadConfig(std::string(argv[1]));
  }
  catch(std::exception &e)
  {
      ROS_ERROR("Failed to load yaml file");
  }

  GraspCoverageEvaluator gce;

  std::string file_name (argv[2]);
  std::string result_file_name (argv[3]);
  std::ifstream result_file(result_file_name);
  std::cout << file_name <<std::endl;

  std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f> > grasp_data;
  std::vector<Eigen::Isometry3f> grasp_transforms;
  std::vector<float> grasp_widths;
  while (result_file)
  {
    Eigen::Vector3f grasp_bottom, grasp_surface, axis, approach, binormal;
    float grasp_width;
    readVector(result_file, grasp_bottom);
    readVector(result_file, grasp_surface);
    readVector(result_file, axis);
    readVector(result_file, approach);
    readVector(result_file, binormal);
    result_file >> grasp_width;
    grasp_data.push_back(std::make_pair(grasp_bottom,approach));

    Eigen::Isometry3f grasp_transform;
    grasp_transform.linear().col(0) = axis;
    grasp_transform.linear().col(1) = approach;
    grasp_transform.linear().col(2) = binormal;
    grasp_transform.linear() = grasp_transform.linear() * Eigen::AngleAxisf(90./180.*M_PI, Eigen::Vector3f::UnitZ());
    grasp_transform.translation() = grasp_bottom;
    grasp_transforms.push_back(grasp_transform);

    grasp_widths.push_back(grasp_width);

      // gd.hand_transform.linear().col(0) = direction_vector; // X
      // gd.hand_transform.linear().col(1) = norm.cross(direction_vector); // Y = Z cross X
      // gd.hand_transform.linear().col(2) = norm; // Z
  }
  // grasp_transforms.resize(5);
  
  FCLGripper gripper_model;
  gripper_model.setParams(config.gripper_params[0], config.gripper_params[1], config.gripper_params[2], config.gripper_params[3],config.gripper_params[4],config.gripper_params[5]);

  // Mesh Load
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFile(file_name, mesh);
  std::vector<TrianglePlaneData> triangles = buildTriangleData(mesh);

  std::vector<float> dists;
  for(auto& trans : grasp_transforms)
  {
    std::cout << "transform: " << std::endl << trans.matrix() << std::endl;
    float dist = getGraspDistance(trans, gripper_model, triangles);
    dists.push_back(dist);
    std::cout << dist  << std::endl; 
  }
  float average = std::accumulate(dists.begin(), dists.end(), 0.0) / grasp_transforms.size();
  std::cout << "ave: " << average << std::endl;

  gce.setModel(triangles);
  gce.setLeafSize(config.leaf_size,config.num_orientation_leaf);
  gce.setGraspPoints(grasp_data);
  gce.getNumberOfBin();

  float full_entropy = gce.getFullEntropy();
  float pos_entropy = gce.getPosEntropy();

  std::cout << "full_entropy: " << full_entropy << std::endl;
  std::cout << "pos_entropy: " << pos_entropy << std::endl;

  if(config.display_figure)
  {
    pcl::visualization::PCLVisualizer vis ("VOXELIZED SAMPLES CLOUD");
    vis.setBackgroundColor (config.background_color[0], config.background_color[1], config.background_color[2]);
    vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, config.point_size);
    vis.setCameraPosition(config.camera_position[0],config.camera_position[1],config.camera_position[2],config.camera_position[3],config.camera_position[4],config.camera_position[5]);
    if(config.attach_coordination)
      vis.addCoordinateSystem(0.1);


    vis.addPolygonMesh(mesh, "meshes",0);
    vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, config.mesh_color[0], config.mesh_color[1], config.mesh_color[2], "meshes");
    vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "meshes");
    vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_FLAT, "meshes");
    
    int id_num = 0;
    for (int i=0; i<grasp_transforms.size(); i++)
    {
      // std::cout << grasp_transforms[i].matrix() << std::endl << std::endl;
      gripper_model.drawGripper(vis, grasp_transforms[i], std::to_string(id_num++),config.gripper_color[0],config.gripper_color[1],config.gripper_color[2], config.gripper_opacity, 
      grasp_widths[i]/2);
    }
    vis.spin ();
  }

  std::ofstream full_entropy_file("full_entropy.txt");
  std::ofstream pos_entropy_file("pos_entropy.txt");

  full_entropy_file << full_entropy;
  pos_entropy_file << pos_entropy;

  return 0;
}
