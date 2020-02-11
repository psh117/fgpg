
#include <ros/ros.h>

#include <fstream>
#include "fgpg/fcl_utils.h"
#include "fgpg/grasp_coverage_evaluator.h"
#include "fgpg/vtk_mesh_utils.h"
#include "fgpg/yaml_config.h"


void readVector(ifstream &stream, Eigen::Vector3d & vec)
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

  std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d> > grasp_data;
  std::vector<Eigen::Affine3d> grasp_transforms;
  std::vector<double> grasp_widths;
  while (result_file)
  {
    Eigen::Vector3d grasp_bottom, grasp_surface, axis, approach, binormal;
    double grasp_width;
    readVector(result_file, grasp_bottom);
    readVector(result_file, grasp_surface);
    readVector(result_file, axis);
    readVector(result_file, approach);
    readVector(result_file, binormal);
    result_file >> grasp_width;
    grasp_data.push_back(std::make_pair(grasp_bottom,approach));

    Eigen::Affine3d grasp_transform;
    grasp_transform.linear().col(0) = axis;
    grasp_transform.linear().col(1) = approach;
    grasp_transform.linear().col(2) = binormal;
    grasp_transform.linear() = grasp_transform.linear() * Eigen::AngleAxisd(90./180.*M_PI, Eigen::Vector3d::UnitZ());
    grasp_transform.translation() = grasp_bottom;
    grasp_transforms.push_back(grasp_transform);

    grasp_widths.push_back(grasp_width);

      // gd.hand_transform.linear().col(0) = direction_vector; // X
      // gd.hand_transform.linear().col(1) = norm.cross(direction_vector); // Y = Z cross X
      // gd.hand_transform.linear().col(2) = norm; // Z
  }
  
  // Mesh Load
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFile(file_name, mesh);
  std::vector<TrianglePlaneData> triangles = buildTriangleData(mesh);

  gce.setModel(triangles);
  gce.setLeafSize(config.leaf_size,config.num_orientation_leaf);
  gce.setGraspPoints(grasp_data);
  gce.getNumberOfBin();

  double full_entropy = gce.getFullEntropy();
  double pos_entropy = gce.getPosEntropy();

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
    FCLGripper gripper_model;
    gripper_model.setParams(config.gripper_params[0], config.gripper_params[1], config.gripper_params[2], config.gripper_params[3],config.gripper_params[4],config.gripper_params[5]);

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
