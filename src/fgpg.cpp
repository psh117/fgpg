#include <ros/ros.h>

#include <ctime>
#include <fstream>

#include "fgpg/grasp_point_generator.h"
#include "fgpg/fcl_utils.h"
#include "fgpg/hsv2rgb.h"
#include "fgpg/grasp_coverage_evaluator.h"
#include "fgpg/yaml_config.h"
#include "fgpg/vtk_mesh_utils.h"

std::string remove_extension(const std::string& filename) {
  size_t lastdot = filename.find_last_of(".");
  if (lastdot == std::string::npos) return filename;
    return filename.substr(0, lastdot); 
}

int main(int argc, char** argv)
{
  bool use_custom_output_name = false;
  std::string custom_output_name;
  if (argc<3) return -1;
  if (argc == 4) 
  {
    use_custom_output_name = true;
    custom_output_name = std::string(argv[3]);
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

  // random with microsec
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);
  srand(spec.tv_nsec);

  std::string file_name (argv[2]);

  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFile(file_name, mesh);

  std::vector<TrianglePlaneData> triangles = buildTriangleData(mesh);

  GraspPointGenerator gpg;
  gpg.setConfig(config);
  gpg.setMesh(triangles);
  gpg.generate();
  gpg.findGraspableOutline();
  gpg.display(mesh);
  gpg.displayOutline(mesh);

  std::string obj_name;
  if (use_custom_output_name)
  {
    obj_name = custom_output_name;
  }
  else
  {
    obj_name = remove_extension(file_name);
  }
  std::string of_name = obj_name + config.output_file_suffix;
  std::string of_cont_name = obj_name + "_cont"+ config.output_file_suffix;

  std::ofstream of(of_name);
  gpg.saveGraspCandidates(of);

  std::ofstream of_cont(of_cont_name);
  gpg.saveContGraspCandidates(of_cont);

  GraspCoverageEvaluator gce;

  const auto & grasp_cand = gpg.getGraspData();
  std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d> > grasp_datas;
  grasp_datas.reserve(grasp_cand.size());
  for(const auto & grasp : grasp_cand)
  {
    grasp_datas.push_back(std::make_pair(grasp.hand_transform.translation(), grasp.hand_transform.linear().col(0)));
  }

  gce.setModel(triangles);
  gce.setLeafSize(config.leaf_size,config.num_orientation_leaf);
  gce.setGraspPoints(grasp_datas);
  gce.getNumberOfBin();
  double full_entropy = gce.getFullEntropy();
  double pos_entropy = gce.getPosEntropy();

  std::cout << "full_entropy: " << full_entropy << std::endl;
  std::cout << "pos_entropy: " << pos_entropy << std::endl;

  return 0;
}
