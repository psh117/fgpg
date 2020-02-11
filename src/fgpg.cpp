#include <ros/ros.h>

#include <ctime>
#include <fstream>

#include "fgpg/grasp_point_generator.h"
#include "fgpg/fcl_utils.h"
#include "fgpg/hsv2rgb.h"
#include "fgpg/grasp_coverage_evaluator.h"
#include "fgpg/yaml_config.h"
#include "fgpg/vtk_mesh_utils.h"

int main(int argc, char** argv)
{
  if (argc<3) return -1;

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
  gpg.display(mesh);

  std::ofstream of(file_name + config.output_file_suffix);
  gpg.saveGraspCandidates(of);

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
