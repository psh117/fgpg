
#include "fgpg/grasp_point_generator.h"

Eigen::Vector3d GraspPointGenerator::PCL2eigen(const PointT &pcl)
{
  Eigen::Vector3d eig;
  eig(0) = pcl.x;
  eig(1) = pcl.y;
  eig(2) = pcl.z;
  return eig;
}

Eigen::Vector3d GraspPointGenerator::PCLNormal2eigen(const PointT &pcl)
{
  Eigen::Vector3d eig;
  eig(0) = pcl.normal_x;
  eig(1) = pcl.normal_y;
  eig(2) = pcl.normal_z;
  return eig;
}

void GraspPointGenerator::eigen2PCL(const Eigen::Vector3d &eig, PointT &pcl, int r, int g, int b)
{
  pcl.x = eig(0);
  pcl.y = eig(1);
  pcl.z = eig(2);
  pcl.r = r;
  pcl.g = g;
  pcl.b = b;
}

void GraspPointGenerator::eigen2PCL(const Eigen::Vector3d &eig, const Eigen::Vector3d &norm, PointT &pcl, int r, int g, int b)
{
  pcl.x = eig(0);
  pcl.y = eig(1);
  pcl.z = eig(2);
  pcl.normal_x = norm(0);
  pcl.normal_y = norm(1);
  pcl.normal_z = norm(2);
  pcl.r = r;
  pcl.g = g;
  pcl.b = b;
}

const std::vector <TrianglePlaneData> & GraspPointGenerator::getTrianglePlaneData() 
{ return planes_; }

const std::vector <GraspData> & GraspPointGenerator::getGraspData() 
{ return grasp_cand_collision_free_; }

void GraspPointGenerator::setConfig(const YAMLConfig &config)
{
  config_ = config;
  collision_check_.gripper_model_.setParams(
    config_.gripper_params[0], config_.gripper_params[1], config_.gripper_params[2], 
    config_.gripper_params[3], config_.gripper_params[4], config_.gripper_params[5]);
}

void GraspPointGenerator::setMesh(const std::vector <TrianglePlaneData> triangle_mesh)
{
  planes_ = triangle_mesh;
  collision_check_.loadMesh(triangle_mesh);
}

void GraspPointGenerator::generate()
{
  sample();
  collisionCheck();
}

void GraspPointGenerator::findGraspableOutline()
{
  for (auto & plane : planes_)
  {
    for(auto & line : plane.line_data)
    {
      for (auto & grasp : line.sampled_grasp_data)
      {
        collisionCheck(grasp);
      }
      line.calcGraspable();
    }
  }
}

void GraspPointGenerator::display(pcl::PolygonMesh& mesh)
{
  if(config_.display_figure)
  {
    pcl::visualization::PCLVisualizer vis1 ("Generated preliminary points");
    vis1.addPointCloud<pcl::PointXYZRGBNormal> (candid_sample_cloud_);
    vis1.addPolygonMesh(mesh, "meshes",0);
    vis1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, config_.mesh_color[0], config_.mesh_color[1], config_.mesh_color[2], "meshes");
    vis1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "meshes");
    vis1.setBackgroundColor (config_.background_color[0], config_.background_color[1], config_.background_color[2]);
    vis1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, config_.point_size);
    vis1.addPointCloudNormals<pcl::PointXYZRGBNormal> (candid_sample_cloud_, 1, .01f,"cloud_normals");
    if(config_.attach_coordination)
      vis1.addCoordinateSystem(0.1);
    vis1.setCameraPosition(config_.camera_position[0],config_.camera_position[1],config_.camera_position[2],config_.camera_position[3],config_.camera_position[4],config_.camera_position[5]);

    vis1.spin ();

    if(config_.display_hand)
    {
      pcl::visualization::PCLVisualizer vis2 ("Generated candidates");

      vis2.addPointCloud<pcl::PointXYZRGBNormal> (candid_result_cloud);
      vis2.addPolygonMesh(mesh, "meshes",0);
      vis2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 
        config_.mesh_color[0], config_.mesh_color[1], config_.mesh_color[2], "meshes");
      vis2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
        3, "meshes");
      vis2.setBackgroundColor(config_.background_color[0], config_.background_color[1], config_.background_color[2]);
      vis2.setCameraPosition(config_.camera_position[0],config_.camera_position[1],config_.camera_position[2],config_.camera_position[3],config_.camera_position[4],config_.camera_position[5]);
      
      if(config_.attach_coordination)
        vis2.addCoordinateSystem(0.1);

      int id_num = 0;

      for(auto & grasp : grasp_cand_collision_free_)
      {
        PointT point;
        GraspPointGenerator::eigen2PCL(grasp.points[0], grasp.hand_transform.linear().row(2), point, 
                  config_.point_color[0]*255,config_.point_color[1]*255,config_.point_color[2]*255);
        
        candid_result_cloud->points.push_back(point);
        candid_result_cloud->width++;

        PointT pcl_point_2;
        GraspPointGenerator::eigen2PCL(grasp.points[1], grasp.hand_transform.linear().row(2), point, 
                  config_.point_color[0]*255,config_.point_color[1]*255,config_.point_color[2]*255);
        
        candid_result_cloud->points.push_back(point);
        candid_result_cloud->width++;   

        collision_check_.gripper_model_.drawGripper(vis2, grasp.hand_transform, std::to_string(id_num++),
          config_.gripper_color[0],config_.gripper_color[1],config_.gripper_color[2], 
          config_.gripper_opacity, grasp.getDist()/2);

      }

      if(config_.display_collision)
      {
        for(auto & grasp : grasp_cand_in_collision_)
        {
            collision_check_.gripper_model_.drawGripper(vis2, grasp.hand_transform, std::to_string(id_num++),1,0,0,config_.gripper_opacity, grasp.getDist()/2);
        }
      }
      vis2.spin();
    }
  }
}

void GraspPointGenerator::displayOutline(pcl::PolygonMesh& mesh)
{
  if(config_.display_figure)
  {
    pcl::visualization::PCLVisualizer vis1 ("Generated preliminary points");
    vis1.addPointCloud<pcl::PointXYZRGBNormal> (candid_result_cloud);
    vis1.addPolygonMesh(mesh, "meshes",0);
    vis1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, config_.mesh_color[0], config_.mesh_color[1], config_.mesh_color[2], "meshes");
    vis1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "meshes");
    vis1.setBackgroundColor (config_.background_color[0], config_.background_color[1], config_.background_color[2]);
    vis1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, config_.point_size);
    vis1.addPointCloudNormals<pcl::PointXYZRGBNormal> (candid_result_cloud, 1, .01f,"cloud_normals");
    if(config_.attach_coordination)
      vis1.addCoordinateSystem(0.1);
    vis1.setCameraPosition(config_.camera_position[0],config_.camera_position[1],config_.camera_position[2],config_.camera_position[3],config_.camera_position[4],config_.camera_position[5]);


    int line_id = 0;
    for (auto & plane : planes_)
    {
      for(auto & line : plane.line_data)
      {
        if(line.graspable)
        {
          PointT p1, p2;
          eigen2PCL(line.points.first, p1, 255,0,0);
          eigen2PCL(line.points.second, p2, 255,0,0);
          vis1.addLine(p1,p2,std::string("line") + std::to_string(line_id++));
        }
      }
    }

    vis1.spin ();

  }
}

void GraspPointGenerator::saveGraspCandidates(std::ofstream &of)
{  
  of << "grasp_points: " << std::endl;
  Eigen::IOFormat CommaInitFmt(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");

  for(auto & grasp : grasp_cand_collision_free_)
  {
    Eigen::Matrix3d new_rot; // Z<-X, Y <-Z
    new_rot.col(0) = grasp.hand_transform.linear().col(1);
    new_rot.col(1) = grasp.hand_transform.linear().col(2);
    new_rot.col(2) = grasp.hand_transform.linear().col(0);

    Eigen::Quaterniond quat(new_rot);
    of << "    - position:    " << grasp.hand_transform.translation().transpose().format(CommaInitFmt) <<  std::endl
        << "      orientation: [" << quat.x() << ", " << quat.y() <<", " << quat.z() << ", " << quat.w() << "]" << std::endl; 
  }
}

void GraspPointGenerator::samplePointsInTriangle(TrianglePlaneData plane)
{
  plane.calculateIncenter();

  auto & n = plane.normal;
  auto & p1 = plane.points[0];
  auto & p2 = plane.points[1];
  auto & p3 = plane.points[2];

  std::vector<Line> lines;
  lines.push_back(std::make_pair(p1, p3));
  lines.push_back(std::make_pair(p2, p1));
  lines.push_back(std::make_pair(p3, p2));

  for(int i=0; i<3; i++)
  {
    Eigen::Vector3d e = lines[i].first - lines[i].second;
    Eigen::Vector3d c = (n.cross(e)).normalized();
    
    Eigen::Vector3d new_p1, new_p2;
    double grasp_length = config_.gripper_params[0] - config_.gripper_depth_epsilon;
    new_p1 = lines[i].first + c * grasp_length;
    new_p2 = lines[i].second + c * grasp_length;

    plane.line_data[i].points = lines[i];
    plane.line_data[i].approach_direction = c;

    samplePointsInLine (n, new_p1, new_p2, c, plane.line_data[i]);
  }
}

void GraspPointGenerator::samplePointsInLine(const Eigen::Vector3d &norm, Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d direction_vector, LineData & line_data)
{
  Eigen::Vector3d u = p2 - p1; // e
  double len = u.norm(); // ||e||
  double point_len = len - config_.point_distance;
  int point_n = round(point_len / config_.point_distance);
  double real_point_dist = point_len / point_n;

  Eigen::Vector3d u_norm = u.normalized();

  for(int i=0; i<point_n+1; i++)
  {
    Eigen::Vector3d new_p;
    new_p = p1 + u_norm * (config_.point_distance/2 + i * real_point_dist);
    makePair(norm,new_p,direction_vector, line_data);
  }
}

void GraspPointGenerator::makePair(const Eigen::Vector3d &norm, Eigen::Vector3d new_p, Eigen::Vector3d direction_vector, LineData & line_data)
{
  static int h = 0;
  PointT pcl_point_1;
  for (auto & plane : planes_)
  {
    auto & n = plane.normal;
    if ((norm+n).norm() < 6e-1) // opposit dir tolerance
    {
      Eigen::Vector3d result_p;
      calcLinePlaneIntersection(plane, new_p, -norm, result_p);

      if (!pointInTriangle(result_p, plane))
      {
          continue;
      }
      
      eigen2PCL(new_p, norm, pcl_point_1, config_.point_color[0]*255,config_.point_color[1]*255,config_.point_color[2]*255);
      candid_sample_cloud_->points.push_back(pcl_point_1);
      candid_sample_cloud_->width++;

      PointT pcl_point_2;
      eigen2PCL(result_p, n, pcl_point_2, config_.point_color[0]*255,config_.point_color[1]*255,config_.point_color[2]*255);
      candid_sample_cloud_->points.push_back(pcl_point_2);
      candid_sample_cloud_->width++;

      GraspData gd;
      gd.hand_transform.linear().col(0) = direction_vector; // X
      gd.hand_transform.linear().col(1) = norm.cross(direction_vector); // Y = Z cross X
      gd.hand_transform.linear().col(2) = norm; // Z
      gd.hand_transform.translation() = (new_p + result_p) / 2;
      gd.points.push_back(new_p);
      gd.points.push_back(result_p);

      grasps_.push_back(gd);

      break;
      
    }
  }
}

void GraspPointGenerator::sample()
{
  if (config_.point_generation_method == "geometry_analysis")
  {
    analyticSample();
  }
  else if (config_.point_generation_method == "random_sample")
  {
    randomSample();
  }
}

void GraspPointGenerator::analyticSample ()
{
  for (auto & plane : planes_)
  {
    samplePointsInTriangle(plane);
  }
}

void GraspPointGenerator::randomSample ()
{
  std::default_random_engine generator;
  std::uniform_real_distribution<double> orientation_distribution(0.0,M_PI);
  std::uniform_real_distribution<double> mesh_distribution(0.0,1.0);

  double totalArea = 0;
  std::vector<double> cumulativeAreas (planes_.size(), 0);
  for (int i=0; i<planes_.size(); i++)
  {
    totalArea += planes_[i].area;
    cumulativeAreas[i] = totalArea;

  }
  candid_sample_cloud_->points.resize (config_.random_point_num);
  candid_sample_cloud_->width = static_cast<std::uint32_t> (config_.random_point_num);
  candid_sample_cloud_->height = 1;

  for (std::size_t i = 0; i < config_.random_point_num; i++)
  {
    Eigen::Vector3d p;
    Eigen::Vector3d n (0, 0, 0);
    double r = mesh_distribution(generator) * totalArea;
    double r1 = mesh_distribution(generator);
    double r2 = mesh_distribution(generator);
    randPSurface (planes_, cumulativeAreas, totalArea, p, n, r, r1, r2);
    double theta = orientation_distribution(generator);
    Eigen::Vector3d orth = getOrthogonalVector(n);
    Eigen::Vector3d dir = orthogonalVector3d(n, orth, theta);

    if(dir.norm() > 1.01 || dir.norm() < 0.99)
    {
      std::cout << "dir norm is " << dir.norm() << std::endl;
      std::cout <<"[WARN] norm error dir: " << dir.transpose()<< std::endl;
      std::cout <<"[WARN] n: " << n.transpose() << std::endl; 
    }

    if(n.norm() > 1.01 || n.norm() < 0.99)
    {
      std::cout << "n norm is " << n.norm() << endl;
      std::cout <<"[WARN] norm error n: " << n.transpose()<< std::endl; 
    }
    LineData tmp;
    makePair(n, p, dir, tmp);
  }
}

void GraspPointGenerator::collisionCheck()
{
  for(auto & grasp : grasps_)
  {
    if (grasp.getDist() > config_.gripper_params[1] * 2) continue;
    if(collision_check_.isFeasible(grasp.hand_transform, grasp.getDist()/2 + 0.001))
    {
      if(config_.remove_same_pose)
      {
        bool is_same = false;
        for(auto & in_grasp : grasp_cand_collision_free_)
        {
          if( in_grasp.isSame(grasp, config_.same_dist, config_.same_angle) )
          {
            is_same = true;
            break;
          }
        }
        if( is_same )
        {
          continue;
        }
      }
      grasp.available = true;
      grasp_cand_collision_free_.push_back(grasp);
    }
    else
    {
      grasp_cand_in_collision_.push_back(grasp);
      grasp.available = false;
    }
  }
}

void GraspPointGenerator::collisionCheck(GraspData &grasp)
{
  grasp.available = false;
  
  if (grasp.getDist() > config_.gripper_params[1] * 2);
    return;

  if(collision_check_.isFeasible(grasp.hand_transform, grasp.getDist()/2 + 0.001))
  {
    grasp.available = true;
  }
}
