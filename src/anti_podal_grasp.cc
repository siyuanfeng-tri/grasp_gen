#include "grasp_gen/anti_podal_grasp.h"

std::vector<double> stringToDouble(const std::string &str) {
  std::vector<double> values;
  std::stringstream ss(str);
  double v;

  while (ss >> v) {
    values.push_back(v);
    if (ss.peek() == ' ') {
      ss.ignore();
    }
  }

  return values;
}

AntiPodalGraspPlanner::AntiPodalGraspPlanner(std::string config_file_name) {
  ConfigFile config_file(config_file_name);

  double finger_width = config_file.getValueOfKey<double>("finger_width", 0.01);
  double hand_outer_diameter =
      config_file.getValueOfKey<double>("hand_outer_diameter", 0.12);
  double hand_depth = config_file.getValueOfKey<double>("hand_depth", 0.06);
  double hand_height = config_file.getValueOfKey<double>("hand_height", 0.02);
  double init_bite = config_file.getValueOfKey<double>("init_bite", 0.01);

  std::cout << "finger_width: " << finger_width << "\n";
  std::cout << "hand_outer_diameter: " << hand_outer_diameter << "\n";
  std::cout << "hand_depth: " << hand_depth << "\n";
  std::cout << "hand_height: " << hand_height << "\n";
  std::cout << "init_bite: " << init_bite << "\n";

  bool voxelize = config_file.getValueOfKey<bool>("voxelize", true);
  bool remove_outliers =
      config_file.getValueOfKey<bool>("remove_outliers", false);
  std::string workspace_str =
      config_file.getValueOfKeyAsString("workspace", "");
  std::string camera_pose_str =
      config_file.getValueOfKeyAsString("camera_pose", "");
  std::vector<double> workspace = stringToDouble(workspace_str);
  std::vector<double> camera_pose = stringToDouble(camera_pose_str);
  std::cout << "voxelize: " << voxelize << "\n";
  std::cout << "remove_outliers: " << remove_outliers << "\n";
  std::cout << "workspace: " << workspace_str << "\n";
  std::cout << "camera_pose: " << camera_pose_str << "\n";

  int num_samples = config_file.getValueOfKey<int>("num_samples", 1000);
  int num_threads = config_file.getValueOfKey<int>("num_threads", 1);
  double nn_radius = config_file.getValueOfKey<double>("nn_radius", 0.01);
  int num_orientations = config_file.getValueOfKey<int>("num_orientations", 8);
  int rotation_axis = config_file.getValueOfKey<int>("rotation_axis", 2);
  std::cout << "num_samples: " << num_samples << "\n";
  std::cout << "num_threads: " << num_threads << "\n";
  std::cout << "nn_radius: " << nn_radius << "\n";
  std::cout << "num_orientations: " << num_orientations << "\n";
  std::cout << "rotation_axis: " << rotation_axis << "\n";

  bool plot_grasps = config_file.getValueOfKey<bool>("plot_grasps", true);
  bool plot_normals = config_file.getValueOfKey<bool>("plot_normals", false);
  std::cout << "plot_grasps: " << plot_grasps << "\n";
  std::cout << "plot_normals: " << plot_normals << "\n";

  // Create object to generate grasp candidates.
  CandidatesGenerator::Parameters generator_params;
  generator_params.num_samples_ = num_samples;
  generator_params.num_threads_ = num_threads;
  generator_params.plot_normals_ = plot_normals;
  generator_params.plot_grasps_ = plot_grasps;
  generator_params.remove_statistical_outliers_ = remove_outliers;
  generator_params.voxelize_ = voxelize;
  generator_params.workspace_ = workspace;
  HandSearch::Parameters hand_search_params;
  hand_search_params.finger_width_ = finger_width;
  hand_search_params.hand_outer_diameter_ = hand_outer_diameter;
  hand_search_params.hand_depth_ = hand_depth;
  hand_search_params.hand_height_ = hand_height;
  hand_search_params.init_bite_ = init_bite;
  hand_search_params.nn_radius_frames_ = nn_radius;
  hand_search_params.num_orientations_ = num_orientations;
  hand_search_params.num_samples_ = num_samples;
  hand_search_params.num_threads_ = num_threads;
  hand_search_params.rotation_axis_ = rotation_axis;
  candidates_generator = std::make_unique<CandidatesGenerator>(
      generator_params, hand_search_params);
}

void AntiPodalGraspPlanner::SetInputCloud(
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr &cloud) {
  // Set the camera pose.
  Eigen::Matrix3Xd view_points(3, 1);
  view_points << 0.5, 0, 1;

  Eigen::MatrixXi camera_source = Eigen::MatrixXi::Ones(1, cloud->size());
  cloud_camera =
      std::make_unique<CloudCamera>(cloud, camera_source, view_points);
}

void AntiPodalGraspPlanner::SetInputCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &points,
    const pcl::PointCloud<pcl::Normal>::ConstPtr &normals) {
  // Set the camera pose.
  Eigen::Matrix3Xd view_points(3, 1);
  view_points << 0.5, 0, 1;

  Eigen::MatrixXi camera_source = Eigen::MatrixXi::Ones(1, points->size());
  cloud_camera = std::make_unique<CloudCamera>(points, normals, camera_source,
                                               view_points);
}

std::vector<AntiPodalGrasp> AntiPodalGraspPlanner::GenerateAntipodalGrasp() {
  candidates_generator->preprocessPointCloud(*cloud_camera);
  std::vector<Grasp> candidates =
      candidates_generator->generateGraspCandidates(*cloud_camera);
  // std::cout << "generated grasps to evaluate antipodalness" << std::endl;
  std::vector<AntiPodalGrasp> antipodal_grasps;
  for (size_t i = 0; i < candidates.size(); ++i) {
    // Get approach direction as negative z.
    Eigen::Vector3d hand_z_dir = -candidates[i].getApproach();
    Eigen::Vector3d hand_y_dir = candidates[i].getBinormal();
    Eigen::Vector3d hand_x_dir = hand_y_dir.cross(hand_z_dir);
    Eigen::Matrix3d hand_frame;
    hand_frame.col(0) = hand_x_dir;
    hand_frame.col(1) = hand_y_dir;
    hand_frame.col(2) = hand_z_dir;
    Eigen::Isometry3d tf_hand = Eigen::Isometry3d::Identity();
    tf_hand.translation() = candidates[i].getGraspTop();
    tf_hand.linear() = hand_frame;

    AntiPodalGrasp grasp;
    grasp.hand_pose = tf_hand;
    antipodal_grasps.push_back(grasp);

    AntiPodalGrasp flip_grasp;
    flip_grasp.hand_pose = tf_hand *
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
    antipodal_grasps.push_back(flip_grasp);
  }

  return antipodal_grasps;
}
