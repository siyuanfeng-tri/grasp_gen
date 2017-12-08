#pragma once

#include <gpg/candidates_generator.h>
#include <gpg/config_file.h>
#include <gpg/hand_search.h>
#include <memory>
#include <string>
#include <vector>

struct AntiPodalGrasp {
  Eigen::Isometry3d hand_pose;
  double score;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
// This class implements anti-podal grasp geneneration. Mostly a wrapper over
// Andreas Ten Pan's gpg package with slight internal modification.

class AntiPodalGraspPlanner {
public:
  AntiPodalGraspPlanner(std::string config_file);
  void
  SetInputCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr &cloud);
  void SetInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
                     const pcl::PointCloud<pcl::Normal>::ConstPtr &normal);
  std::vector<AntiPodalGrasp> GenerateAntipodalGrasp();

private:
  std::unique_ptr<CandidatesGenerator> candidates_generator;
  std::unique_ptr<CloudCamera> cloud_camera;
};
