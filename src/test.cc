#include "grasp_gen/anti_podal_grasp.h"
#include <pcl/filters/voxel_grid.h>

template <typename T>
typename pcl::PointCloud<T>::Ptr
DownSample(const typename pcl::PointCloud<T>::ConstPtr &cloud,
           double leaf_size = 0.002) {
  pcl::VoxelGrid<T> grid;
  grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  grid.setInputCloud(cloud);

  typename pcl::PointCloud<T>::Ptr tmp =
      boost::make_shared<typename pcl::PointCloud<T>>();
  grid.filter(*tmp);
  return tmp;
}

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cout << "need to in put cloud\n";
    exit(-1);
  }

  AntiPodalGraspPlanner grasp_planner(GRASP_PARAM_PATH);

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
  pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(argv[1], *cloud);

  cloud = DownSample<pcl::PointXYZRGBNormal>(cloud, 0.004),

  grasp_planner.SetInputCloud(cloud);
  auto all_grasps =
    grasp_planner.GenerateAntipodalGrasp();


  while(true)
    ;

  return 0;
}
