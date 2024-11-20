#include "bspline_surface.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

void VisualizePointCloudV2(PointCloud::Ptr downsample_cloud,
                           PointCloud::Ptr ctp_cloud,
                           PointCloud::Ptr sur_cloud) {
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);

  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_handler(
      downsample_cloud, 255, 255, 255);
  viewer->addPointCloud(downsample_cloud, cloud_color_handler,
                        "downsample_points");

  pcl::visualization::PointCloudColorHandlerCustom<PointT>
      control_color_handler(ctp_cloud, 255, 0, 0);
  viewer->addPointCloud(ctp_cloud, control_color_handler, "ctp_cloud");

  // pcl::visualization::PointCloudColorHandlerCustom<PointT> surf_color_handler(
  //     sur_cloud, 0, 255, 0);
  // viewer->addPointCloud(sur_cloud, surf_color_handler, "sur_cloud");

  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "downsample_points");
  // viewer->setPointCloudRenderingProperties(
  //     pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sur_cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "ctp_cloud");

  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
  }
}

int main(int argc, char **argv) {

  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <input_pcd_file>" << std::endl;
    return -1;
  }

  PointCloud::Ptr cloud(new PointCloud);
  if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud) == -1) {
    PCL_ERROR("Couldn't read file %s \n", argv[1]);
    return -1;
  }

  PointCloud::Ptr filtered_cloud(new PointCloud);
  PointCloud::Ptr filtered_ground(new PointCloud);

  VoxelDownSample(cloud, filtered_cloud, 1.0);

  StatisticalRemoveOutlier(filtered_cloud, filtered_ground, 10, 2.0);

  std::cout << "cloud size is: " << filtered_ground->points.size() << std::endl;

  auto start = std::chrono::high_resolution_clock::now();

  BspSurface bsp_fit(3, 10.0);
  bsp_fit.SetData(filtered_ground, 0.95);

  auto end = std::chrono::high_resolution_clock::now();

  double ptc = bsp_fit.GetHeight(137.99, -50.81); // test

  auto end1 = std::chrono::high_resolution_clock::now();

  double duration = time_inc(end, start);
  std::cout << "Bspline-fitting Initialization: " << duration << " milliseconds"
            << std::endl;

  double duration1 = time_inc(end1, end);
  std::cout << "Query Bspline-fitting Point: " << duration1 << " milliseconds"
            << std::endl;

  // std::cout << "x: " << ptc.x << std::endl;
  // std::cout << "y: " << ptc.y << std::endl;
  std::cout << "z: " << ptc << std::endl;

  // -----------------test--------------------
  PointCloud::Ptr surf_cloud(new PointCloud);
  bsp_fit.GetSurface(surf_cloud);
  PointCloud::Ptr ctp_cloud = bsp_fit.GetCtrlPts();
  VisualizePointCloudV2(filtered_ground, ctp_cloud, surf_cloud);

  return 0;
}