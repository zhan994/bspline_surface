#include "utils.h"

double SquareError(double sx, double sy, double fx, double fy) {
  return std::sqrt((fx - sx) * (fx - sx) + (fy - sy) * (fy - sy));
}

double time_inc(std::chrono::high_resolution_clock::time_point &t_end,
                std::chrono::high_resolution_clock::time_point &t_begin) {

  return std::chrono::duration_cast<std::chrono::duration<double>>(t_end -
                                                                   t_begin)
             .count() *
         1000;
}

void VoxelDownSample(const PointCloud::Ptr &input_cloud,
                     PointCloud::Ptr &cloud_filtered, float voxel_size) {
  // Create the filtering object
  pcl::VoxelGrid<PointT> sor;

  sor.setInputCloud(input_cloud);
  sor.setLeafSize(voxel_size, voxel_size, voxel_size); // voxel_size
  // sor.setLeafSize(0.5f, 0.5f, 0.5f);  // voxel_size
  sor.filter(*cloud_filtered);
}

void StatisticalRemoveOutlier(const PointCloud::Ptr &input_cloud,
                              PointCloud::Ptr &cloud_filtered, int mean_k,
                              float std_thresh) {
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(input_cloud);
  sor.setMeanK(mean_k);               // default: 10 越大越严格
  sor.setStddevMulThresh(std_thresh); // default: 2.0 越小越严格
  sor.filter(*cloud_filtered);
}

float GetAverageHeight(const PointCloud::Ptr &cloud) {

  float temp_height = 0.0;
  for (auto &point : cloud->points) {
    temp_height += point.z;
  }
  return temp_height / cloud->points.size();
}