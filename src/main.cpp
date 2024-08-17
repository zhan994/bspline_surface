#include <iostream>
#include <vector>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <chrono>
#include <boost/filesystem.hpp>
#include <omp.h>
#include <queue>
#include <limits>
#include <cmath>
#include <iomanip>
#include <bspline_surface.h>


void VisualizePointCloudV2(pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr ctp_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr sur_cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(downsample_cloud, 255, 255, 255);
    viewer->addPointCloud(downsample_cloud, cloud_color_handler, "downsample_points");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> control_color_handler(ctp_cloud, 255, 0, 0);
    viewer->addPointCloud(ctp_cloud, control_color_handler, "ctp_cloud");

    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> surf_color_handler(sur_cloud, 0, 255, 0);
    // viewer->addPointCloud(sur_cloud, surf_color_handler, "sur_cloud");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "downsample_points");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sur_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "ctp_cloud");

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
}


float GetAverageHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    float temp_height = 0.0;
    for (auto& point: cloud->points) {
        temp_height += point.z;
    }
    return temp_height / cloud->points.size();
}


void VoxelDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud , pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, float voxel_size) {
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor;

    sor.setInputCloud(input_cloud);
    sor.setLeafSize(voxel_size, voxel_size, voxel_size);  // voxel_size     
    // sor.setLeafSize(0.5f, 0.5f, 0.5f);  // voxel_size 
    sor.filter(*cloud_filtered);
}

void StatisticalRemoveOutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud , pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, int mean_k, float std_thresh) {
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (input_cloud);
    sor.setMeanK(mean_k); // default: 10 越大越严格
    sor.setStddevMulThresh (std_thresh); // default: 2.0 越小越严格
    sor.filter(*cloud_filtered);
}

double time_inc(std::chrono::high_resolution_clock::time_point &t_end,
                std::chrono::high_resolution_clock::time_point &t_begin) {
  
  return std::chrono::duration_cast<std::chrono::duration<double>>(t_end -t_begin).count() * 1000;
}


int main(int argc, char** argv) {


    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <input_pcd_file>" << std::endl;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s \n", argv[1]);
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_ground(new pcl::PointCloud<pcl::PointXYZ>);

    VoxelDownSample(cloud, filtered_cloud, 1.0);

    StatisticalRemoveOutlier(filtered_cloud, filtered_ground, 50, 1.0);

    std::cout << "cloud size is: " << filtered_ground->points.size() << std::endl;

    auto start = std::chrono::high_resolution_clock::now();

    BspSurface bsp_fit(3, 10.0);
    bsp_fit.SetData(filtered_ground, 0.1);

    auto end = std::chrono::high_resolution_clock::now();

    double ptc = bsp_fit.GetHeight(137.99, -50.81); // test

    auto end1 = std::chrono::high_resolution_clock::now();

    double duration = time_inc(end, start);
    std::cout << "Bspline-fitting Initialization: " << duration << " milliseconds" << std::endl;  

    double duration1 = time_inc(end1, end);
    std::cout << "Query Bspline-fitting Point: " << duration1 << " milliseconds" << std::endl;  


    // std::cout << "x: " << ptc.x << std::endl;
    // std::cout << "y: " << ptc.y << std::endl;
    std::cout << "z: " << ptc << std::endl;

    // -----------------test--------------------

    // pcl::PointCloud<pcl::PointXYZ>::Ptr surf_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // bsp_fit.GetSurface(surf_cloud);

    // // // 可视化拟合点和控制点
    // pcl::PointCloud<pcl::PointXYZ>::Ptr ctp_cloud = bsp_fit.ct_pts_pcl_; // ct_pts_pcl_为private
    // VisualizePointCloudV2(filtered_ground, ctp_cloud, surf_cloud);

    return 0;
}