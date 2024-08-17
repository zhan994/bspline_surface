/**
 * \file utils.h
 * \author Zhihao Zhan (zhanzhihao_dt@163.com)
 * \brief 工具函数
 * \version 0.1
 * \date 2024-08-07
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef UTILS_H
#define UTILS_H

#include <chrono>
#include <cmath>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ PointT;
// typedef pcl::PointXYZINormal PointT;
typedef pcl::PointCloud<PointT> PointCloud;

/**
 * \brief 计算平方差
 *
 * \param sx 查询点x
 * \param sy 查询点y
 * \param fx 拟合点x
 * \param fy 拟合点y
 * \return 返回误差平方
 */
double SquareError(double sx, double sy, double fx, double fy);

/**
 * \brief 计时
 *
 * \param t_end
 * \param t_begin
 * \return double
 */
double time_inc(std::chrono::high_resolution_clock::time_point &t_end,
                std::chrono::high_resolution_clock::time_point &t_begin);

/**
 * \brief 体素滤波
 *
 * \param input_cloud
 * \param cloud_filtered
 * \param voxel_size 体素大小
 */
void VoxelDownSample(const PointCloud::Ptr &input_cloud,
                     PointCloud::Ptr &cloud_filtered, float voxel_size);

/**
 * \brief 去离群点
 *
 * \param input_cloud
 * \param cloud_filtered
 * \param mean_k
 * \param std_thresh
 */
void StatisticalRemoveOutlier(const PointCloud::Ptr &input_cloud,
                              PointCloud::Ptr &cloud_filtered, int mean_k,
                              float std_thresh);


/**
 * \brief Get the Average Height object
 * 
 * \param cloud 
 * \return float 
 */
float GetAverageHeight(const PointCloud::Ptr &cloud);

#endif // UTILS_H