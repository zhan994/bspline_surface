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

#include <cmath>
#include <iostream>
#include <pcl/common/common.h>
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

#endif // UTILS_H