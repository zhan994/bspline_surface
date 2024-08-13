/**
 * \file bspline_surface.h
 * \author Zhihao Zhan (zhanzhihao_dt@163.com)
 * \brief bspline拟合的表面模型
 * \version 0.1
 * \date 2024-08-07
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef BSPLINE_SURFACE_H
#define BSPLINE_SURFACE_H

#include "utils.h"

class BspSurface {
public:
  /**
   * \brief Construct a new Bsp Surface object
   *
   * \param k 阶
   * \param grid_size 控制点采样区间
   */
  BspSurface(int k, double grid_size);

  /**
   * \brief 输入数据
   *
   * \param input 输入点云
   * \param perc 高度选取百分比
   * \param kn kdtree近邻点数
   */
  void SetData(const PointCloud::Ptr &input, double perc, int kn);

  /**
   * \brief 获取高度
   *
   * \param x 位置
   * \param y 
   * \return double 对应位置的高度
   */
  double GetHeight(double x, double y);

private:
  /**
   * \brief 通过kdtree来插补控制点
   *
   * \param k 近邻点个数
   */
  void KdInterplation(int k);

  /**
   * \brief 生成节点向量
   *
   */
  void KnotVector();

  /**
   * \brief 采样
   * 
   * \param sx 采样点位置
   * \param sy 
   * \return PointT 采样点
   */
  PointT Sample(double sx, double sy);

  /**
   * \brief Get the Span object
   * 
   * \param sx 
   * \param sy 
   * \param su 
   * \param sv 
   */
  void GetUV(double sx, double sy, double& su, double& sv);

  int ku_, kv_;                 // uv阶
  std::vector<double> knots_u_; // u向节点向量
  std::vector<double> knots_v_; // v向节点向量

  double grid_size_;
  std::vector<std::vector<PointT>> ct_pts_; // 控制点
  PointCloud::Ptr ct_pts_pcl_;              // 控制点的点云
  int ct_x_num_, ct_y_num_;                 // xy控制点数
};

#endif // BSPLINE_SURFACE_H