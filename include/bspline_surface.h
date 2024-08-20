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
   * \brief Construct a new Bsp Surface object
   *
   * \param ku
   * \param kv
   * \param grid_size
   */
  BspSurface(int ku, int kv, double grid_size);

  /**
   * \brief 输入数据
   *
   * \param input 输入点云
   * \param perc 高度选取百分比
   * \param kn kdtree近邻点数
   */
  void SetData(const PointCloud::Ptr &input, double perc, int kn = 3);

  /**
   * \brief Get the Ctrl Pts object
   * 
   * \return PointCloud::Ptr 
   */
  PointCloud::Ptr GetCtrlPts();

  /**
   * \brief 获取高度
   *
   * \param x enu位置
   * \param y
   * \return double 对应位置的高度
   */
  double GetHeight(double x, double y);

  /**
   * \brief 获得拟合面
   *
   * \param surface
   * \param step
   */
  void GetSurface(PointCloud::Ptr &surface, double step = 0.05);

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
   * \brief 节点向量的空间分布
   *
   */
  void KnotSpatio();

  /**
   * \brief 采样
   *  // note: 按距离比例取初值，loss较大时采用二分逼近
   *
   * \param sx 采样点空间x
   * \param sy 采样点空间y
   * \param su 对应采样点u向节点
   * \param sv 对应采样点u向节点
   * \return PointT 逼近后的采样点
   */
  PointT SampleXY(double sx, double sy);

  /**
   * \brief 采样
   *
   * \param sx 采样点位置
   * \param sy
   * \return PointT 采样点
   */
  PointT SampleUV(double su, double sv);

  /**
   * \brief 采样
   *
   * \param ct_pts 控制点
   * \param knots 节点向量
   * \param t 节点值
   * \param L节点区间下边界索引
   * \return PointT 采样点
   */
  PointT Sample(const std::vector<PointT> ct_pts,
                const std::vector<double> knots, double t, int L);

  /**
   * \brief 获得节点对应的节点向量区间下边界索引，仅更新节点相关的控制点
   *
   * \param n 控制点数-1
   * \param k 阶数
   * \param t 节点向量
   * \return int 节点向量区间的id
   */
  int KnotId(const std::vector<double> &knots, int k, int n, double t);

  int ku_, kv_;                                   // uv阶
  std::vector<double> knots_u_, knots_v_;         // uv向节点向量
  std::vector<PointT> pts_knots_u_, pts_knots_v_; // uv向节点向量对应点

  double grid_size_;
  std::vector<std::vector<PointT>> ct_pts_; // 控制点
  PointCloud::Ptr ct_pts_pcl_;              // 控制点的点云
  int ct_x_num_, ct_y_num_;                 // xy控制点数
  bool debug_;
};

#endif // BSPLINE_SURFACE_H