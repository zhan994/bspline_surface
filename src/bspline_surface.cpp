#include "bspline_surface.h"

BspSurface::BspSurface(int k, double grid_size)
    : ku_(k), kv_(k), grid_size_(grid_size), ct_pts_pcl_(new PointCloud) {
  std::cout << "Cons. BSpline Surface." << std::endl;
}

void BspSurface::SetData(const PointCloud::Ptr &input, double perc, int kn) {
  std::cout << "Init,,," << std::endl;
  ct_pts_.clear();
  ct_pts_pcl_->clear();

  // step: 计算数据范围
  PointT max_pt, min_pt;
  pcl::getMinMax3D(*input, min_pt, max_pt);
  double range_x = max_pt.x - min_pt.x;
  double range_y = max_pt.y - min_pt.y;
  ct_x_num_ = std::floor(range_x / grid_size_) + 1;
  ct_y_num_ = std::floor(range_y / grid_size_) + 1;
  std::vector<std::vector<PointCloud>> grids(
      ct_x_num_, std::vector<PointCloud>(ct_y_num_));
  ct_pts_ = std::vector<std::vector<PointT>>(ct_x_num_,
                                             std::vector<PointT>(ct_y_num_));

  // step: 点云分组，对每组点云进行高度提取
  int invalid = 0;
  for (const auto &pt : *input) {
    int x_ind = std::floor((pt.x - min_pt.x) / grid_size_);
    int y_ind = std::floor((pt.y - min_pt.y) / grid_size_);
    if (x_ind >= 0 && x_ind < ct_x_num_ && y_ind >= 0 && y_ind < ct_y_num_)
      grids[x_ind][y_ind].points.push_back(pt);
  }

  for (int i = 0; i < ct_x_num_; ++i) {
    for (int j = 0; j < ct_y_num_; ++j) {
      auto &pts = grids[i][j].points;
      PointT ct_pt;
      ct_pt.x = (i + 0.5) * grid_size_ + min_pt.x;
      ct_pt.y = (i + 0.5) * grid_size_ + min_pt.y;
      if (!pts.empty()) {
        std::sort(pts.begin(), pts.end(),
                  [](PointT a, PointT b) { return a.z < b.z; });
        int sel_ind = pts.size() * perc;
        ct_pt.z = pts[sel_ind].z;
        ct_pts_pcl_->points.push_back(ct_pt);
      } else {
        std::cout << "No data in this grid!!! Set infinity<double>"
                  << std::endl;
        ct_pt.z = std::numeric_limits<double>::infinity();
        invalid++;
      }

      ct_pts_[i][j] = ct_pt;
    }
  }

  // step: 对空缺的控制点进行插值
  if (invalid > 0)
    KdInterplation(kn);

  // step: 均匀有理节点向量
  KnotVector();
  KnotSpatio();

  std::cout << "Finish Init." << std::endl;
}

void BspSurface::KdInterplation(int kn) {
  // step: 提取2d点
  PointCloud::Ptr ct_pts_pcl_2d(new PointCloud);
  for (const auto &pt : *ct_pts_pcl_) {
    PointT pt_2d;
    pt_2d.x = pt.x;
    pt_2d.y = pt.y;
    pt_2d.z = pt.z;
    ct_pts_pcl_2d->points.push_back(pt_2d);
  }

  // step: 对于失效高度进行插值
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(ct_pts_pcl_2d);
  std::vector<int> indices(kn);
  std::vector<float> sq_dists(kn);
  for (int i = 0; i < ct_x_num_; ++i) {
    for (int j = 0; i < ct_y_num_; ++j) {
      PointT tgt_pt = ct_pts_[i][j];
      if (std::isinf(tgt_pt.z)) {
        tgt_pt.z = 0.0;
        if (kdtree.nearestKSearch(tgt_pt, kn, indices, sq_dists) > 0) {
          double h_sum = 0;
          for (const auto &ind : indices)
            h_sum += ct_pts_pcl_->points.at(ind).z;

          ct_pts_[i][j].z = h_sum / kn;
        }
      }
    }
  }
}

void BspSurface::KnotVector() {
  int knots_u_num = ct_x_num_ + ku_; // 控制点数(n+1)+阶数k
  int knots_v_num = ct_y_num_ + kv_;
  knots_u_.clear();
  knots_v_.clear();
  knots_u_.resize(knots_u_num);
  knots_v_.resize(knots_v_num);

  for (int i = 0; i < ku_; ++i) {
    knots_u_[i] = 0.0f;
    knots_u_[knots_u_num - 1 - i] = static_cast<float>(ct_x_num_ - ku_ + 1);
  }

  for (int i = ku_; i < knots_u_num - ku_; ++i) {
    knots_u_[i] = static_cast<float>(i - ku_ + 1);
  }

  for (int i = 0; i < kv_; ++i) {
    knots_v_[i] = 0.0f;
    knots_v_[knots_v_num - 1 - i] = static_cast<float>(ct_y_num_ - kv_ + 1);
  }
  for (int i = kv_; i < knots_v_num - kv_; ++i) {
    knots_v_[i] = static_cast<float>(i - kv_ + 1);
  }
}

// TODO
void BspSurface::KnotSpatio() {}

PointT BspSurface::Sample(double sx, double sy) {
  PointT ret;
  ret.x = sx;
  ret.y = sy;

  // step: 判断采样点是否在控制点区间内
  PointT ct_min_pt = ct_pts_[0][0];
  PointT ct_max_pt = ct_pts_[ct_x_num_ - 1][ct_y_num_ - 1];
  if (sx < ct_min_pt.x || sy < ct_min_pt.y || sx > ct_max_pt.x ||
      sy > ct_max_pt.y) {
    std::cout << "Err. invalid sample" << std::endl;
    ret.z = std::numeric_limits<double>::infinity();
    return ret;
  }

  // step: 采样高度
}

Point BspSurface::Sample(double su, double sv) {}

PointT BspSurface::Sample(const std::vector<PointT> ct_pts,
                          const std::vector<double> knots, double t);

double BspSurface::GetHeight(double x, double y) {}

void BspSurface::GetSurface(PointCloud::Ptr &surface) {}