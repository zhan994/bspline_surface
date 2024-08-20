#include "bspline_surface.h"

BspSurface::BspSurface(int k, double grid_size)
    : ku_(k), kv_(k), grid_size_(grid_size), ct_pts_pcl_(new PointCloud) {
  debug_ = true;
  std::cout << "Cons. BSpline Surface." << std::endl;
}

BspSurface::BspSurface(int ku, int kv, double grid_size)
    : ku_(ku), kv_(kv), grid_size_(grid_size), ct_pts_pcl_(new PointCloud) {
  debug_ = true;
  std::cout << "Cons. BSpline Surface." << std::endl;
}

void BspSurface::SetData(const PointCloud::Ptr &input, double perc, int kn) {
  std::cout << "Init,,," << std::endl;
  ct_pts_.clear();
  ct_pts_pcl_->clear();

  // step: 计算数据范围
  PointT max_pt, min_pt;
  pcl::getMinMax3D(*input, min_pt, max_pt);

  if (debug_) {
    std::cout << "min_x: " << min_pt.x << " "
              << "max_x: " << max_pt.x << std::endl;
    std::cout << "min_y: " << min_pt.y << " "
              << "max_y: " << max_pt.y << std::endl;
  }

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
      ct_pt.y = (j + 0.5) * grid_size_ + min_pt.y;
      if (!pts.empty()) {
        std::sort(pts.begin(), pts.end(),
                  [](PointT a, PointT b) { return a.z < b.z; });
        int sel_ind = pts.size() * perc;
        ct_pt.z = pts[sel_ind].z;
        ct_pts_pcl_->points.push_back(ct_pt);
      } else {
        // std::cout << "No data in this grid!!! Set infinity<double>"
        //           << std::endl;
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
    pt_2d.z = 0.;
    ct_pts_pcl_2d->points.push_back(pt_2d);
  }

  // step: 对于失效高度进行插值
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(ct_pts_pcl_2d);
  std::vector<int> indices(kn);
  std::vector<float> sq_dists(kn);
  for (int i = 0; i < ct_x_num_; ++i) {
    for (int j = 0; j < ct_y_num_; ++j) {
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

// 计算节点对应的ENU点
void BspSurface::KnotSpatio() {
  // 单行
  for (int j = kv_ - 1; j <= kv_ - 1; j++) {
    for (int i = ku_ - 1; i <= ct_x_num_; i++) {
      double u = knots_u_[i];
      double v = knots_v_[j];
      PointT temp = SampleUV(u, v);
      pts_knots_u_.push_back(temp);
    }
  }
  // 单列
  for (int i = ku_ - 1; i <= ku_ - 1; i++) {
    for (int j = kv_ - 1; j <= ct_y_num_; j++) {
      double u = knots_u_[i];
      double v = knots_v_[j];
      PointT temp = SampleUV(u, v);
      pts_knots_v_.push_back(temp);
    }
  }
}

int BspSurface::KnotId(const std::vector<double> &knots, int k, int n,
                       double t) {

  int L = 0;
  if (t >= knots[n + 1]) {
    L = n;
  } else if (t <= knots[k - 1]) {
    L = k - 1;
  } else {
    for (int i = k - 1; i <= n + 1; ++i) {
      if (t >= knots[i] && t < knots[i + 1]) {
        L = i;
        break;
      }
    }
  }

  if (L >= n + 1)
    L = n;

  return L;
}

PointT BspSurface::SampleXY(double sx, double sy) {
  std::cout << "sx: " << sx << std::endl;
  std::cout << "sy: " << sy << std::endl;

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

  // step: 确定sx、sy所处的节点向量区间
  int su_i, sv_i;
  for (int i = 0; i < pts_knots_u_.size() - 1; i++) {
    if (sx >= pts_knots_u_[i].x && sx < pts_knots_u_[i + 1].x) {
      su_i = i;
    }
  }

  for (int i = 0; i < pts_knots_v_.size() - 1; i++) {
    if (sy >= pts_knots_v_[i].y && sy < pts_knots_v_[i + 1].y) {
      sv_i = i;
    }
  }

  // step: 按距离比例取初值， loss较大时采用二分逼近
  double tolerance = 0.1;
  double epsilon = 0.001;
  double u_min = knots_u_[su_i + ku_ - 1]; // knot值
  double v_min = knots_v_[sv_i + kv_ - 1];
  double u_max = knots_u_[su_i + ku_];
  double v_max = knots_v_[sv_i + kv_];
  double u_span = u_max - u_min;
  double v_span = v_max - v_min;
  float u_res = (sx - pts_knots_u_[su_i].x) /
                (pts_knots_u_[su_i + 1].x - pts_knots_u_[su_i].x) * u_span;
  float v_res = (sy - pts_knots_v_[sv_i].y) /
                (pts_knots_v_[sv_i + 1].y - pts_knots_v_[sv_i].y) * v_span;

  int count = 1;
  PointT init_pt = SampleUV(u_min + u_res, v_min + v_res);
  if (SquareError(sx, sy, init_pt.x, init_pt.y) <
      tolerance) { // 若loss足够小，直接返回拟合点
    if (debug_) {
      std::cout << "x_fit: " << init_pt.x << std::endl;
      std::cout << "y_fit: " << init_pt.y << std::endl;
    }

    ret.z = init_pt.z;
    return ret;
  } else { // 重新初始化u_min, v_min, u_max, v_max
    if (init_pt.x < sx) {
      u_min = u_min + u_res;
    } else {
      u_max = u_min + u_res;
    }

    if (init_pt.y < sy) {
      v_min = v_min + v_res;
    } else {
      v_max = v_min + v_res;
    }
  }

  while ((u_max - u_min > epsilon) && (v_max - v_min > epsilon)) {
    double u_mid = (u_min + u_max) / 2.0;
    double v_mid = (v_min + v_max) / 2.0;

    double x_fit, y_fit;
    PointT temp = SampleUV(u_mid, v_mid);
    x_fit = temp.x;
    y_fit = temp.y;

    double error = SquareError(sx, sy, x_fit, y_fit);
    if (error < tolerance) {
      if (debug_) {
        std::cout << "x_fit: " << x_fit << std::endl;
        std::cout << "y_fit: " << y_fit << std::endl;
        std::cout << "iterations: " << count << std::endl;
      }

      ret.z = temp.z;
      return ret;
    }

    count++;
    // 更新区间
    if (x_fit < sx) {
      u_min = u_mid;
    } else {
      u_max = u_mid;
    }

    if (y_fit < sy) {
      v_min = v_mid;
    } else {
      v_max = v_mid;
    }
  }

  std::cout << "Error Occurs !!! Return Is Invalid." << std::endl;
  ret.z = init_pt.z;
  return ret;
}

PointT BspSurface::SampleUV(double su, double sv) {

  // step: 确定su, sv所处的节点向量区间下边界索引
  int u_id = KnotId(knots_u_, ku_, ct_x_num_ - 1, su);
  int v_id = KnotId(knots_v_, kv_, ct_y_num_ - 1, sv);

  std::vector<PointT> v_constant(ct_x_num_);

  // step: 根据u_id仅更新与拟合位置相关的控制点
  for (int i = u_id - ku_ + 1; i <= u_id; i++) {
    v_constant[i] = Sample(ct_pts_[i], knots_v_, sv, v_id);
  }

  return Sample(v_constant, knots_u_, su, u_id);
}

PointT BspSurface::Sample(const std::vector<PointT> ct_pts,
                          const std::vector<double> knots, double t, int L) {

  int n = ct_pts.size() - 1;
  int k = knots.size() - ct_pts.size();

  std::vector<PointT> temp(k);
  for (int i = 0; i < k; ++i) {
    temp[i] = ct_pts[i + L - k + 1];
  }

  // de-BoorCox
  for (int r = 1; r <= k - 1; ++r) {
    for (int i = 0; i <= k - r - 1; ++i) {
      int index = L - k + 1 + r;
      double factor = 0;
      if (knots[index + i + k - r] != knots[index + i]) {
        factor = (t - knots[index + i]) /
                 (knots[index + i + k - r] - knots[index + i]);
      }

      temp[i].x = factor * temp[i + 1].x + (1 - factor) * temp[i].x;
      temp[i].y = factor * temp[i + 1].y + (1 - factor) * temp[i].y;
      temp[i].z = factor * temp[i + 1].z + (1 - factor) * temp[i].z;
    }
  }

  return temp[0];
}

double BspSurface::GetHeight(double x, double y) {
  PointT ret = SampleXY(x, y);
  return ret.z;
}

void BspSurface::GetSurface(PointCloud::Ptr &surface, double step) {
  int m = static_cast<int>((knots_u_[ct_x_num_] - knots_u_[ku_ - 1]) / step);
  int n = static_cast<int>((knots_v_[ct_y_num_] - knots_v_[kv_ - 1]) / step);

  for (int i = 0; i <= m; ++i) {
    for (int j = 0; j <= n; ++j) {
      float u = 0, v = 0;
      if (i == m) {
        u = knots_u_[ct_x_num_];
        v = knots_v_[kv_ - 1] + j * step;

      } else if (j == n) {
        u = knots_u_[ku_ - 1] + i * step;
        v = knots_v_[ct_y_num_];

      } else {
        u = knots_u_[ku_ - 1] + i * step;
        v = knots_v_[kv_ - 1] + j * step;
      }
      PointT temp = SampleUV(u, v);
      surface->points.push_back(temp);
    }
  }
}

PointCloud::Ptr BspSurface::GetCtrlPts() { return ct_pts_pcl_; }