#pragma once
#include <ros/ros.h>
#include <queue>
#include <Eigen/Geometry>
#include <algorithm>

struct Ekf {
  typedef std::shared_ptr<Ekf> Ptr;
  int id;
  double dt;
  ros::Time last_update_stamp_;
  int age, update_num;
  Eigen::MatrixXd A, B, C;
  Eigen::MatrixXd Qt, Rt;
  Eigen::MatrixXd Sigma, K;
  Eigen::VectorXd x;

  std::deque<Eigen::MatrixXd> InnoCov_list;
  bool adaptive_q{true};
  int innovation_window{20};
  double q_min{1e-4};
  double q_max{10.0};
  double vmax{4.0};

  Ekf(double _dt) : dt(_dt) {
    A.setIdentity(6, 6);
    Sigma.setZero(6, 6);
    B.setZero(6, 6);
    C.setZero(6, 6);
    A(0, 3) = dt;
    A(1, 4) = dt;
    A(2, 5) = dt;
    double t2 = dt * dt / 2;
    B(0, 0) = t2;
    B(1, 1) = t2;
    B(2, 2) = t2;
    B(3, 3) = dt;
    B(4, 4) = dt;
    B(5, 5) = dt;
    C(0, 0) = 1;
    C(1, 1) = 1;
    C(2, 2) = 1;
    C(3, 3) = 1;
    C(4, 4) = 1;
    C(5, 5) = 1;
    K = C;
    Qt.setIdentity(6, 6);
    Rt.setIdentity(6, 6);
    Qt(0, 0) = 0.1;
    Qt(1, 1) = 0.1;
    Qt(2, 2) = 0.1;
    Qt(3, 3) = 0.1;
    Qt(4, 4) = 0.1;
    Qt(5, 5) = 0.1;
    Rt(0, 0) = 0.09;
    Rt(1, 1) = 0.09;
    Rt(2, 2) = 0.09;
    Rt(3, 3) = 0.4;
    Rt(4, 4) = 0.4;
    Rt(5, 5) = 0.4;
    x.setZero(6);
  }

  inline void setDt(double _dt) {
    dt = _dt;
    A.setIdentity(6, 6);
    A(0, 3) = dt;
    A(1, 4) = dt;
    A(2, 5) = dt;
    B.setZero(6, 6);
    const double t2 = dt * dt / 2;
    B(0, 0) = t2;
    B(1, 1) = t2;
    B(2, 2) = t2;
    B(3, 3) = dt;
    B(4, 4) = dt;
    B(5, 5) = dt;
  }

  inline void configure(double q_pos, double q_vel, double r_pos, double r_vel,
                        bool adaptive, int window, double min_q, double max_q,
                        double max_vel) {
    adaptive_q = adaptive;
    innovation_window = std::max(1, window);
    q_min = min_q;
    q_max = std::max(min_q, max_q);
    vmax = max_vel;
    Qt.setIdentity(6, 6);
    Rt.setIdentity(6, 6);
    Qt.diagonal().head(3).setConstant(q_pos);
    Qt.diagonal().tail(3).setConstant(q_vel);
    Rt.diagonal().head(3).setConstant(r_pos);
    Rt.diagonal().tail(3).setConstant(r_vel);
  }

  inline void predict() {
    x = A * x;
    Sigma = A * Sigma * A.transpose() + Qt;
    return;
  }
  inline void reset(const Eigen::Vector3d& z, int id_) {
    x.head(3) = z;
    x.tail(3).setZero();
    Sigma.setZero();
    last_update_stamp_ = ros::Time::now();
    age = 1;
    update_num = 0;
    id = id_;
  }
  inline bool checkValid(const Eigen::Vector3d& z1, const Eigen::Vector3d& z2) {
    Eigen::VectorXd z(6);
    z << z1, z2;
    Eigen::MatrixXd K_tmp = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    Eigen::VectorXd x_tmp = x + K_tmp * (z - C * x);
    if (x_tmp.tail(3).norm() > vmax) {
      return false;
    } else {
      return true;
    }
  }
  inline void update(const Eigen::Vector3d& z1, const Eigen::Vector3d& z2) {
    Eigen::VectorXd z(6);
    z << z1, z2;
    Eigen::VectorXd innovation = z - C * x;
    Eigen::MatrixXd innovation_cov = C * Sigma * C.transpose() + Rt;
    K = Sigma * C.transpose() * innovation_cov.inverse();
    if (adaptive_q) {
      InnoCov_list.emplace_back(innovation * innovation.transpose());
      while ((int)InnoCov_list.size() > innovation_window) {
        InnoCov_list.pop_front();
      }
      if ((int)InnoCov_list.size() == innovation_window) {
        Eigen::MatrixXd actual_cov = Eigen::MatrixXd::Zero(6, 6);
        for (const auto& cov : InnoCov_list) {
          actual_cov += cov;
        }
        actual_cov /= static_cast<double>(InnoCov_list.size());
        for (int i = 0; i < 6; ++i) {
          const double extra_var = std::max(0.0, actual_cov(i, i) - innovation_cov(i, i));
          const double q = 0.8 * Qt(i, i) + 0.2 * (Qt(i, i) + extra_var);
          Qt(i, i) = std::min(q_max, std::max(q_min, q));
        }
      }
    }
    x = x + K * innovation;
    Sigma = Sigma - K * C * Sigma;
    
    last_update_stamp_ = ros::Time::now();
    update_num ++;
  }
  inline void constrainVerticalVelocity(double scale, double max_abs_vz) {
    x(5) *= scale;
    if (max_abs_vz >= 0.0) {
      x(5) = std::min(max_abs_vz, std::max(-max_abs_vz, x(5)));
    }
  }
  inline const Eigen::Vector3d pos() const {
    return x.head(3);
  }
  inline const Eigen::Vector3d vel() const {
    return x.tail(3);
  }
  inline const Eigen::MatrixXd& cov() const {
    return Sigma;
  }
};
