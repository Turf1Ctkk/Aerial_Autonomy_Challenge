#ifndef __POSITION_ADRC_H
#define __POSITION_ADRC_H

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>

class PositionAdrc
{
public:
  struct Config
  {
    std::array<bool, 3> axis_enable{{true, true, true}};
    std::array<double, 3> beta1{{1.0, 1.0, 1.0}};
    std::array<double, 3> beta2{{1.0, 1.0, 1.0}};
    std::array<double, 3> beta3{{1.0, 1.0, 1.0}};
    std::array<double, 3> b0{{1.0, 1.0, 1.0}};
    std::array<double, 3> fal_delta{{0.01, 0.01, 0.01}};
    std::array<double, 3> leak_rate{{0.0, 0.0, 0.0}};
    std::array<double, 3> max_dist_acc{{0.0, 0.0, 0.0}};
  };

  PositionAdrc() = default;

  void configure(const Config &config)
  {
    config_ = config;
    for (int i = 0; i < 3; ++i)
    {
      config_.b0[i] = std::abs(config_.b0[i]) > 1.0e-6 ? config_.b0[i] : 1.0;
      config_.fal_delta[i] = std::max(config_.fal_delta[i], 1.0e-6);
      config_.leak_rate[i] = std::max(0.0, config_.leak_rate[i]);
      eso_[i].enabled = config_.axis_enable[i];
      eso_[i].beta1 = std::max(0.0, config_.beta1[i]);
      eso_[i].beta2 = std::max(0.0, config_.beta2[i]);
      eso_[i].beta3 = std::max(0.0, config_.beta3[i]);
      eso_[i].b0 = config_.b0[i];
      eso_[i].fal_delta = config_.fal_delta[i];
      eso_[i].leak_rate = config_.leak_rate[i];
      eso_[i].max_dist_acc = config_.max_dist_acc[i];
    }
  }

  void reset(const Eigen::Vector3d &position, const Eigen::Vector3d &velocity)
  {
    for (int i = 0; i < 3; ++i)
    {
      eso_[i].z1 = position(i);
      eso_[i].z2 = velocity(i);
      eso_[i].z3 = 0.0;
    }
    dist_acc_.setZero();
    initialized_ = true;
  }

  bool initialized() const
  {
    return initialized_;
  }

  Eigen::Vector3d update(const Eigen::Vector3d &position,
                         const Eigen::Vector3d &velocity,
                         const Eigen::Vector3d &control_acc,
                         const double dt)
  {
    if (!initialized_)
    {
      reset(position, velocity);
      return Eigen::Vector3d::Zero();
    }

    if (!std::isfinite(dt) || dt <= 0.0)
    {
      return dist_acc_;
    }

    for (int i = 0; i < 3; ++i)
    {
      if (!eso_[i].enabled)
      {
        eso_[i].z1 = position(i);
        eso_[i].z2 = velocity(i);
        eso_[i].z3 = 0.0;
        dist_acc_(i) = 0.0;
        continue;
      }

      updateEso(position(i), control_acc(i), eso_[i], dt);
      dist_acc_(i) = eso_[i].z3 / eso_[i].b0;

      if (eso_[i].max_dist_acc > 0.0)
      {
        dist_acc_(i) = std::max(-eso_[i].max_dist_acc,
                                std::min(eso_[i].max_dist_acc, dist_acc_(i)));
        eso_[i].z3 = dist_acc_(i) * eso_[i].b0;
      }
    }

    return dist_acc_;
  }

  Eigen::Vector3d disturbanceAcc() const
  {
    return dist_acc_;
  }

private:
  struct ExtendedStateObserver
  {
    double z1{0.0};
    double z2{0.0};
    double z3{0.0};
    double beta1{1.0};
    double beta2{1.0};
    double beta3{1.0};
    double b0{1.0};
    double fal_delta{0.01};
    double leak_rate{0.0};
    double max_dist_acc{0.0};
    bool enabled{true};
  };

  double fal(const double error, const double alpha, const double delta) const
  {
    const double abs_error = std::abs(error);
    if (abs_error <= delta)
    {
      return error / std::pow(delta, 1.0 - alpha);
    }

    return std::pow(abs_error, alpha) * (error >= 0.0 ? 1.0 : -1.0);
  }

  void updateEso(const double measurement,
                 const double control_input,
                 ExtendedStateObserver &eso,
                 const double dt) const
  {
    const double error = eso.z1 - measurement;
    eso.z1 += dt * (eso.z2 - eso.beta1 * error);
    eso.z2 += dt * (eso.z3 - eso.beta2 * fal(error, 0.5, eso.fal_delta) + eso.b0 * control_input);
    eso.z3 += dt * (-eso.beta3 * fal(error, 0.25, eso.fal_delta) - eso.leak_rate * eso.z3);
  }

  Config config_;
  ExtendedStateObserver eso_[3];
  Eigen::Vector3d dist_acc_{Eigen::Vector3d::Zero()};
  bool initialized_{false};
};

#endif
