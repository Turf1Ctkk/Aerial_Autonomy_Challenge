#include "controller.h"

#include <algorithm>
#include <cmath>

using namespace std;

double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}

LinearControl::LinearControl(Parameter_t &param) : param_(param)
{
  int_e_v_.setZero();
  resetThrustMapping();
}

/* 
  compute u.thrust and u.q, controller gains and other parameters are in param_ 
*/
quadrotor_msgs::Px4ctrlDebug
LinearControl::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    Controller_Output_t &u)
{
  Eigen::Matrix3d Kp = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d Kv = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d Kvi = Eigen::Matrix3d::Zero();
  Kp(0, 0) = param_.gain.Kp0;
  Kp(1, 1) = param_.gain.Kp1;
  Kp(2, 2) = param_.gain.Kp2;
  Kv(0, 0) = param_.gain.Kv0;
  Kv(1, 1) = param_.gain.Kv1;
  Kv(2, 2) = param_.gain.Kv2;
  Kvi(0, 0) = param_.gain.Kvi0;
  Kvi(1, 1) = param_.gain.Kvi1;
  Kvi(2, 2) = param_.gain.Kvi2;

  const double yaw_curr = uav_utils::get_yaw_from_quaternion(odom.q);
  const double yaw_des = des.yaw;
  const Eigen::Matrix3d wRc = uav_utils::rotz(yaw_curr);
  const Eigen::Matrix3d cRw = wRc.transpose();

  const Eigen::Vector3d e_p = des.p - odom.p;
  const Eigen::Vector3d u_p = wRc * Kp * cRw * e_p;
  const Eigen::Vector3d e_v = des.v + u_p - odom.v;

  if (des.v.norm() > 1.0e-3)
  {
    int_e_v_.setZero();
  }
  else
  {
    const double ctrl_dt = 1.0 / std::max(param_.ctrl_freq_max, 1.0);
    for (int k = 0; k < 3; ++k)
    {
      if (std::fabs(e_v(k)) < 0.2)
      {
        int_e_v_(k) += e_v(k) * ctrl_dt;
      }
    }
  }

  const Eigen::Vector3d u_v_p = wRc * Kv * cRw * e_v;
  Eigen::Vector3d u_v_i = wRc * Kvi * cRw * int_e_v_;
  for (int k = 0; k < 3; ++k)
  {
    uav_utils::limit_range(u_v_i(k), 0.4);
  }

  Eigen::Vector3d des_acc = u_v_p + u_v_i + des.a + Eigen::Vector3d(0.0, 0.0, param_.gra);

  const double min_z_acc = 0.1 * param_.gra;
  const double max_z_acc = 2.0 * param_.gra;
  if (std::fabs(des_acc(2)) < 1.0e-6)
  {
    des_acc(2) = min_z_acc;
  }
  else if (des_acc(2) < min_z_acc)
  {
    des_acc *= min_z_acc / des_acc(2);
  }
  else if (des_acc(2) > max_z_acc)
  {
    des_acc *= max_z_acc / des_acc(2);
  }

  const double trigger_tilt = std::tan(uav_utils::toRad(50.0));
  const double max_tilt = std::tan(uav_utils::toRad(30.0));
  if (std::fabs(des_acc(0) / des_acc(2)) > trigger_tilt)
  {
    des_acc(0) = std::copysign(des_acc(2) * max_tilt, des_acc(0));
  }
  if (std::fabs(des_acc(1) / des_acc(2)) > trigger_tilt)
  {
    des_acc(1) = std::copysign(des_acc(2) * max_tilt, des_acc(1));
  }

  const Eigen::Vector3d z_b_des = des_acc.normalized();
  const Eigen::Vector3d y_c_des(-std::sin(yaw_des), std::cos(yaw_des), 0.0);
  Eigen::Vector3d x_b_des = y_c_des.cross(z_b_des);
  if (x_b_des.norm() < 1.0e-6)
  {
    x_b_des = Eigen::Vector3d(std::cos(yaw_des), std::sin(yaw_des), 0.0);
  }
  else
  {
    x_b_des.normalize();
  }
  const Eigen::Vector3d y_b_des = z_b_des.cross(x_b_des);

  Eigen::Matrix3d R_des1;
  R_des1 << x_b_des, y_b_des, z_b_des;

  Eigen::Matrix3d R_des2;
  R_des2 << -x_b_des, -y_b_des, z_b_des;

  const Eigen::Vector3d e1 = uav_utils::R_to_ypr(R_des1.transpose() * odom.q.toRotationMatrix());
  const Eigen::Vector3d e2 = uav_utils::R_to_ypr(R_des2.transpose() * odom.q.toRotationMatrix());
  const Eigen::Matrix3d R_des = (e1.norm() < e2.norm()) ? R_des1 : R_des2;

  const Eigen::Vector3d z_b_curr = odom.q.toRotationMatrix().col(2);
  const double collective_acc = des_acc.dot(z_b_curr);
  u.thrust = computeDesiredCollectiveThrustSignal(Eigen::Vector3d(0.0, 0.0, collective_acc));
  u.bodyrates = Eigen::Vector3d::Zero();
  const Eigen::Quaterniond q_des(R_des);
  u.q = (imu.q * odom.q.inverse() * q_des).normalized();

  //used for debug
  debug_msg_.des_p_x = des.p(0);
  debug_msg_.des_p_y = des.p(1);
  debug_msg_.des_p_z = des.p(2);
  
  debug_msg_.des_v_x = des.v(0);
  debug_msg_.des_v_y = des.v(1);
  debug_msg_.des_v_z = des.v(2);
  
  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);
  
  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();
  
  debug_msg_.des_thr = u.thrust;
  
  // Used for thrust-accel mapping estimation
  timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }
  return debug_msg_;
}

/*
  compute throttle percentage 
*/
double 
LinearControl::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc)
{
  double throttle_percentage(0.0);
  
  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = des_acc(2) / thr2acc_;

  return throttle_percentage;
}

bool 
LinearControl::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const Parameter_t &param)
{
  ros::Time t_now = ros::Time::now();
  while (timed_thrust_.size() >= 1)
  {
    // Choose data before 35~45ms ago
    std::pair<ros::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045) // 45ms
    {
      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
      // printf("skip, time_passed=%f\n", time_passed);
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust_.pop();
    
    /***********************************/
    /* Model: est_a(2) = thr1acc_ * thr */
    /***********************************/
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    P_ = (1 - K * thr) * P_ / rho2_;
    //printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
    //fflush(stdout);

    // debug_msg_.thr2acc = thr2acc_;
    return true;
  }
  return false;
}

void 
LinearControl::resetThrustMapping(void)
{
  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
  P_ = 1e6;
}
