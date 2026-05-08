#ifndef __MPC_CONTROLLER_H
#define __MPC_CONTROLLER_H

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>
#include <queue>
#include <deque>

#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <traj_utils/PolyTraj.h>

#include "controller.h"
#include "polynomial_trajectory.h"

struct oneTraj_Data_t
{
public:
	ros::Time traj_start_time{0};
	ros::Time traj_end_time{0};
	Trajectory traj;
	Trajectory yaw_traj;
};

class Trajectory_Data_t
{
public:
	ros::Time total_traj_start_time{0};
	ros::Time total_traj_end_time{0};
	int exec_traj = 0;
	std::deque<oneTraj_Data_t> traj_queue;

	Trajectory_Data_t()
	{
		total_traj_start_time = ros::Time(0);
		total_traj_end_time = ros::Time(0);
		exec_traj = 0;
	}

	void feed_from_traj_utils(const traj_utils::PolyTrajConstPtr &pMsg);
};

class MpcWrapper;

class OMMPCControl
{
public:
	OMMPCControl(Parameter_t &param);
	void feedTrajectory(const traj_utils::PolyTrajConstPtr &pMsg);
	quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des,
												  const Odom_Data_t &odom,
												  const Imu_Data_t &imu,
												  Controller_Output_t &u,
												  bool allow_direct_polytraj);
	bool estimateThrustModel(const Eigen::Vector3d &est_a,
							 const Parameter_t &param);
	void resetThrustMapping(void);
	void clearTrajectory();

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	Parameter_t param_;
	quadrotor_msgs::Px4ctrlDebug debug_msg_;
	std::queue<std::pair<ros::Time, double>> timed_thrust_;

	std::shared_ptr<MpcWrapper> mpc_wrapper_;
	std::vector<Eigen::SparseMatrix<double>> Fx_;
	std::vector<Eigen::SparseMatrix<double>> Fu_;
	std::vector<Eigen::VectorXd> u_lb_;
	std::vector<Eigen::VectorXd> u_ub_;
	Trajectory_Data_t trajectory_data_;

	Controller_Output_t last_u_;

	const double rho2_ = 0.998;
	double thr2acc_;
	double P_;
	double last_yaw_;
	double last_yaw_dot_;

	void initializeMpc();
	bool execMPC(const Odom_Data_t &odom, Controller_Output_t &u);
	void setHoverReference(const Eigen::Vector4d &quad_pose);
	void setTextReference(const std::vector<Eigen::Vector3d> &quad_positions,
							const std::vector<Eigen::Vector3d> &quad_velocities,
							const Odom_Data_t &odom,
							const double start_yaw,
							const std::vector<double> &yaws);
	void setTrajectoryReference(const Trajectory &traj,
						  const double tstart,
						  const double start_yaw,
						  const Trajectory &yaw_traj,
						  const Odom_Data_t &odom);
	void setReferenceFromPx4cmd(const Desired_State_t &des, const Odom_Data_t &odom);
	void setStateMatricesAndBounds(const int i,
							  const Eigen::Quaterniond &q,
							  const Eigen::Vector3d &omg,
							  const double t_step,
							  const double thracc);
	double angleDiff(double a, double b);
	void calculateYaw(const Eigen::Vector3d &vel, const double dt, double &yaw, double &yawdot);
	void computeFlatInputwithHopfFibration(const Eigen::Vector3d &thr_acc,
										const Eigen::Vector3d &jer,
										const double &yaw,
										const double &yawd,
										const Eigen::Quaterniond &att_est,
										Eigen::Quaterniond &att,
										Eigen::Vector3d &omg) const;
};

#endif
