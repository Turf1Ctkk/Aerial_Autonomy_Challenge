#include "mpc_controller.h"

#include <osqp/osqp.h>
#include <cmath>
#include <cstring>

#include "so3_math.h"

static constexpr int kNstep = 20;
static constexpr int kNx = 9;
static constexpr int kNstate = 10;
static constexpr int kNu = 4;

struct Solution
{
	std::vector<Eigen::VectorXd> delta_u;
	std::vector<Eigen::VectorXd> delta_x;
	double optimal_cost;
};

class MpcWrapper
{
public:
	MpcWrapper()
	{
		total_vars_ = (kNstep + 1) * kNx + kNstep * kNu;
		total_constraints_ = kNstep * kNx + kNx + kNstep * kNu;
	}

	~MpcWrapper()
	{
		free(P_data_);
		free(P_indices_);
		free(P_indptr_);
	}

	void setInitValue(const Eigen::VectorXd &x0)
	{
		for (int i = 0; i < kNx; ++i)
		{
			l_[i] = x0[i];
			u_[i] = x0[i];
		}
	}

	void setDesiredStart(const Eigen::VectorXd &xdes, const Eigen::VectorXd &udes)
	{
		x_des_start_ = xdes;
		u_des_start_ = udes;
	}

	void getDesiredStart(Eigen::VectorXd &xdes, Eigen::VectorXd &udes)
	{
		xdes = x_des_start_;
		udes = u_des_start_;
	}

	bool solve(Solution &sol)
	{
		OSQPData osqp_data;
		OSQPSettings osqp_settings;
		OSQPWorkspace *work = nullptr;

		osqp_data.n = total_vars_;
		osqp_data.m = total_constraints_;
		osqp_data.P = csc_matrix(total_vars_, total_vars_, P_nnz_, P_data_, P_indices_, P_indptr_);
		osqp_data.q = q_.data();
		osqp_data.A = csc_matrix(total_constraints_, total_vars_, A_nnz_, A_data_, A_indices_, A_indptr_);
		osqp_data.l = l_.data();
		osqp_data.u = u_.data();

		osqp_set_default_settings(&osqp_settings);
		osqp_settings.polish = true;
		osqp_settings.verbose = false;

		c_int exit_code = osqp_setup(&work, &osqp_data, &osqp_settings);
		if (exit_code != 0)
		{
			free(A_data_);
			free(A_indices_);
			free(A_indptr_);
			return false;
		}

		osqp_solve(work);

		bool success = false;
		if (work != nullptr && work->info != nullptr)
		{
			if (work->info->status_val == OSQP_SOLVED || work->info->status_val == OSQP_SOLVED_INACCURATE)
			{
				success = true;
				extractSolution(work->solution->x, sol);
				sol.optimal_cost = work->info->obj_val;
			}
		}

		if (work != nullptr)
		{
			osqp_cleanup(work);
		}

		free(A_data_);
		free(A_indices_);
		free(A_indptr_);

		return success;
	}

	void buildHessianMatrix(const Eigen::Matrix<double, kNx, kNx> &Q_diag,
							const Eigen::Matrix<double, kNu, kNu> &R_diag,
							const double state_cost_exponential,
							const double input_cost_exponential)
	{
		P_nnz_ = total_vars_;

		P_data_ = (c_float *)malloc(P_nnz_ * sizeof(c_float));
		P_indices_ = (c_int *)malloc(P_nnz_ * sizeof(c_int));
		P_indptr_ = (c_int *)malloc((total_vars_ + 1) * sizeof(c_int));

		int var_idx = 0;
		for (int col = 0; col <= total_vars_; ++col)
		{
			P_indptr_[col] = col;
		}

		for (int k = 0; k < kNstep; ++k)
		{
			double state_decay = std::exp(-(double)k / (double)kNstep * state_cost_exponential);
			for (int i = 0; i < kNx; ++i)
			{
				P_indices_[var_idx] = var_idx;
				P_data_[var_idx] = Q_diag(i, i) * state_decay;
				var_idx++;
			}

			double input_decay = std::exp(-(double)k / (double)kNstep * input_cost_exponential);
			for (int i = 0; i < kNu; ++i)
			{
				P_indices_[var_idx] = var_idx;
				P_data_[var_idx] = R_diag(i, i) * input_decay;
				var_idx++;
			}
		}

		const Eigen::Matrix<double, kNx, kNx> P_final_diag = Q_diag * std::exp(-state_cost_exponential);
		for (int i = 0; i < kNx; ++i)
		{
			P_indices_[var_idx] = var_idx;
			P_data_[var_idx] = P_final_diag(i, i);
			var_idx++;
		}
	}

	void buildConstraintMatrix(const std::vector<Eigen::SparseMatrix<double>> &Fx,
							  const std::vector<Eigen::SparseMatrix<double>> &Fu)
	{
		A_nnz_ = kNx;
		for (int k = 0; k < kNstep; ++k)
		{
			A_nnz_ += Fx[k].nonZeros() + Fu[k].nonZeros() + kNx;
		}
		A_nnz_ += kNstep * kNu;

		A_data_ = (c_float *)malloc(A_nnz_ * sizeof(c_float));
		A_indices_ = (c_int *)malloc(A_nnz_ * sizeof(c_int));
		A_indptr_ = (c_int *)malloc((total_vars_ + 1) * sizeof(c_int));

		std::vector<int> col_nnz(total_vars_, 0);
		for (int i = 0; i < kNx; ++i)
		{
			col_nnz[i]++;
		}

		for (int k = 0; k < kNstep; ++k)
		{
			int xk_offset = k * (kNx + kNu);
			int uk_offset = xk_offset + kNx;
			int xkp1_offset = (k + 1) * (kNx + kNu);

			for (int j = 0; j < Fx[k].outerSize(); ++j)
				for (Eigen::SparseMatrix<double>::InnerIterator it(Fx[k], j); it; ++it)
					col_nnz[xk_offset + it.col()]++;

			for (int j = 0; j < Fu[k].outerSize(); ++j)
				for (Eigen::SparseMatrix<double>::InnerIterator it(Fu[k], j); it; ++it)
					col_nnz[uk_offset + it.col()]++;

			for (int i = 0; i < kNx; ++i)
			{
				col_nnz[xkp1_offset + i]++;
			}
		}

		for (int k = 0; k < kNstep; ++k)
		{
			int uk_offset = k * (kNx + kNu) + kNx;
			for (int i = 0; i < kNu; ++i)
			{
				col_nnz[uk_offset + i]++;
			}
		}

		A_indptr_[0] = 0;
		for (int col = 0; col < total_vars_; ++col)
		{
			A_indptr_[col + 1] = A_indptr_[col] + col_nnz[col];
		}

		std::vector<int> col_pos(total_vars_, 0);
		std::vector<c_float> temp_data(A_nnz_);
		std::vector<c_int> temp_indices(A_nnz_);

		int constraint_idx = 0;
		for (int i = 0; i < kNx; ++i)
		{
			int pos = A_indptr_[i] + col_pos[i];
			temp_data[pos] = 1.0;
			temp_indices[pos] = constraint_idx++;
			col_pos[i]++;
		}

		for (int k = 0; k < kNstep; ++k)
		{
			int xk_offset = k * (kNx + kNu);
			int uk_offset = xk_offset + kNx;
			int xkp1_offset = (k + 1) * (kNx + kNu);

			for (int j = 0; j < Fx[k].outerSize(); ++j)
			{
				for (Eigen::SparseMatrix<double>::InnerIterator it(Fx[k], j); it; ++it)
				{
					int col = xk_offset + it.col();
					int pos = A_indptr_[col] + col_pos[col];
					temp_data[pos] = -it.value();
					temp_indices[pos] = constraint_idx + it.row();
					col_pos[col]++;
				}
			}

			for (int j = 0; j < Fu[k].outerSize(); ++j)
			{
				for (Eigen::SparseMatrix<double>::InnerIterator it(Fu[k], j); it; ++it)
				{
					int col = uk_offset + it.col();
					int pos = A_indptr_[col] + col_pos[col];
					temp_data[pos] = -it.value();
					temp_indices[pos] = constraint_idx + it.row();
					col_pos[col]++;
				}
			}

			for (int i = 0; i < kNx; ++i)
			{
				int col = xkp1_offset + i;
				int pos = A_indptr_[col] + col_pos[col];
				temp_data[pos] = 1.0;
				temp_indices[pos] = constraint_idx + i;
				col_pos[col]++;
			}

			constraint_idx += kNx;
		}

		for (int k = 0; k < kNstep; ++k)
		{
			int uk_offset = k * (kNx + kNu) + kNx;
			for (int i = 0; i < kNu; ++i)
			{
				int col = uk_offset + i;
				int pos = A_indptr_[col] + col_pos[col];
				temp_data[pos] = 1.0;
				temp_indices[pos] = constraint_idx++;
				col_pos[col]++;
			}
		}

		memcpy(A_data_, temp_data.data(), A_nnz_ * sizeof(c_float));
		memcpy(A_indices_, temp_indices.data(), A_nnz_ * sizeof(c_int));
	}

	void buildConstraintVectors(const std::vector<Eigen::VectorXd> &u_min,
							   const std::vector<Eigen::VectorXd> &u_max)
	{
		q_.resize(total_vars_, 0.0);
		l_.resize(total_constraints_, 0.0);
		u_.resize(total_constraints_, 0.0);

		int offset = kNx;
		std::fill(l_.begin() + offset, l_.begin() + offset + kNstep * kNx, 0.0);
		std::fill(u_.begin() + offset, u_.begin() + offset + kNstep * kNx, 0.0);

		offset += kNstep * kNx;
		for (int k = 0; k < kNstep; ++k)
		{
			for (int i = 0; i < kNu; ++i)
			{
				l_[offset] = u_min[k][i];
				u_[offset] = u_max[k][i];
				offset++;
			}
		}
	}

private:
	int total_vars_;
	int total_constraints_;

	c_float *P_data_ = nullptr;
	c_int *P_indices_ = nullptr;
	c_int *P_indptr_ = nullptr;
	int P_nnz_ = 0;

	c_float *A_data_ = nullptr;
	c_int *A_indices_ = nullptr;
	c_int *A_indptr_ = nullptr;
	int A_nnz_ = 0;

	std::vector<c_float> q_;
	std::vector<c_float> l_;
	std::vector<c_float> u_;

	Eigen::VectorXd x_des_start_;
	Eigen::VectorXd u_des_start_;

	void extractSolution(const c_float *solution, Solution &sol)
	{
		sol.delta_u.resize(kNstep);
		sol.delta_x.resize(kNstep + 1);
		for (int k = 0; k < kNstep; ++k)
		{
			int x_offset = k * (kNx + kNu);
			int u_offset = x_offset + kNx;
			sol.delta_x[k] = Eigen::VectorXd(kNx);
			sol.delta_u[k] = Eigen::VectorXd(kNu);
			for (int i = 0; i < kNx; ++i)
				sol.delta_x[k][i] = solution[x_offset + i];
			for (int i = 0; i < kNu; ++i)
				sol.delta_u[k][i] = solution[u_offset + i];
		}

		int final_offset = kNstep * (kNx + kNu);
		sol.delta_x[kNstep] = Eigen::VectorXd(kNx);
		for (int i = 0; i < kNx; ++i)
		{
			sol.delta_x[kNstep][i] = solution[final_offset + i];
		}
	}
};

void Trajectory_Data_t::feed_from_traj_utils(const traj_utils::PolyTrajConstPtr &pMsg)
{
	const traj_utils::PolyTraj &traj = *pMsg;
	if (traj.order < 3)
	{
		exec_traj = -1;
		return;
	}

	oneTraj_Data_t traj_data;
	traj_data.traj_start_time = pMsg->start_time;
	double t_total = 0;
	for (int i = 0; i < int(traj.duration.size()); ++i)
	{
		int num_dim = 3;
		int num_order = traj.order;
		t_total += traj.duration[i];

		Eigen::MatrixXd piece_coef;
		piece_coef.resize(num_dim, num_order + 1);
		for (int j = 0; j <= num_order; j++)
		{
			piece_coef(0, j) = traj.coef_x[i * (num_order + 1) + j];
			piece_coef(1, j) = traj.coef_y[i * (num_order + 1) + j];
			piece_coef(2, j) = traj.coef_z[i * (num_order + 1) + j];
		}
		traj_data.traj.emplace_back(traj.duration[i], piece_coef);
	}

	traj_data.traj_end_time = traj_data.traj_start_time + ros::Duration(t_total);
	if (ros::Time::now() < traj_data.traj_start_time)
	{
		while ((!traj_queue.empty()) && traj_queue.back().traj_start_time > traj_data.traj_start_time)
		{
			traj_queue.pop_back();
		}
		traj_queue.push_back(traj_data);
	}
	else
	{
		while ((!traj_queue.empty()) && traj_queue.front().traj_start_time < traj_data.traj_start_time)
		{
			traj_queue.pop_front();
		}
		traj_queue.push_front(traj_data);
	}

	total_traj_end_time = traj_queue.back().traj_end_time;
	total_traj_start_time = traj_queue.front().traj_start_time;
	exec_traj = 1;
}

OMMPCControl::OMMPCControl(Parameter_t &param) : param_(param)
{
	last_u_.q.setIdentity();
	last_u_.bodyrates = Eigen::Vector3d::Zero();
	last_u_.thrust = param_.thr_map.hover_percentage;
	last_yaw_ = 0.0;
	last_yaw_dot_ = 0.0;
	resetThrustMapping();
	initializeMpc();
}

void OMMPCControl::feedTrajectory(const traj_utils::PolyTrajConstPtr &pMsg)
{
	trajectory_data_.feed_from_traj_utils(pMsg);
}

void OMMPCControl::initializeMpc()
{
	mpc_wrapper_.reset(new MpcWrapper());
	Fx_.resize(kNstep);
	Fu_.resize(kNstep);
	u_lb_.resize(kNstep);
	u_ub_.resize(kNstep);
	for (int i = 0; i < kNstep; ++i)
	{
		u_lb_[i].resize(kNu);
		u_ub_[i].resize(kNu);
	}

	Eigen::Matrix<double, kNx, kNx> Q = (Eigen::Matrix<double, kNx, 1>()
			<< param_.mpc.Q_pos_xy, param_.mpc.Q_pos_xy, param_.mpc.Q_pos_z,
			param_.mpc.Q_velocity, param_.mpc.Q_velocity, param_.mpc.Q_velocity,
			param_.mpc.Q_attitude_rp, param_.mpc.Q_attitude_rp, param_.mpc.Q_attitude_yaw)
			   .finished()
			   .asDiagonal();
	Eigen::Matrix<double, kNu, kNu> R = (Eigen::Matrix<double, kNu, 1>()
			<< param_.mpc.R_thrust, param_.mpc.R_pitchroll, param_.mpc.R_pitchroll, param_.mpc.R_yaw)
			   .finished()
			   .asDiagonal();

	mpc_wrapper_->buildHessianMatrix(Q, R, param_.mpc.state_cost_exponential, param_.mpc.input_cost_exponential);
}

void OMMPCControl::computeFlatInputwithHopfFibration(const Eigen::Vector3d &thr_acc,
											  const Eigen::Vector3d &jer,
											  const double &yaw,
											  const double &yawd,
											  const Eigen::Quaterniond &att_est,
											  Eigen::Quaterniond &att,
											  Eigen::Vector3d &omg) const
{
	constexpr double kThrAccNormEps = 1.0e-3;
	constexpr double kOnePlusCEps = 1.0e-4;

	auto setFallbackOutput = [&]() {
		if (att_est.coeffs().allFinite() && att_est.norm() > 1.0e-6)
		{
			att = att_est.normalized();
		}
		else
		{
			att.setIdentity();
		}

		double yawd_safe = std::isfinite(yawd) ? yawd : 0.0;
		omg = Eigen::Vector3d(0.0, 0.0, yawd_safe);
	};

	if (!thr_acc.allFinite() || !jer.allFinite() || !std::isfinite(yaw) || !std::isfinite(yawd))
	{
		setFallbackOutput();
		return;
	}

	double thr_norm = thr_acc.norm();
	if (thr_norm < kThrAccNormEps)
	{
		setFallbackOutput();
		return;
	}

	Eigen::Vector3d abc = thr_acc / thr_norm;
	double a = abc(0), b = abc(1), c = abc(2);
	double one_plus_c = 1.0 + c;
	if (one_plus_c < kOnePlusCEps)
	{
		setFallbackOutput();
		return;
	}

	Eigen::Matrix3d projector = Eigen::Matrix3d::Identity() - abc * abc.transpose();
	Eigen::Vector3d abc_dot = projector * jer / thr_norm;
	if (!abc_dot.allFinite())
	{
		setFallbackOutput();
		return;
	}

	double a_dot = abc_dot(0), b_dot = abc_dot(1), c_dot = abc_dot(2);
	double norm = sqrt(2.0 * one_plus_c);
	Eigen::Quaterniond q(one_plus_c / norm, -b / norm, a / norm, 0.0);
	Eigen::Quaterniond q_yaw(cos(yaw / 2.0), 0.0, 0.0, sin(yaw / 2.0));
	att = (q * q_yaw).normalized();

	double syaw = sin(yaw), cyaw = cos(yaw);
	double inv_one_plus_c = 1.0 / one_plus_c;
	omg(0) = syaw * a_dot - cyaw * b_dot - (a * syaw - b * cyaw) * c_dot * inv_one_plus_c;
	omg(1) = cyaw * a_dot + syaw * b_dot - (a * cyaw + b * syaw) * c_dot * inv_one_plus_c;
	omg(2) = (b * a_dot - a * b_dot) * inv_one_plus_c + yawd;

	if (!att.coeffs().allFinite() || !omg.allFinite())
	{
		setFallbackOutput();
		return;
	}
}

double OMMPCControl::angleDiff(double a, double b)
{
	double d1 = a - b;
	double d2 = 2 * M_PI - fabs(d1);
	if (d1 > 0)
		d2 *= -1.0;
	return (fabs(d1) < fabs(d2)) ? d1 : d2;
}

void OMMPCControl::calculateYaw(const Eigen::Vector3d &vel, const double dt, double &yaw, double &yawdot)
{
	const double YAW_DOT_MAX_PER_SEC = param_.mpc.max_bodyrate_z * 0.90;
	const double YAW_DOT_DOT_MAX_PER_SEC = param_.mpc.max_bodyrate_z * 4.0;

	double yaw_temp = vel.norm() > 0.1 ? atan2(vel(1), vel(0)) : last_yaw_;
	double d_yaw = angleDiff(yaw_temp, last_yaw_);

	const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
	const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
	double d_yaw_max;
	if (fabs(last_yaw_dot_ + dt * YDDM) <= fabs(YDM))
	{
		d_yaw_max = last_yaw_dot_ * dt + 0.5 * YDDM * dt * dt;
	}
	else
	{
		double t1 = (YDM - last_yaw_dot_) / YDDM;
		d_yaw_max = ((dt - t1) + dt) * (YDM - last_yaw_dot_) / 2.0;
	}

	if (fabs(d_yaw) > fabs(d_yaw_max))
	{
		d_yaw = d_yaw_max;
	}
	yawdot = d_yaw / dt;
	yaw = last_yaw_ + d_yaw;
	if (yaw > M_PI)
		yaw -= 2 * M_PI;
	if (yaw < -M_PI)
		yaw += 2 * M_PI;

	last_yaw_ = yaw;
	last_yaw_dot_ = yawdot;
}

void OMMPCControl::setStateMatricesAndBounds(const int i,
								 const Eigen::Quaterniond &q,
								 const Eigen::Vector3d &omg,
								 const double t_step,
								 const double thracc)
{
	Fx_[i] = Eigen::SparseMatrix<double>(kNx, kNx);
	Fu_[i] = Eigen::SparseMatrix<double>(kNx, kNu);

	std::vector<Eigen::Triplet<double>> tripletList;
	tripletList.reserve(27);

	for (int k = 0; k < 3; ++k)
		tripletList.push_back(Eigen::Triplet<double>(k, k, 1.0));
	for (int k = 0; k < 3; ++k)
		tripletList.push_back(Eigen::Triplet<double>(3 + k, 3 + k, 1.0));
	for (int k = 0; k < 3; ++k)
		tripletList.push_back(Eigen::Triplet<double>(k, 3 + k, t_step));

	Eigen::Matrix3d exp_mat = SO3::exp(-omg * t_step);
	for (int row = 0; row < 3; ++row)
		for (int col = 0; col < 3; ++col)
			tripletList.push_back(Eigen::Triplet<double>(6 + row, 6 + col, exp_mat(row, col)));

	Eigen::Matrix3d mat_3_6 = t_step * q.toRotationMatrix() * SO3::hat(Eigen::Vector3d(0, 0, -thracc));
	for (int row = 0; row < 3; ++row)
		for (int col = 0; col < 3; ++col)
			tripletList.push_back(Eigen::Triplet<double>(3 + row, 6 + col, mat_3_6(row, col)));

	Fx_[i].setFromTriplets(tripletList.begin(), tripletList.end());
	Fx_[i].makeCompressed();

	tripletList.clear();
	tripletList.reserve(12);

	Eigen::Vector3d vec_3_0 = t_step * q.toRotationMatrix() * Eigen::Vector3d(0, 0, 1);
	for (int k = 0; k < 3; ++k)
		tripletList.push_back(Eigen::Triplet<double>(3 + k, 0, vec_3_0(k)));

	Eigen::Matrix3d mat_6_1 = SO3::leftJacobian(omg * t_step).transpose() * t_step;
	for (int row = 0; row < 3; ++row)
		for (int col = 0; col < 3; ++col)
			tripletList.push_back(Eigen::Triplet<double>(6 + row, 1 + col, mat_6_1(row, col)));

	Fu_[i].setFromTriplets(tripletList.begin(), tripletList.end());
	Fu_[i].makeCompressed();

	u_ub_[i] << thracc - param_.mpc.min_thrust,
		param_.mpc.max_bodyrate_xy + omg(0),
		param_.mpc.max_bodyrate_xy + omg(1),
		param_.mpc.max_bodyrate_z + omg(2);
	u_lb_[i] << -(param_.mpc.max_thrust - thracc),
		omg(0) - param_.mpc.max_bodyrate_xy,
		omg(1) - param_.mpc.max_bodyrate_xy,
		omg(2) - param_.mpc.max_bodyrate_z;
}

void OMMPCControl::setHoverReference(const Eigen::Vector4d &quad_pose)
{
	double yaw = quad_pose(3);
	double thracc = param_.gra;
	Eigen::Vector3d des_acc_in_world = Eigen::Vector3d(0, 0, param_.gra);
	Eigen::Quaterniond identity_q(1, 0, 0, 0), q;
	Eigen::Vector3d omg;
	computeFlatInputwithHopfFibration(des_acc_in_world, Eigen::Vector3d::Zero(), yaw, 0, identity_q, q, omg);

	for (int i = 0; i < kNstep; ++i)
	{
		setStateMatricesAndBounds(i, q, omg, param_.mpc.step_T, thracc);
	}
	mpc_wrapper_->buildConstraintMatrix(Fx_, Fu_);
	mpc_wrapper_->buildConstraintVectors(u_lb_, u_ub_);

	Eigen::VectorXd x_des_start(kNstate);
	Eigen::VectorXd u_des_start(kNu);
	x_des_start << quad_pose(0), quad_pose(1), quad_pose(2),
		q.w(), q.x(), q.y(), q.z(),
		0.0, 0.0, 0.0;
	u_des_start << param_.gra, omg(0), omg(1), omg(2);
	mpc_wrapper_->setDesiredStart(x_des_start, u_des_start);
}

void OMMPCControl::setTextReference(const std::vector<Eigen::Vector3d> &quad_positions,
							const std::vector<Eigen::Vector3d> &quad_velocities,
							const Odom_Data_t &odom,
							const double start_yaw,
							const std::vector<double> &yaws)
{
	if (quad_positions.size() != kNstep + 1 || quad_velocities.size() != kNstep + 1 || yaws.size() != kNstep + 1)
	{
		ROS_ERROR("[px4ctrl] setTextReference: bad input size");
		return;
	}

	const double t_step = param_.mpc.step_T;
	if (t_step <= 1.0e-6)
	{
		ROS_ERROR("[px4ctrl] setTextReference: invalid mpc.step_T");
		return;
	}

	Eigen::Quaterniond last_q = odom.q;
	Eigen::Vector3d body_z = last_q.toRotationMatrix() * Eigen::Vector3d(0, 0, 1);
	const int vel_size = static_cast<int>(quad_velocities.size());

	for (int i = 0; i < kNstep; ++i)
	{
		Eigen::Vector3d vel_i = quad_velocities[i];
		Eigen::Vector3d acc_i = Eigen::Vector3d::Zero();
		Eigen::Vector3d jer_i = Eigen::Vector3d::Zero();
		if (i == 0)
		{
			if (vel_size >= 2)
			{
				acc_i = (quad_velocities[1] - quad_velocities[0]) / t_step;
				if (vel_size >= 3)
				{
					Eigen::Vector3d acc_next = (quad_velocities[2] - quad_velocities[1]) / t_step;
					jer_i = (acc_next - acc_i) / t_step;
				}
			}
		}
		else
		{
			acc_i = (quad_velocities[i] - quad_velocities[i - 1]) / t_step;
			if (i + 1 < vel_size)
			{
				Eigen::Vector3d acc_next = (quad_velocities[i + 1] - quad_velocities[i]) / t_step;
				jer_i = (acc_next - acc_i) / t_step;
			}
		}

		double yaw_i = 0.0;
		double yaw_dot_i = 0.0;
		if (!param_.mpc.use_fix_yaw)
		{
			if (i == 0)
			{
				last_yaw_ = start_yaw;
			}
			calculateYaw(vel_i, t_step, yaw_i, yaw_dot_i);
			if (i == 0)
			{
				last_yaw_dot_ = yaw_dot_i;
			}
		}

		Eigen::Vector3d des_acc_in_world = acc_i + Eigen::Vector3d(0, 0, param_.gra);
		double thracc = des_acc_in_world.dot(body_z);

		Eigen::Quaterniond q;
		Eigen::Vector3d omg;
		computeFlatInputwithHopfFibration(des_acc_in_world, jer_i, yaw_i, yaw_dot_i, last_q, q, omg);
		q.normalize();
		last_q = q;

		if (i == 0)
		{
			Eigen::VectorXd x_des_start(kNstate);
			Eigen::VectorXd u_des_start(kNu);
			x_des_start << quad_positions[i](0), quad_positions[i](1), quad_positions[i](2),
				q.w(), q.x(), q.y(), q.z(),
				quad_velocities[i](0), quad_velocities[i](1), quad_velocities[i](2);
			u_des_start << thracc, omg(0), omg(1), omg(2);
			mpc_wrapper_->setDesiredStart(x_des_start, u_des_start);
		}

		body_z = q.toRotationMatrix() * Eigen::Vector3d(0, 0, 1);
		setStateMatricesAndBounds(i, q, omg, t_step, thracc);
	}

	mpc_wrapper_->buildConstraintMatrix(Fx_, Fu_);
	mpc_wrapper_->buildConstraintVectors(u_lb_, u_ub_);
}

void OMMPCControl::setTrajectoryReference(const Trajectory &traj,
						  const double tstart,
						  const double start_yaw,
						  const Trajectory &yaw_traj,
						  const Odom_Data_t &odom)
{
	(void)yaw_traj;
	const double t_step = param_.mpc.step_T;
	double t_all = traj.getTotalDuration() - 1.0e-3;
	double t = tstart;
	double yaw, yaw_dot;
	Eigen::Vector3d pos_quad, vel_quad, acc_quad, jerk_quad;
	Eigen::Quaterniond quat, last_quat = odom.q;
	Eigen::Vector3d body_z = last_quat.toRotationMatrix() * Eigen::Vector3d(0, 0, 1);
	Eigen::Vector3d omg;

	for (int i = 0; i < kNstep; i++)
	{
		Eigen::MatrixXd pvajs;
		if (t > t_all)
		{
			t = t_all;
			pvajs = traj.getPVAJSC(t);
			pos_quad = pvajs.col(0);
			vel_quad = Eigen::Vector3d::Zero();
			acc_quad = Eigen::Vector3d::Zero();
			jerk_quad = Eigen::Vector3d::Zero();
		}
		else
		{
			pvajs = traj.getPVAJSC(t);
			pos_quad = pvajs.col(0);
			vel_quad = pvajs.col(1);
			acc_quad = pvajs.col(2);
			jerk_quad = pvajs.col(3);
		}

		if (param_.mpc.use_fix_yaw)
		{
			yaw = 0.0;
			yaw_dot = 0.0;
		}
		else
		{
			if (i == 0)
			{
				last_yaw_ = start_yaw;
			}
			calculateYaw(vel_quad, t_step, yaw, yaw_dot);
			if (i == 0)
			{
				last_yaw_dot_ = yaw_dot;
			}
		}

		Eigen::Vector3d des_acc_in_world = acc_quad + Eigen::Vector3d(0, 0, param_.gra);
		double thracc = des_acc_in_world.dot(body_z);
		computeFlatInputwithHopfFibration(des_acc_in_world, jerk_quad, yaw, yaw_dot, last_quat, quat, omg);
		last_quat = quat;

		if (i == 0)
		{
			Eigen::VectorXd x_des_start(kNstate);
			Eigen::VectorXd u_des_start(kNu);
			x_des_start << pos_quad(0), pos_quad(1), pos_quad(2),
				last_quat.w(), last_quat.x(), last_quat.y(), last_quat.z(),
				vel_quad(0), vel_quad(1), vel_quad(2);
			u_des_start << thracc, omg(0), omg(1), omg(2);
			mpc_wrapper_->setDesiredStart(x_des_start, u_des_start);
		}

		body_z = last_quat.toRotationMatrix() * Eigen::Vector3d(0, 0, 1);
		setStateMatricesAndBounds(i, last_quat, omg, t_step, thracc);
		t += t_step;
	}

	mpc_wrapper_->buildConstraintMatrix(Fx_, Fu_);
	mpc_wrapper_->buildConstraintVectors(u_lb_, u_ub_);
}

void OMMPCControl::setReferenceFromPx4cmd(const Desired_State_t &des, const Odom_Data_t &odom)
{
	std::vector<Eigen::Vector3d> quad_positions(kNstep + 1), quad_velocities(kNstep + 1);
	std::vector<double> yaws(kNstep + 1, des.yaw);
	for (int i = 0; i <= kNstep; ++i)
	{
		double t = i * param_.mpc.step_T;
		quad_positions[i] = des.p + des.v * t + 0.5 * des.a * t * t;
		quad_velocities[i] = des.v + des.a * t;
		yaws[i] = des.yaw + des.yaw_rate * t;
	}
	setTextReference(quad_positions, quad_velocities, odom, des.yaw, yaws);
}

bool OMMPCControl::execMPC(const Odom_Data_t &odom, Controller_Output_t &u)
{
	Eigen::VectorXd x_des_start(kNstate), u_des_start(kNu);
	mpc_wrapper_->getDesiredStart(x_des_start, u_des_start);

	Eigen::Quaterniond est_q = odom.q.normalized();
	Eigen::Quaterniond des_q(x_des_start(3), x_des_start(4), x_des_start(5), x_des_start(6));
	Eigen::Vector3d err_q = SO3::log(est_q.toRotationMatrix().transpose() * des_q.toRotationMatrix());
	Eigen::Vector3d err_p = x_des_start.head(3) - odom.p;
	Eigen::Vector3d err_v = x_des_start.tail(3) - odom.v;
	Eigen::VectorXd delta_x_init(kNx);
	delta_x_init << err_p(0), err_p(1), err_p(2), err_v(0), err_v(1), err_v(2), err_q(0), err_q(1), err_q(2);
	mpc_wrapper_->setInitValue(delta_x_init);

	Solution solution;
	bool mpc_solved = mpc_wrapper_->solve(solution);
	if (!mpc_solved)
	{
		return false;
	}

	u.bodyrates(0) = u_des_start(1) - solution.delta_u[0](1);
	u.bodyrates(1) = u_des_start(2) - solution.delta_u[0](2);
	u.bodyrates(2) = u_des_start(3) - solution.delta_u[0](3);
	double thrustacc = u_des_start(0) - solution.delta_u[0](0);
	u.thrust = thrustacc / thr2acc_;

	timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
	while (timed_thrust_.size() > 100)
	{
		timed_thrust_.pop();
	}
	return true;
}

quadrotor_msgs::Px4ctrlDebug OMMPCControl::calculateControl(const Desired_State_t &des,
											   const Odom_Data_t &odom,
											   const Imu_Data_t &imu,
											   Controller_Output_t &u)
{
	ros::Time now_time = ros::Time::now();
	if (param_.mpc.use_polytraj_direct &&
		now_time >= trajectory_data_.total_traj_start_time &&
		now_time <= trajectory_data_.total_traj_end_time &&
		trajectory_data_.exec_traj == 1 &&
		(!trajectory_data_.traj_queue.empty()))
	{
		oneTraj_Data_t *traj_info = &trajectory_data_.traj_queue.front();
		if (trajectory_data_.traj_queue.size() > 1)
		{
			oneTraj_Data_t *next_traj_info = &trajectory_data_.traj_queue.at(1);
			while (now_time > next_traj_info->traj_start_time && trajectory_data_.traj_queue.size() > 1)
			{
				trajectory_data_.traj_queue.pop_front();
				traj_info = &trajectory_data_.traj_queue.front();
				if (trajectory_data_.traj_queue.size() > 1)
					next_traj_info = &trajectory_data_.traj_queue.at(1);
				else
					break;
			}
		}

		if (now_time < traj_info->traj_start_time)
		{
			Eigen::Vector4d hov;
			hov << odom.p(0), odom.p(1), odom.p(2), des.yaw;
			setHoverReference(hov);
		}
		else
		{
			double traj_time = (now_time - traj_info->traj_start_time).toSec();
			setTrajectoryReference(traj_info->traj, traj_time, des.yaw, traj_info->yaw_traj, odom);
		}
	}
	else
	{
		if (des.v.norm() < 1e-3 && des.a.norm() < 1e-3 && des.j.norm() < 1e-3)
		{
			Eigen::Vector4d hov;
			hov << des.p(0), des.p(1), des.p(2), des.yaw;
			setHoverReference(hov);
		}
		else
		{
			setReferenceFromPx4cmd(des, odom);
		}
	}

	bool mpc_solved = execMPC(odom, u);

	if (!mpc_solved)
	{
		ROS_ERROR_THROTTLE(1.0, "[px4ctrl] OM-MPC solve failed, keep last command.");
		u = last_u_;
	}
	else
	{
		u.q = imu.q;
		last_u_ = u;
	}

	debug_msg_.des_p_x = des.p(0);
	debug_msg_.des_p_y = des.p(1);
	debug_msg_.des_p_z = des.p(2);
	debug_msg_.des_v_x = des.v(0);
	debug_msg_.des_v_y = des.v(1);
	debug_msg_.des_v_z = des.v(2);
	debug_msg_.des_a_x = des.a(0);
	debug_msg_.des_a_y = des.a(1);
	debug_msg_.des_a_z = des.a(2) + param_.gra;
	debug_msg_.des_q_x = u.q.x();
	debug_msg_.des_q_y = u.q.y();
	debug_msg_.des_q_z = u.q.z();
	debug_msg_.des_q_w = u.q.w();
	debug_msg_.des_thr = u.thrust;

	return debug_msg_;
}

bool OMMPCControl::estimateThrustModel(const Eigen::Vector3d &est_a,
							   const Parameter_t &param)
{
	ros::Time t_now = ros::Time::now();
	while (timed_thrust_.size() >= 1)
	{
		std::pair<ros::Time, double> t_t = timed_thrust_.front();
		double time_passed = (t_now - t_t.first).toSec();
		if (time_passed > 0.045)
		{
			timed_thrust_.pop();
			continue;
		}
		if (time_passed < 0.035)
		{
			return false;
		}

		double thr = t_t.second;
		timed_thrust_.pop();

		double gamma = 1 / (rho2_ + thr * P_ * thr);
		double K = gamma * P_ * thr;
		thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
		P_ = (1 - K * thr) * P_ / rho2_;

		const double hover_percentage = param_.gra / thr2acc_;
		if (hover_percentage > 0.8 || hover_percentage < 0.1)
		{
			thr2acc_ = hover_percentage > 0.8 ? param_.gra / 0.8 : thr2acc_;
			thr2acc_ = hover_percentage < 0.1 ? param_.gra / 0.1 : thr2acc_;
		}
		return true;
	}
	return false;
}

void OMMPCControl::resetThrustMapping(void)
{
	thr2acc_ = param_.gra / param_.mpc.hover_percentage;
	P_ = 1e6;
}
