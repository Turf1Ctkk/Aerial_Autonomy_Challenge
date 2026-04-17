#include "PX4CtrlParam.h"

Parameter_t::Parameter_t()
{
}

void Parameter_t::config_from_ros_handle(const ros::NodeHandle &nh)
{
	read_essential_param(nh, "gain/Kp0", gain.Kp0);
	read_essential_param(nh, "gain/Kp1", gain.Kp1);
	read_essential_param(nh, "gain/Kp2", gain.Kp2);
	read_essential_param(nh, "gain/Kv0", gain.Kv0);
	read_essential_param(nh, "gain/Kv1", gain.Kv1);
	read_essential_param(nh, "gain/Kv2", gain.Kv2);

	read_essential_param(nh, "rotor_drag/x", rt_drag.x);
	read_essential_param(nh, "rotor_drag/y", rt_drag.y);
	read_essential_param(nh, "rotor_drag/z", rt_drag.z);
	read_essential_param(nh, "rotor_drag/k_thrust_horz", rt_drag.k_thrust_horz);

	read_essential_param(nh, "msg_timeout/odom", msg_timeout.odom);
	read_essential_param(nh, "msg_timeout/rc", msg_timeout.rc);
	read_essential_param(nh, "msg_timeout/cmd", msg_timeout.cmd);
	read_essential_param(nh, "msg_timeout/imu", msg_timeout.imu);
	read_essential_param(nh, "msg_timeout/bat", msg_timeout.bat);

	read_param_or_default(nh, "controller_type", controller_type, 0);
	read_essential_param(nh, "mass", mass);
	read_essential_param(nh, "gra", gra);
	read_essential_param(nh, "ctrl_freq_max", ctrl_freq_max);
	read_essential_param(nh, "use_bodyrate_ctrl", use_bodyrate_ctrl);
	read_essential_param(nh, "max_manual_vel", max_manual_vel);

	read_essential_param(nh, "rc_reverse/roll", rc_reverse.roll);
	read_essential_param(nh, "rc_reverse/pitch", rc_reverse.pitch);
	read_essential_param(nh, "rc_reverse/yaw", rc_reverse.yaw);
	read_essential_param(nh, "rc_reverse/throttle", rc_reverse.throttle);

	read_essential_param(nh, "auto_takeoff_land/enable", takeoff_land.enable);
    read_essential_param(nh, "auto_takeoff_land/enable_auto_arm", takeoff_land.enable_auto_arm);
    read_essential_param(nh, "auto_takeoff_land/no_RC", takeoff_land.no_RC);
	read_essential_param(nh, "auto_takeoff_land/takeoff_height", takeoff_land.height);
	read_essential_param(nh, "auto_takeoff_land/takeoff_land_speed", takeoff_land.speed);

	read_essential_param(nh, "thrust_model/print_value", thr_map.print_val);
	read_essential_param(nh, "thrust_model/K1", thr_map.K1);
	read_essential_param(nh, "thrust_model/K2", thr_map.K2);
	read_essential_param(nh, "thrust_model/K3", thr_map.K3);
	read_essential_param(nh, "thrust_model/accurate_thrust_model", thr_map.accurate_thrust_model);
	read_essential_param(nh, "thrust_model/hover_percentage", thr_map.hover_percentage);

	read_param_or_default(nh, "mpc/step_T", mpc.step_T, 0.01);
	read_param_or_default(nh, "mpc/hover_percentage", mpc.hover_percentage, thr_map.hover_percentage);
	read_param_or_default(nh, "mpc/Q_pos_xy", mpc.Q_pos_xy, 1000.0);
	read_param_or_default(nh, "mpc/Q_pos_z", mpc.Q_pos_z, 800.0);
	read_param_or_default(nh, "mpc/Q_velocity", mpc.Q_velocity, 20.0);
	read_param_or_default(nh, "mpc/Q_attitude_rp", mpc.Q_attitude_rp, 40.0);
	read_param_or_default(nh, "mpc/Q_attitude_yaw", mpc.Q_attitude_yaw, 40.0);
	read_param_or_default(nh, "mpc/R_thrust", mpc.R_thrust, 0.5);
	read_param_or_default(nh, "mpc/R_pitchroll", mpc.R_pitchroll, 1.2);
	read_param_or_default(nh, "mpc/R_yaw", mpc.R_yaw, 0.6);
	read_param_or_default(nh, "mpc/state_cost_exponential", mpc.state_cost_exponential, 0.5);
	read_param_or_default(nh, "mpc/input_cost_exponential", mpc.input_cost_exponential, 0.5);
	read_param_or_default(nh, "mpc/max_bodyrate_xy", mpc.max_bodyrate_xy, 6.0);
	read_param_or_default(nh, "mpc/max_bodyrate_z", mpc.max_bodyrate_z, 4.0);
	read_param_or_default(nh, "mpc/min_thrust", mpc.min_thrust, 1.0);
	read_param_or_default(nh, "mpc/max_thrust", mpc.max_thrust, 30.0);
	read_param_or_default(nh, "mpc/use_fix_yaw", mpc.use_fix_yaw, true);
	read_param_or_default(nh, "mpc/use_polytraj_direct", mpc.use_polytraj_direct, false);
	read_param_or_default(nh, "mpc/shadow_compute", mpc.shadow_compute, false);
	read_param_or_default(nh, "mpc/polytraj_topic", mpc.polytraj_topic, std::string("/drone_0_planning/trajectory"));

	if ( takeoff_land.enable_auto_arm && !takeoff_land.enable )
	{
		takeoff_land.enable_auto_arm = false;
		ROS_ERROR("\"enable_auto_arm\" is only allowd with \"auto_takeoff_land\" enabled.");
	}
	if ( takeoff_land.no_RC && (!takeoff_land.enable_auto_arm || !takeoff_land.enable) )
	{
		takeoff_land.no_RC = false;
		ROS_ERROR("\"no_RC\" is only allowd with both \"auto_takeoff_land\" and \"enable_auto_arm\" enabled.");
	}

	if ( thr_map.print_val )
	{
		ROS_WARN("You should disable \"print_value\" if you are in regular usage.");
	}

	if (controller_type == 1 && !use_bodyrate_ctrl)
	{
		use_bodyrate_ctrl = true;
		ROS_WARN("[px4ctrl] controller_type=1 (OM-MPC) requires bodyrate control. Force set use_bodyrate_ctrl=true.");
	}
};

// void Parameter_t::config_full_thrust(double hov)
// {
// 	full_thrust = mass * gra / hov;
// };
