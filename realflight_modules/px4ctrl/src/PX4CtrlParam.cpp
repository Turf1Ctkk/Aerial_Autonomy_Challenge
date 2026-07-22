#include "PX4CtrlParam.h"

#include <string>
#include <vector>

namespace
{
void read_axis_double_param(const ros::NodeHandle &nh,
							const std::string &name,
							std::array<double, 3> &val)
{
	std::vector<double> vec;
	if (nh.getParam(name, vec))
	{
		if (vec.size() != 3)
		{
			ROS_ERROR_STREAM("Read param: " << name << " failed. Expected scalar or 3-element list.");
			ROS_BREAK();
		}

		for (int i = 0; i < 3; ++i)
		{
			val[i] = vec[i];
		}
		return;
	}

	double scalar;
	if (nh.getParam(name, scalar))
	{
		val = {{scalar, scalar, scalar}};
		return;
	}

	ROS_ERROR_STREAM("Read param: " << name << " failed.");
	ROS_BREAK();
}

void read_optional_axis_bool_param(const ros::NodeHandle &nh,
								   const std::string &name,
								   std::array<bool, 3> &val,
								   const bool default_value)
{
	val = {{default_value, default_value, default_value}};

	std::vector<bool> vec;
	if (nh.getParam(name, vec))
	{
		if (vec.size() != 3)
		{
			ROS_ERROR_STREAM("Read param: " << name << " failed. Expected scalar or 3-element list.");
			ROS_BREAK();
		}

		for (int i = 0; i < 3; ++i)
		{
			val[i] = vec[i];
		}
		return;
	}

	bool scalar;
	if (nh.getParam(name, scalar))
	{
		val = {{scalar, scalar, scalar}};
	}
}
}

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
	read_essential_param(nh, "gain/Kvi0", gain.Kvi0);
	read_essential_param(nh, "gain/Kvi1", gain.Kvi1);
	read_essential_param(nh, "gain/Kvi2", gain.Kvi2);
	read_essential_param(nh, "gain/KAngR", gain.KAngR);
	read_essential_param(nh, "gain/KAngP", gain.KAngP);
	read_essential_param(nh, "gain/KAngY", gain.KAngY);

	read_essential_param(nh, "rotor_drag/x", rt_drag.x);
	read_essential_param(nh, "rotor_drag/y", rt_drag.y);
	read_essential_param(nh, "rotor_drag/z", rt_drag.z);
	read_essential_param(nh, "rotor_drag/k_thrust_horz", rt_drag.k_thrust_horz);

	read_essential_param(nh, "msg_timeout/odom", msg_timeout.odom);
	read_essential_param(nh, "msg_timeout/rc", msg_timeout.rc);
	read_essential_param(nh, "msg_timeout/cmd", msg_timeout.cmd);
	read_essential_param(nh, "msg_timeout/imu", msg_timeout.imu);
	read_essential_param(nh, "msg_timeout/bat", msg_timeout.bat);

	read_essential_param(nh, "pose_solver", pose_solver);
	read_essential_param(nh, "mass", mass);
	read_essential_param(nh, "gra", gra);
	read_essential_param(nh, "ctrl_freq_max", ctrl_freq_max);
	read_essential_param(nh, "max_manual_vel", max_manual_vel);
	read_essential_param(nh, "max_angle", max_angle);
	read_essential_param(nh, "low_voltage", low_voltage);

	read_essential_param(nh, "rc_reverse/roll", rc_reverse.roll);
	read_essential_param(nh, "rc_reverse/pitch", rc_reverse.pitch);
	read_essential_param(nh, "rc_reverse/yaw", rc_reverse.yaw);
	read_essential_param(nh, "rc_reverse/throttle", rc_reverse.throttle);

	read_essential_param(nh, "auto_takeoff_land/enable", takeoff_land.enable);
    read_essential_param(nh, "auto_takeoff_land/enable_auto_arm", takeoff_land.enable_auto_arm);
	read_essential_param(nh, "auto_takeoff_land/no_RC", takeoff_land.no_RC);
	read_essential_param(nh, "auto_takeoff_land/takeoff_height", takeoff_land.height);
	read_essential_param(nh, "auto_takeoff_land/takeoff_land_speed", takeoff_land.speed);
	read_essential_param(nh, "auto_takeoff_land/land_point_x", takeoff_land.land_point_x);
	read_essential_param(nh, "auto_takeoff_land/land_point_y", takeoff_land.land_point_y);

	read_essential_param(nh, "thrust_model/print_value", thr_map.print_val);
	read_essential_param(nh, "thrust_model/K1", thr_map.K1);
	read_essential_param(nh, "thrust_model/K2", thr_map.K2);
	read_essential_param(nh, "thrust_model/K3", thr_map.K3);
	read_essential_param(nh, "thrust_model/accurate_thrust_model", thr_map.accurate_thrust_model);
	read_essential_param(nh, "thrust_model/hover_percentage", thr_map.hover_percentage);
	read_essential_param(nh, "thrust_model/noisy_imu", thr_map.noisy_imu);

	read_essential_param(nh, "position_adrc/enable", pos_adrc.enable);
	read_optional_axis_bool_param(nh, "position_adrc/axis_enable", pos_adrc.axis_enable, true);
	read_axis_double_param(nh, "position_adrc/beta1", pos_adrc.beta1);
	read_axis_double_param(nh, "position_adrc/beta2", pos_adrc.beta2);
	read_axis_double_param(nh, "position_adrc/beta3", pos_adrc.beta3);
	read_axis_double_param(nh, "position_adrc/b0", pos_adrc.b0);
	read_axis_double_param(nh, "position_adrc/fal_delta", pos_adrc.fal_delta);
	read_axis_double_param(nh, "position_adrc/comp_gain", pos_adrc.comp_gain);
	read_axis_double_param(nh, "position_adrc/deadband", pos_adrc.deadband);
	read_essential_param(nh, "position_adrc/horizontal_deadband", pos_adrc.horizontal_deadband);
	read_axis_double_param(nh, "position_adrc/leak_rate", pos_adrc.leak_rate);
	read_axis_double_param(nh, "position_adrc/max_dist_acc", pos_adrc.max_dist_acc);


	max_angle /= (180.0 / M_PI);

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
};

// void Parameter_t::config_full_thrust(double hov)
// {
// 	full_thrust = mass * gra / hov;
// };
