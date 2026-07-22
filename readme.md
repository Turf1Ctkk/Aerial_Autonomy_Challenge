# Aerial Autonomy Challenge Code
For competitions: 
[2026 China University Intelligent Robot Creative Competition - Embodied Intelligence Challenge (Special Competition) - Aerial Embodied Intelligence Group](https://www.robotcontest.cn/home/newsDetails?newsId=123038). Champion.

Also useful to [2026 ZJRobocon Aerial Challenge](http://www.zjrobocon.net/home/homepage). Second Prize.
## Brief Introduction
It is essentially built upon open-source repo [Diff-Planner](https://github.com/DifferentialRobotics/Diff-Planner), 
but relevant parameters in planner have been fine-tuned to suit the competition map: 
[run_exp_single_lio.launch](./diff_planner/plan_manage/launch/exp/run_exp_single_lio.launch), [advanced_param_exp.xml](./diff_planner/plan_manage/launch/include/advanced_param_exp.xml)

It also supports the detection and avoidance of dynamic obstacles with
**[ROI limitation](./diff_planner/plan_manage/launch/exp/run_exp_single_lio.launch#L45)**. Dynamic obstacle avoidance in **real competition flight:**
<p align="center">
  <img src="./misc/dyn_rviz1.gif" width="400">
  <img src="./misc/dyn_rviz2.gif" width="383">
</p>

**Other parameters have also been customized, please review and use them with caution.**

# Acknowledge
[Diff-Planner](https://github.com/DifferentialRobotics/Diff-Planner) for robust, full-stack autonomous drone framework.

[FAPP](https://github.com/arclab-hku/FAPP/) for tracking and planning of dynamic obstacle.

[LiDAR_IMU_Init](https://github.com/hku-mars/LiDAR_IMU_Init) for calibration of extrinsic parameters.
