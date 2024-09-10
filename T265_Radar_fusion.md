雷达扫描频率典型值：6Hz

fusion定位方法主要依靠雷达数据解算，但是数据帧时间差过长，为了控制闭环速率提高，在两个数据帧之间使用t265的定位数据，t265采用差值数据进行定位，即使t265已经是漂移状态，传给飞控的位置数据对定位影响也不大（建立在副坐标系有效的前提下）

拟合直线的方法是否只能在矩形的场地内有效，空中出现障碍物，无法扫到初始的右侧线和下侧线是否就会定位错误

假如t265定位发生漂移，副坐标系建立失败时的后果（由于会直接调用slam结果的位置数据）

坐标转换过程

激光定高

```python

    
def establish_secondary_origin(
    self,
    force_level: bool = True,
    x_offset: float = 0.0,
    y_offset: float = 0.0,
    z_offset: float = 0.0,
    yaw_offset: float = 0,
):
    """
    以当前位置和姿态建立副坐标系原点
    force_level: 强制副坐标系为水平面
    offset: 当前位置相对于副坐标系原点的偏移
    (yaw_offset: 仅当返回eular时有效)
    """
    # 获取当前位置和朝向
    position = np.array([self.pose.translation.x, self.pose.translation.y, self.pose.translation.z])
    orientation = np.array([self.pose.rotation.x, self.pose.rotation.y, self.pose.rotation.z, self.pose.rotation.w])
    if force_level:
        orientation[0] = 0
        orientation[2] = 0

    # 将当前位置和朝向作为副坐标系的原点和朝向
    # rotation_matrix = quaternions_to_rotation_matrix(*orientation)
    self._secondary_position = position
    self._secondary_orientation = orientation
    self._secondary_rotation = Rotation.from_quat(orientation)  # xyzw
    self._secondary_rotation_matrix = self._secondary_rotation.as_matrix()
    self._offset_position = np.array([x_offset, y_offset, z_offset])
    self._offset_yaw = yaw_offset
    # logger.debug(f"[T265] Secondary origin established: {self._secondary_position}, {self._secondary_orientation}")
    self.secondary_frame_established = True

def get_pose_in_secondary_frame(self, as_eular=True) -> Tuple[np.ndarray, np.ndarray]:
    """
    获取当前位置和姿态在副坐标系中的表示
    as_eular: 是否返回欧拉角
    return: xyz位置, xyzw四元数/rpy欧拉角
    """
    if not self.secondary_frame_established:
        raise RuntimeError("Secondary frame not established")
    # 获取当前位置和朝向
    position = np.array([self.pose.translation.x, self.pose.translation.y, self.pose.translation.z])
    orientation = np.array([self.pose.rotation.x, self.pose.rotation.y, self.pose.rotation.z, self.pose.rotation.w])

    # 将当前位置和朝向转换到副坐标系中
    position -= self._secondary_position	# 数据帧差值
    # 反向应用副坐标系的旋转矩阵
    position = np.dot(position, self._secondary_rotation_matrix.T)
    # 反向应用副坐标系的朝向
    # rotation_matrix = quaternions_to_rotation_matrix(*orientation)
    # rotation_matrix = np.dot(rotation_matrix, self._secondary_rotation_matrix.T)
    # orientation = rotation_matrix_to_quaternions(rotation_matrix)
    rotation = Rotation.from_quat(orientation) * self._secondary_rotation.inv()

    position -= self._offset_position
    if as_eular:
        euler = rotation.as_euler("zxy", degrees=True)
        if self._offset_yaw != 0:
            euler[2] = (euler[2] - self._offset_yaw + 180) % 360 - 180
        return position, euler
    return position, rotation.as_quat()    
    
def calibrate_realsense(self, wait=True):
    """
    根据雷达数据校准T265的副坐标系，dx,dy,dyaw：雷达相对于基地点的绝对位置
    """
    if wait and not self.radar.rt_pose_update_event.wait(1):
        logger.error("[NAVI] calibrate_realsense(): Radar pose update timeout")
        raise RuntimeError("Radar pose update timeout")
    x, y, yaw = self.radar.rt_pose
    dx = x - self.basepoint[0]  # -> t265 -z * 100
    dx = -dx / 100.0
    dy = y - self.basepoint[1]  # -> t265 -x * 100
    dy = -dy / 100.0
    dyaw = -yaw
    logger_dbg.info(f"[NAVI] Calibrate T265: radar={self.radar.rt_pose} dz={dx}, dx={dy}, dyaw={dyaw}")
    self.rs.establish_secondary_origin(force_level=True, z_offset=dx, x_offset=dy, yaw_offset=dyaw)


def _get_t265_pose(self, wait=True) -> Optional[Tuple[float, float, float, bool]]:
    if wait and not self.rs.update_event.wait(1):
        logger.warning("[NAVI] RealSense pose timeout")
        return None
    self.rs.update_event.clear()
    if not self.rs.secondary_frame_established:
        current_x = -self.rs.pose.translation.z * 100
        current_y = -self.rs.pose.translation.x * 100
        current_yaw = -self.rs.eular_rotation[2]
    else:
        position, eular = self.rs.get_pose_in_secondary_frame(as_eular=True)
        current_x = -position[2] * 100
        current_y = -position[0] * 100
        current_yaw = -eular[2]
    available = self.rs.pose.tracker_confidence >= 2
    logger_dbg.debug(f"[NAVI] RealSense pose: {current_x}, {current_y}, {current_yaw}, {available}")
    return current_x, current_y, current_yaw, available    	#current_& 表示当前坐标（相对于基地点）
    
def _get_fusion_pose(self) -> Optional[Tuple[float, float, float, bool]]:
    if self.radar.rt_pose_update_event.is_set():
        self.calibrate_realsense(wait=False)
        self.radar.rt_pose_update_event.clear()
    return self._get_t265_pose()


pose = self._get_fusion_pose()
self.current_x, self.current_y, self.current_yaw, available = pose

out_x = round(self.navi_x_pid(self.current_x))
if out_x is not None:
	self.fc.update_realtime_control(vel_x=out_x)

out_y = round(self.navi_y_pid(self.current_y))
if out_y is not None:
	self.fc.update_realtime_control(vel_y=out_y)
    
out_yaw = round(self.yaw_pid(self.current_yaw))
if out_yaw is not None:
	self.fc.update_realtime_control(yaw=out_yaw)
    
# update_realtime_control() 函数会将数据发给飞控
```
