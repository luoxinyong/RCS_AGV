int16 id_task                    # 站点ID
int16 type_current_task          # 当前节点类型
float32 max_speed_current_task   # 当前节点速度
int16 type_next_task             # 下个节点类型
float32 max_speed_next_task      # 下个节点速度
float32 radius_sign_next_task    # 下个节点转弯半径，带正负号
geometry_msgs/Pose point_current # 实时位姿
float32 yaw_current_pose

float32 delta_center  # 车体中心到路线投影距离
float32 delta_head    # 前轮中心到路线投影距离
float32 delta_tail    # 后轮中心到路线投影距离
float32 delta_angle   # 航向方向与路径朝向的夹角

float32 angle_diff_spin_around # 自旋角度差
float32 radius_turn            # 转弯半径
float32 length_turn            # 转弯弧长
float32 dis_to_stop_node       # 停车距离
float32 dis_to_type_switch_node # 切换距离
float32 dis_remain_curr_node    # 当前节点剩余距离
int16 flag_need_avoid      # 当前任务是否需要绕障
int32 error_code           # 故障列表
int8 flag_enter_elevator   # 当前任务是否需要进电梯
int8 flag_door_stop        # 等待门开状态 0-行驶 1-停止等待

int8 flag_charge           # 充电控制，0-未知；1-充电；2-充电停止
int16 obs_area             # 避障区域设置
int8 flag_pick_task_valid  # 取货任务是否有效, 0 表示无效, 1 表示有效
int8 flag_detect_pallet    # 相机是否检测到托盘: 1 成功 2 失败
int32 x_detect           # 托盘在相机坐标系下的 x, 单位 mm
int32 y_detect           # 托盘在相机坐标系下的 y, 单位 mm
int32 theta_detect       # 托盘在相机坐标系下的 theta

geometry_msgs/Point32 point_lookahead
geometry_msgs/Point32 project_point_base
geometry_msgs/Point32 project_point_head
geometry_msgs/Point32 project_point_tail
geometry_msgs/Twist cmd_vel