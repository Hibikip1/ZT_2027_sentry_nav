# 当前所在区域信息 (由 zone_manager 发布)
# 决策层行为树据此决定任务行为 (如狗洞区: 不小陀螺 + 正对洞口)

string zone_name          # 当前区域名 (空串 = 不在任何区域)
bool in_zone              # 是否在区域内
float32 nav_yaw           # 区域建议导航朝向 (rad, 用于正对洞口等)
bool spin_enabled         # 是否允许小陀螺
float32 speed_limit       # 区域限速 (m/s), 0 = 不限速
float32 zone_center_x     # 区域中心 x (map 系)
float32 zone_center_y     # 区域中心 y (map 系)
