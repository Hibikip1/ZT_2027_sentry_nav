uint8 game_type              # 4是高校联盟赛 1是超级对抗赛
uint8 game_progress          # 0未开始 1准备阶段 2倒计时15s 3裁判系统自检 4是比赛中 5结算中
uint16 stage_remain_time     # 当前阶段剩余时间

uint16 enemy_1_robot_hp      # 敌方英雄机器人血量
uint16 enemy_base_hp         # 敌方基地血量
uint16 enemy_7_robot_hp      # 敌方哨兵机器人血量
uint16 enemy_outpose_hp      # 敌方前哨站血量
uint16 enemy_3_robot_hp      # 敌方3号机器人血量
uint16 enemy_4_robot_hp      # 敌方4号机器人血量
uint16 enemy_5_robot_hp      # 敌方5号机器人血量


uint16 our_base_hp           # 我方基地血量
uint16 our_7_robot_hp        # 我方哨兵血量
uint16 our_outpose_hp        # 我方前哨站血量

uint16 bullet_17mm_allown    # 17mm弹丸允许发弹量
uint16 remain_money          # 剩余金币数量

uint16 success_convert_bullet_number    # 成功兑换的发弹量
uint8 success_remote_bullet_chance      # 成功远程兑换发弹量的次数
uint8 success_remote_hp_chance          # 成功远程兑换血量的次数

bool recharge_area_front_occupied   # 己方补给站前补血点的占领状态
bool recharge_area_inside_occupied  # 己方补给站内部补血点的占领状态
bool recharge_area_occupied         # 补给区占领情况(RMUL)

bool energy_machine_occupied            # 己方能量机关激活点的占领状态
bool small_energy_machine_activate      # 己方小能量机关的激活状态
bool big_energy_machine_activate        # 己方大能量机关的激活状态

uint8 circular_highland_occupied        # 己方环形高地的占领状态
uint8 t3_highland_cooupied              # 己方3号梯形高地的占领状态
uint8 t4_highland_cooupied              # 己方4号梯形高地的占领状态

uint8 visual_shield_remain_percent      # 己方基地虚拟护盾的剩余值百分比

uint8 gain_point_occupy       # 中心增益点占领情况(RMUL)

bool recharge_area_rfid                 # 补血点RFID检测情况
bool our_patrol_area_rfid               # 己方哨兵巡逻区RFID
bool enemy_patrol_area_rfid             # 对方哨兵巡逻区RFID
bool our_circular_highland_rfid         # 己方环形高地增益点
bool enemy_circular_highland_rfid       # 对方环形高地增益点
bool our_t3_highland_rfid               # 己方 R3/B3 梯形高地增益点
bool enemy_t3_highland_rfid             # 对方 R3/B3 梯形高地增益点
bool our_t4_highland_rfid               # 己方 R4/B4 梯形高地增益点
bool enemy_t4_highland_rfid             # 对方 R4/B4 梯形高地增益点

bool our_energy_machine_rfid            # 己方能量机关激活点
bool our_outpose_rfid                   # 己方前哨站增益点

bool gain_point_rfid         # 中心增益点RFID检测情况(RMUL)

uint8 recovery_buff             # 机器人回血增益
uint8 cooling_buff              # 机器人枪口冷却倍率
uint8 defence_buff              # 机器人防御增益
uint8 vulnerability_buff        # 机器人负防御增益
uint16 attack_buff              # 机器人攻击增益

bool power_management_shooter_output        # 电源管理模块 shooter 口输出

bool armor_underattack                      # 装甲板受击导致扣血

uint8 our_color                             # 我方颜色
float32 target_x                            # 目标位置x轴坐标，单位m
float32 target_y                            # 目标位置y轴坐标，单位m
uint8 pressed_key                           # 操作手按下的键值
uint8 enemy_number                          # 对方机器人ID
uint8 radar_number
float32 radar_x                                # 
float32 radar_y

bool is_spin                 # 订阅是否在小陀螺模式