# 比赛信息
uint8 competition_type
uint8 competition_stage
uint16 stage_time
uint8 dart_target
uint8 small_energy_state
uint8 big_energy_state

# 己方血量
uint16 friend_hero
uint16 friend_engineer
uint16 friend_infantry3
uint16 friend_infantry4
uint16 friend_sentry
uint16 friend_outpost
uint16 friend_base

uint16 enemy_outpost_hp
uint16 enemy_base_hp   

# 战场状态
bool status_supply_non_overlap
bool status_supply_overlap
bool status_supply_occupied
uint8 energy_activation
uint8 highland_status
uint8 trapezoid_status
uint16 dart_hit_time
# uint8 dart_target
uint8 center_control

# 本机状态
uint8 robot_id
uint8 robot_level
uint16 current_hp
uint16 max_hp
uint16 cooling_rate
uint16 heat_limit
uint16 chassis_heat_limit

# 位置信息
float32 position_x
float32 position_y
float32 orientation
uint16 bullet_17mm
uint16 bullet_42mm
uint16 remaining_gold
uint16 projectile_allowance_fortress

# 队友位置
float32 hero_x
float32 hero_y
float32 engineer_x
float32 engineer_y
float32 infantry3_x
float32 infantry3_y
float32 infantry4_x
float32 infantry4_y

# 哨兵状态
uint16 sentry_bullet_allowance
uint8 sentry_remote_bullet_count
uint8 sentry_remote_hp_count
bool sentry_confirm_revive
bool sentry_immediate_revive
uint16 sentry_revive_cost

uint8 energy_mechanism_available

# 附加状态
bool additional_status_bit0
uint16 additional_status_bits1_11  # Bits 1-11: 17mm剩余可兑换数
uint8 additional_status_bits12_15 # Bits 12-15: 保留位