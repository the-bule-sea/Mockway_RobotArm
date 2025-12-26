local config = {}
-- 通用
config.r_motor = 57 / 2                                                  -- 电机直径57
config.r_motor_flank = 35 / 2                                            -- 电机法兰半径
config.r_screw_distribut = 50 / 2                                        -- 电机外壳螺丝分布半径
config.r_screw_motor_flank = 27 / 2                                      -- 电机法兰螺丝分布半径
config.h_motor_convex = 1                                                -- 电机凸台高度
config.thickness = 3                                                     -- 壳壁厚3
config.interspace = 0.2                                                  -- 电机壁与壳内壁的空隙0.2
config.r_outer = config.r_motor + config.interspace + config.thickness   -- 外壁半径
config.h_flank = 4                                                       -- 法兰高度(凹台高度)
config.t_flank_groove = 2                                                -- 法兰凹台壁厚
config.r_flank_groove = config.r_outer - config.t_flank_groove           -- 法兰凹台内壁半径
config.r_m2_hole = 1 + 0.1                                               -- M2螺丝通孔半径
config.r_m2d3_nut = 1.5 - 0.2                                            -- M2D3热熔螺母半径
config.r_m3d4_nut = 2 - 0.1                                              -- M3D4热熔螺母半径
config.r_m3_hole = 1.5 + 0.1                                             -- M3通孔半径
config.r_m3_head = 3                                                     -- M3平头空间半径
config.r_m2_tapping = 1.6 / 2                                            -- M2自攻螺丝孔
config.r_calib_slot = 1.7                                                -- 标定槽半径
config.h_calib_slot = 1                                                  -- 标定槽深度
config.r_flank = config.r_outer - config.t_flank_groove - 0.5            -- 法兰半径，预留0.5空间转动
config.r_flank_screw_head = 3.3                                          -- 法兰螺丝头通孔，用于通过M3外六角螺丝头
-- config.r_flank_screw_pos = config.r_flank - config.r_flank_screw_head - 2 -- 法兰螺丝位置半径
config.r_flank_screw_pos = config.r_screw_distribut                      -- 法兰螺丝位置半径
-- 基座
config.r_base_down = 45                                                  -- 基座下部半径
config.h_base_down = 8                                                   -- 基座下部高度
config.h_base_up = 16.6 + config.h_flank                                 -- 基座上部高度
config.h_base = config.h_base_down + config.h_base_up                    -- 基座总高度
config.r1_ellip = config.h_base_up                                       -- 切椭圆的半长轴
config.r2_ellip = config.r_base_down - config.r_outer                    -- 切椭圆的半短轴
config.z_base_m2_hole = config.h_base - config.h_flank / 2               -- 法兰侧边螺丝孔Z位置
config.r_m4_hole = 2.1                                                   -- 底座M4固定螺丝孔
config.r_m4_head = 4                                                     -- 底座M4固定螺丝头半径
config.r_base_vert_hole = 16                                             -- 竖直线孔半径
config.r_base_horiz_hole = 4                                             -- 水平线孔半径
config.z_base_horiz_hole = config.h_base_down + config.r_base_horiz_hole -- 水平线孔Z位置
config.r_base_screw_pos = 40                                             -- 底座螺丝位置半径
-- 外壳
config.r_shell_outer = config.r_outer                                    -- 外壳外壁半径
config.r_shell_inner = config.r_shell_outer - config.thickness           -- 外壳内壁半径
config.h_shell = 2 * config.r_outer                                      -- 外壳高度等于直径
config.r_shell_flank_hole = config.r_motor_flank + 1                     -- 外壳法兰孔半径，预留1的间隙
config.r_wire_hole = 12.5                                                -- 圆形通线孔半径
config.w_wire_solt = 2.4 * 2                                             -- 方形通线槽深度
config.l_wire_solt = 10                                                  -- 方形通线槽长度
config.w_can_plug = 16                                                   -- CAN插口宽度
config.w_ttl_plug = 10                                                   -- TTL插口宽度
config.h_plug = 19.2                                                     -- 插口高度
config.z_plug = config.thickness + 37                                    -- 插口底部Z位置
config.z_plug_top = config.z_plug + config.h_plug                        -- 插口顶部Z位置
-- 法兰
config.deg_slot = 18                                                     -- 螺丝槽角度
config.h_flank_reserve = 3.2;                                            -- 法兰预计抽空高度
-- 法兰颈部高度
config.h_flank_nick = (config.thickness - config.h_motor_convex) + config.h_flank_reserve;
config.h_flank_sum = config.h_flank_nick + config.h_flank;  -- 法兰总高度
config.r_flank_outer = config.r_flank_groove - 0.5;         -- 法兰外壁半径，预留1mm空间转动
config.r_flank_inner = config.r_motor_flank;                -- 法兰本体半径
-- 连杆
config.r_arm_out = config.r_flank_screw_pos - 3.5 - 1;      -- 臂外径，预留螺母位，再额外预留1mm空间
config.r_arm_in = config.r_arm_out - config.thickness;      -- 臂内径，控制壁厚
config.h_upper_arm = 140                                    -- 大臂(上臂)高度
config.h_fore_arm = 127                                     -- 小臂(前臂)高度
-- 末端
config.h_tail_sum = config.thickness + config.h_flank + 0.5 -- 末端高度
return config
