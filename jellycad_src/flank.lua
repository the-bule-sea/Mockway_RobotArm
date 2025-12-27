--[[
Copyright (c) 2025. Li Jianbin. All rights reserved.
MIT License
JellyCAD version v0.3.9
Mockway Structure Flank Model File
--]]
local config = require('config')
flank = cylinder.new(config.r_flank_outer, config.h_flank)
flank:fuse(cylinder.new(config.r_flank_inner, config.h_flank_sum))
for i, deg in ipairs({ 0, 60, 120, 180, 240, 300 }) do
    local rad = deg * math.pi / 180;
    local rad2 = (deg + config.deg_slot) * math.pi / 180;
    local x0 = config.r_flank_screw_pos * math.cos(rad);
    local y0 = config.r_flank_screw_pos * math.sin(rad);
    local x1 = (config.r_flank_screw_pos - config.r_m3_hole) * math.cos(rad);
    local y1 = (config.r_flank_screw_pos - config.r_m3_hole) * math.sin(rad);
    local x2 = (config.r_flank_screw_pos + config.r_m3_hole) * math.cos(rad);
    local y2 = (config.r_flank_screw_pos + config.r_m3_hole) * math.sin(rad);
    -- 螺丝槽
    local slot = line.new({ x1, y1, 0 }, { x2, y2, 0 }):prism(0, 0, config.h_flank)
    slot:revol({ 0, 0, 0 }, { 0, 0, 1 }, config.deg_slot)
    local x3 = config.r_flank_screw_pos * math.cos(rad2);
    local y3 = config.r_flank_screw_pos * math.sin(rad2);
    -- M3孔
    local hole = cylinder.new(config.r_m3_hole, config.h_flank):pos(x0, y0, 0)
    -- M3头孔
    local screw_head = cylinder.new(config.r_flank_screw_head, config.h_flank):pos(x3, y3, 0)
    flank:cut(slot)
    flank:cut(hole)
    flank:cut(screw_head)
    -- 固定在电机法兰上的M3螺丝孔
    local x4 = config.r_screw_motor_flank * math.sin(rad);
    local y4 = config.r_screw_motor_flank * math.cos(rad);
    local screw_inner = cylinder.new(config.r_m3_hole, config.h_flank_sum):pos(x4, y4, 0)
    flank:cut(screw_inner)
    flank:cut(cone.new(config.r_m3_head, 0, config.r_m3_head):pos(x4, y4, 0));
end
-- 侧面两个热熔螺丝孔，防止法兰松动
flank:cut(cylinder.new(config.r_m2d3_nut, 6):rx(90):y(config.r_flank_outer):z(config.h_flank / 2))
flank:cut(cylinder.new(config.r_m2d3_nut, 6):rx(-90):y(-config.r_flank_outer):z(config.h_flank / 2))
show(flank:color('gray'))
-- flank:export_step('flank.step')
return { model = flank:copy(), m = flank:copy():scale(1e-3) }
