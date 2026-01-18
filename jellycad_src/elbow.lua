--[[
Copyright (c) 2025. Li Jianbin. All rights reserved.
MIT License
JellyCAD version v0.3.10
Mockway Structure Elbow Model File
--]]
local config = require('config')

local function model_elbow0()
    local elbow0 = face.new(circle.new({ 0, 0, 0 }, { 0, 0, 1 }, config.r_shell_outer))
    elbow0:revol({ config.r_shell_outer, 0, 0 }, { 0, 1, 0 }, 90)
    local hollow = face.new(circle.new({ 0, 0, config.thickness }, { 0, 0, 1 }, config.r_shell_inner))
    hollow:revol({ config.r_shell_inner, 0, config.thickness }, { 0, 1, 0 }, 90)
    elbow0:cut(hollow)
    elbow0:cut(cylinder.new(config.r_shell_flank_hole, config.thickness))
    elbow0:cut(cylinder.new(config.r_shell_flank_hole, config.thickness):ry(-90):x(config.r_outer):z(config.r_outer))
    elbow0:cut(cylinder.new(config.r_shell_flank_hole, config.thickness))
    elbow0:cut(cylinder.new(config.r_shell_flank_hole, config.thickness):ry(-90):x(config.r_outer):z(config.r_outer))
    elbow0:z(config.h_flank)
    local wall = cylinder.new(config.r_outer, config.h_flank)
    wall:cut(cylinder.new(config.r_flank_groove, config.h_flank))
    elbow0:fuse(wall)
    return elbow0:copy()
end

local function model_elbow()
    local elbow = model_elbow0()
    for deg = 60, 360, 60 do
        local rad = deg * math.pi / 180;
        -- 电机外壳螺丝通孔
        local x0 = config.r_flank_screw_pos * math.sin(rad);
        local y0 = config.r_flank_screw_pos * math.cos(rad);
        elbow:cut(cylinder.new(config.r_m3d4_nut, config.thickness):pos(x0, y0, config.h_flank));
        -- 法兰固定热熔螺母孔
        local y1 = config.r_flank_screw_pos * math.cos(rad);
        local z1 = config.r_flank_screw_pos * math.sin(rad) + config.r_outer;
        elbow:cut(cylinder.new(config.r_m3d4_nut, config.thickness):ry(-90):pos(config.r_outer, y1, z1 + config.h_flank));
    end
    local fix = cylinder.new(config.r_m2_hole, 2 * config.r_shell_outer):ry(-90)
        :x(config.r_shell_outer):z(config.h_flank / 2)
    -- 标定孔
    elbow:cut(cylinder.new(config.r_calib_slot, config.h_calib_slot):rx(90):y(config.r_outer))
    elbow:cut(cylinder.new(config.r_calib_slot, config.h_calib_slot):rx(-90):y(-config.r_outer))
    elbow:cut(fix)
    elbow:cut(box.new({ -config.w_can_plug / 2, -config.r_outer, config.h_flank + config.thickness },
        { config.w_can_plug / 2, config.r_outer, config.h_flank + config.thickness + 6 }))
    return elbow:copy()
end

if config.generate_step_file then
    -- 生成STEP文件用于3D打印
    model_elbow():color('turquoise'):export_step('elbow.step')
end
if not debug.getinfo(3, "S") then
    -- 此文件为主模块时，显示完整模型
    model_elbow():color('turquoise'):show()
end

return { m = model_elbow0():scale(1e-3) }
