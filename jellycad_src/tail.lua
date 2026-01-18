--[[
Copyright (c) 2025. Li Jianbin. All rights reserved.
MIT License
JellyCAD version v0.3.10
Mockway Structure Tail Model File
--]]
local config = require('config')

function model_tail0()
    local tail0 = cylinder.new(config.r_flank_outer, config.h_tail)
    tail0:fuse(cylinder.new(config.r_flank_inner, config.h_tail_sum))
    for i = 0, 5 do
        local rad = i * math.pi / 3 -- 60° = π/3
        local x0 = config.r_flank_screw_pos * math.cos(rad);
        local y0 = config.r_flank_screw_pos * math.sin(rad);
        local screw_inner = cylinder.new(config.r_m3d4_nut, config.h_tail):pos(x0, y0, 0)
        tail0:cut(screw_inner)
    end
    return tail0:copy()
end

function model_tail()
    local tail = model_tail0()
    for i = 0, 5 do
        local rad = i * math.pi / 3 -- 60° = π/3
        -- 固定在电机法兰上的M3螺丝孔
        local x4 = config.r_screw_motor_flank * math.sin(rad);
        local y4 = config.r_screw_motor_flank * math.cos(rad);
        local screw_inner = cylinder.new(config.r_m3_hole, config.h_tail_sum):pos(x4, y4, 0)
        tail:cut(screw_inner)
        tail:cut(cone.new(config.r_m3_head, 0, config.r_m3_head):pos(x4, y4, 0));
    end
    return tail:copy()
end

if config.generate_step_file then
    -- 生成STEP文件用于3D打印
    model_tail():color('gray'):export_step('tail.step')
end
if not debug.getinfo(3, "S") then
    -- 此文件为主模块时，显示完整模型
    model_tail():color('gray'):show()
end

return { m = model_tail0():scale(1e-3) }
