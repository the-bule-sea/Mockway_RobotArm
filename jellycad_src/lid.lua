--[[
Copyright (c) 2025. Li Jianbin. All rights reserved.
MIT License
JellyCAD version v0.3.10
Mockway Structure Lid Model File
--]]
local config = require('config')

local function model_lid0()
    local lid0 = cylinder.new(config.r_shell_outer, 2)
    return lid0:copy()
end

local function model_lid()
    local lid = model_lid0()
    for deg = 45, 360, 90 do
        local rad = deg * math.pi / 180;
        local len = (config.r_shell_outer + config.r_shell_inner) / 2
        local x0 = len * math.sin(rad);
        local y0 = len * math.cos(rad);
        lid:cut(cylinder.new(1.1, 4):pos(x0, y0, 0))
    end
    lid:fuse(text.new('mockway', 10):x(-20):y(-3):prism(0, 0, (2 + 0.4)))
    return lid:copy()
end
if config.generate_step_file then
    -- 生成STEP文件用于3D打印
    model_lid():color('turquoise'):export_step('lid.step')
end
if not debug.getinfo(3, "S") then
    -- 此文件为主模块时，显示完整模型
    model_lid():color('turquoise'):show()
end
return { m = model_lid():scale(1e-3) }
