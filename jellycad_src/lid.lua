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
    return lid:copy()
end

local export_product = false
if export_product then
    model_lid():color('turquoise'):export_step('lid.step'):show()
end
return { m = model_lid():scale(1e-3) }
