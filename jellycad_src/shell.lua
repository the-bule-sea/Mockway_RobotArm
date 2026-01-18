--[[
Copyright (c) 2025. Li Jianbin. All rights reserved.
MIT License
JellyCAD version v0.3.10
Mockway Structure Motor Shell Model File
--]]
local config = require('config')

function model_shell0()
    local shell0 = cylinder.new(config.r_shell_outer, config.h_shell)
    local connecter = cylinder.new(config.r_shell_outer, config.r_shell_outer + config.h_flank)
        :z(config.r_shell_outer):ry(90)
    connecter:cut(cylinder.new(config.r_shell_outer, config.h_shell))
    shell0:fuse(connecter)
    -- 切割法兰孔
    shell0:cut(cylinder.new(config.r_shell_flank_hole, config.thickness))
    -- 切割法兰凹台
    shell0:cut(cylinder.new(config.r_flank_groove, config.h_flank):ry(90):x(config.r_shell_outer):z(config.r_shell_outer))
    -- 切割内圈
    shell0:cut(cylinder.new(config.r_shell_inner, config.h_shell):z(config.thickness))
    -- 圆形通线孔
    shell0:cut(cylinder.new(config.r_wire_hole, config.r_shell_outer):z(config.r_shell_outer):ry(90))
    return shell0:copy()
end

function model_shell()
    local shell = cylinder.new(config.r_shell_outer, config.h_shell)
    -- CAN插口槽
    shell:cut(box.new({ -config.w_can_plug / 2, -config.r_outer, config.z_plug },
        { config.w_can_plug / 2, config.r_outer, config.z_plug_top }))
    -- TTL插口槽
    shell:cut(box.new({ -config.r_outer, -config.w_ttl_plug / 2, config.z_plug },
        { 0, config.w_ttl_plug / 2, config.z_plug_top }))
    -- 标定孔
    shell:cut(cylinder.new(config.r_calib_slot, config.h_calib_slot):rx(90):y(config.r_outer))
    shell:cut(cylinder.new(config.r_calib_slot, config.h_calib_slot):rx(-90):y(-config.r_outer))
    -- 盖板固定孔
    for deg = 45, 360, 90 do
        local rad = deg * math.pi / 180;
        local len = (config.r_shell_outer + config.r_shell_inner) / 2
        local x0 = len * math.sin(rad);
        local y0 = len * math.cos(rad);
        shell:cut(cylinder.new(config.r_m2_tapping, 4):rx(180):pos(x0, y0, config.h_shell))
    end
    -- 合并连接台
    local connecter = cylinder.new(config.r_shell_outer, config.r_shell_outer + config.h_flank)
        :z(config.r_shell_outer):ry(90)
    connecter:cut(cylinder.new(config.r_shell_outer, config.h_shell))
    -- 标定孔
    connecter:cut(cylinder.new(config.r_calib_slot, config.h_calib_slot):rx(90)
        :pos(config.r_outer + config.h_flank, config.r_outer, config.r_outer))
    connecter:cut(cylinder.new(config.r_calib_slot, config.h_calib_slot):rx(-90)
        :pos(config.r_outer + config.h_flank, -config.r_outer, config.r_outer))
    shell:fuse(connecter)
    -- 切割法兰凹台
    shell:cut(cylinder.new(config.r_flank_groove, config.h_flank):ry(90):x(config.r_shell_outer):z(config.r_shell_outer))
    -- 法兰侧部M2螺丝孔
    shell:cut(cylinder.new(config.r_m2_hole, 2 * config.r_shell_outer):x(config.r_shell_outer + config.h_flank / 2));
    -- 切割内圈
    shell:cut(cylinder.new(config.r_shell_inner, config.h_shell):z(config.thickness))
    -- 切割法兰孔
    shell:cut(cylinder.new(config.r_shell_flank_hole, config.thickness))
    -- 圆形通线孔
    shell:cut(cylinder.new(config.r_wire_hole, config.r_shell_outer):z(config.r_shell_outer):ry(90))
    -- 方形通线槽
    shell:cut(box.new(config.w_wire_solt, config.l_wire_solt, config.r_outer):x(config.r_shell_inner):z(config.r_outer))

    for deg = 60, 360, 60 do
        local rad = deg * math.pi / 180;
        -- 电机外壳螺丝通孔
        local x0 = config.r_screw_distribut * math.sin(rad);
        local y0 = config.r_screw_distribut * math.cos(rad);
        shell:cut(cylinder.new(config.r_m3_hole, config.thickness):pos(x0, y0, 0));
        shell:cut(cone.new(config.r_m3_head, 0, config.r_m3_head):pos(x0, y0, 0));
        -- 法兰固定热熔螺母孔
        local y1 = config.r_flank_screw_pos * math.cos(rad);
        local z1 = config.r_flank_screw_pos * math.sin(rad) + config.r_outer;
        shell:cut(cylinder.new(config.r_m3d4_nut, config.r_shell_outer):ry(90):pos(0, y1, z1));
    end
    return shell:copy()
end

if config.generate_step_file then
    -- 生成STEP文件用于3D打印
    model_shell():color('turquoise'):export_step('shell.step')
end
if not debug.getinfo(3, "S") then
    -- 此文件为主模块时，显示完整模型
    model_shell():color('turquoise'):show()
end
return { m = model_shell0():scale(1e-3) }
