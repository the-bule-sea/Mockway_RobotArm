local config = require('config')

local function model_fix_flank()
    local fix_flank = cylinder.new(config.r_flank_outer, config.h_flank)
    fix_flank:fuse(cone.new(config.r_flank_outer, config.r_arm_out, config.r_flank_outer / 2 - config.h_flank)
        :z(config.h_flank))
    return fix_flank:copy()
end

local function model_arm_link0(height0)
    local fix_flank = model_fix_flank()
    local arm_link0 = cylinder.new(config.r_arm_out, height0)
    arm_link0:fuse(fix_flank:copy())
    arm_link0:fuse(fix_flank:copy():rx(180):z(height0))
    arm_link0:cut(cylinder.new(config.r_arm_in, height0))
    return arm_link0:copy()
end

local function model_arm_link(height)
    local arm_link = model_arm_link0(height)
    for deg = 60, 360, 60 do
        local rad = deg * math.pi / 180;
        local x0 = config.r_flank_screw_pos * math.sin(rad);
        local y0 = config.r_flank_screw_pos * math.cos(rad);
        arm_link:cut(cylinder.new(3.5, height - 2 * config.h_flank):pos(x0, y0, config.h_flank))
        arm_link:cut(cylinder.new(config.r_m3_hole, height):pos(x0, y0, 0));
    end
    return arm_link:copy()
end

local export_product = false
if export_product then
    model_arm_link(config.h_fore_arm):color('turquoise'):export_step('fore_arm.step'):show()
    model_arm_link(config.h_upper_arm):color('turquoise'):export_step('upper_arm.step'):show()
end

return {
    m1 = model_arm_link0(config.h_fore_arm):copy():scale(1e-3),
    m2 = model_arm_link0(config.h_upper_arm):copy():scale(1e-3)
}
