local config = require('config')

local fix_flank = cylinder.new(config.r_flank_outer, config.h_flank)
fix_flank:fuse(cone.new(config.r_flank_outer, config.r_arm_out, config.r_flank_outer / 2 - config.h_flank):z(config
    .h_flank))
for deg = 60, 360, 60 do
    local rad = deg * math.pi / 180;
    local x0 = config.r_flank_screw_pos * math.sin(rad);
    local y0 = config.r_flank_screw_pos * math.cos(rad);
    fix_flank:cut(cylinder.new(3.5, config.r_flank_outer / 2 - config.h_flank):pos(x0, y0, config.h_flank))
    fix_flank:cut(cylinder.new(config.r_m3_hole, config.h_flank):pos(x0, y0, 0));
end

local height = config.h_fore_arm
local link1 = cylinder.new(config.r_arm_out, height)
link1:fuse(fix_flank:copy())
link1:fuse(fix_flank:copy():rx(180):z(height))
link1:cut(cylinder.new(config.r_arm_in, height))

local height = config.h_upper_arm
local link2 = cylinder.new(config.r_arm_out, height)
link2:fuse(fix_flank:copy())
link2:fuse(fix_flank:copy():rx(180):z(height))
link2:cut(cylinder.new(config.r_arm_in, height))

-- link1:color('green'):show()
-- link2:show()
-- link1:export_step('link1.step')
-- link2:export_step('link2.step')
return { m1 = link1:copy():scale(1e-3), m2 = link2:copy():scale(1e-3)}
