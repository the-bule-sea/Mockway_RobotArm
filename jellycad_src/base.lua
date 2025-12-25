local config = require('config')

-- 底座实体
base = cylinder.new(config.r_base_down, config.h_base)
-- 切割椭圆环
local elips = ellipse.new({ config.r_base_down, 0, config.h_base }, { 0, 1, 0 }, config.r1_ellip, config.r2_ellip)
base:cut(face.new(elips):revol({ 0, 0, 0 }, { 0, 0, 1 }, 360))
-- 圆角
local edge_info = { type = 'bspline_curve', first = { 45, 0, 8 }, last = { 45, 0, 8 }, tol = 1e-3 }
base:fillet(3, edge_info)
-- 切割法兰凹台
base:cut(cylinder.new(config.r_flank_groove, config.h_flank):z(config.h_base - config.h_flank))
-- 螺丝孔
for deg = 60, 360, 60 do
    local rad = deg * math.pi / 180;
    -- 法兰螺丝孔
    local x0 = config.r_flank_screw_pos * math.sin(rad);
    local y0 = config.r_flank_screw_pos * math.cos(rad);
    base:cut(cylinder.new(config.r_m3d4_nut, config.h_base):pos(x0, y0, 0));
    -- 底座螺丝孔
    local x1 = config.r_base_screw_pos * math.sin(rad);
    local y1 = config.r_base_screw_pos * math.cos(rad);
    base:cut(cylinder.new(config.r_m4_hole, config.h_base):pos(x1, y1, 0));
    base:cut(cylinder.new(config.r_m4_head, config.h_base):pos(x1, y1, 0):z(config.h_base_down));
end
-- 法兰侧部固定螺丝
base:cut(cylinder.new(config.r_m2_hole, 2 * config.r_base_down):x(-config.r_base_down):z(config.z_base_m2_hole):ry(90));
-- 水平线孔
base:cut(cylinder.new(config.r_base_horiz_hole, config.r_base_down):z(config.z_base_horiz_hole):ry(90));
-- 竖直线孔
base:cut(cylinder.new(config.r_base_vert_hole, config.h_base));
base:cut(cylinder.new(config.r_calib_slot, config.h_base):y(-config.r_outer + config.h_calib_slot):z(config.h_base):rx(90));
base:cut(cylinder.new(config.r_calib_slot, config.h_base):y(config.r_outer - config.h_calib_slot):z(config.h_base):rx(-90));
-- base:export_step('base.step')
show(base:color('turquoise'))
return { model = base:copy(), m = base:copy():scale(1e-3) }
