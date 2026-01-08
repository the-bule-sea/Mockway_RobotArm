local base = require('base')
local shell = require('shell')
local flank = require('flank')
local linkage = require('linkage')
local config = require('config')

local motor4310 = cylinder.new(config.r_motor * 1e-3, 45 * 1e-3):color('black')
local motor4340 = cylinder.new(config.r_motor * 1e-3, 52.25 * 1e-3):color('black')
local m_base = base.m:copy()
local m_flank = flank.m:copy()
local m_shell = shell.m:copy()
local m1_linkage = linkage.m1:copy()

-- 设置各个部件的质量
motor4310:mass(295 * 1e-3)
motor4340:mass(360 * 1e-3)
m_base:mass(78 * 1e-3)
m_flank:mass((12 + 4 + 3) * 1e-3)
m_shell:mass((55 + 2) * 1e-3)
m1_linkage:mass((70.5 + 10) * 1e-3)
-- 基座
local base_link = {}
base_link[1] = m_base:copy()
base_link[2] = m_flank:copy():z((config.h_base - config.h_flank) * 1e-3)
for i = 1, #base_link do
    base_link[i]:color('#6495ED')
end
-- 肩部
local shoulder = {}
shoulder[1] = m_shell:copy():z((config.h_base + config.h_flank_reserve) * 1e-3)
    :rz(-90):color('#8470FF')
shoulder[2] = motor4310:copy():z((config.h_base + config.h_flank_reserve + config.thickness) * 1e-3)
shoulder[3] = m_flank:copy():rot(90, 0, 0):y((-config.r_outer) * 1e-3)
    :z((config.h_base + config.h_flank_reserve + config.r_outer) * 1e-3):color('#8470FF')
-- 上臂
local upperarm = {}
upperarm[1] = m_shell:copy():rot(180, -90, -90)
    :y(-(config.r_outer + config.h_flank + config.h_flank_reserve) * 1e-3)
    :z((config.h_base + config.h_flank_reserve + config.r_outer) * 1e-3)
upperarm[2] = motor4340:copy():rot(180, -90, -90)
    :y(-(config.r_outer + config.h_flank + config.h_flank_reserve + config.thickness) * 1e-3)
    :z((config.h_base + config.h_flank_reserve + config.r_outer) * 1e-3)

upperarm[3] = m_shell:copy():rot(0, 90, -90)
    :y(-(config.r_outer + config.h_flank + config.h_flank_reserve) * 1e-3)
    :z((config.h_base + 2 * config.r_outer + config.h_upper_arm + 2 * config.h_flank) * 1e-3)
upperarm[4] = motor4340:copy():rot(0, 90, -90)
    :y(-(config.r_outer + config.h_flank + config.h_flank_reserve + config.thickness) * 1e-3)
    :z((config.h_base + 2 * config.r_outer + config.h_upper_arm + 2 * config.h_flank) * 1e-3)

upperarm[5] = m1_linkage:copy()
    :y(-(2 * config.r_outer + config.h_flank + config.h_flank_reserve) * 1e-3)
    :z((config.h_base + config.h_flank_reserve + 2 * config.r_outer) * 1e-3)

local d1 = (config.h_base + config.h_flank_reserve + config.r_outer) * 1e-3

local joint_axes = {}
joint_axes[1] = axes.new({ 0, 0, d1, 0, 0, 0 }, 0.1)
joint_axes[2] = joint_axes[1]:copy():move({ 0, 0, 0, 90, 0, 0 })

for _, arr in ipairs({ joint_axes, base_link, shoulder, upperarm }) do
    for _, value in ipairs(arr) do
        value:show()
    end
end

j1_limit = { lower = -6.28, upper = 6.28, velocity = 3.14, effort = 9 }
j2_limit = { lower = -6.28, upper = 6.28, velocity = 3.14, effort = 9 }
j3_limit = { lower = -3.14, upper = 3.14, velocity = 3.14, effort = 9 }
j4_limit = { lower = -6.28, upper = 6.28, velocity = 3.14, effort = 3 }
j5_limit = { lower = -6.28, upper = 6.28, velocity = 3.14, effort = 3 }
j6_limit = { lower = -6.28, upper = 6.28, velocity = 3.14, effort = 3 }
urdf = link.new("base_link", base_link)
joint1 = urdf:add(joint.new("joint1", joint_axes[1], "revolute", j1_limit))
link1 = joint1:next(link.new("link1", shoulder))
joint2 = link1:add(joint.new("joint2", joint_axes[2], "revolute", j2_limit))
link2 = joint2:next(link.new("link2", upperarm))
urdf:export({ name = 'mockway_description', path = '../', ros_version = 2 })
