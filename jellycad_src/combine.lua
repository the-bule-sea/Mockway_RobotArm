local base = require('base')
local shell = require('shell')
-- local lid = require('lid')
local flank = require('flank')
-- local elbow = require('elbow')
-- local tail = require('tail')
local config = require('config')


local motor4310 = cylinder.new(config.r_motor * 1e-3, 45 * 1e-3):color('black')
local motor4340 = cylinder.new(config.r_motor * 1e-3, 52.25 * 1e-3):color('black')
--[[
local m_joint = shell.m:copy()
-- m_joint:fuse(lid.m:copy():z((config.h_shell + 1e-3) * 1e-3))
local m_joint_flank = m_joint:copy()
m_joint_flank:fuse(flank.m:copy():move('rz', 90):move('ry', 90):x(config.r_outer * 1e-3):z(config.r_outer * 1e-3):copy())

local base_link = base.m:copy()
base_link:fuse(flank.m:copy():z((config.h_base - config.h_flank) * 1e-3):rz(90))
local shoulder = m_joint_flank:copy():z((config.h_base + config.h_flank_reserve) * 1e-3)

local upperarm = m_joint:copy():rot(180, -90, 0)
upperarm:x((config.r_outer + config.h_flank + config.h_flank_reserve) * 1e-3)
upperarm:z((config.h_base + config.h_flank_reserve + config.r_outer) * 1e-3)

base_link:color('#6495ED'):show()
shoulder:color('#8470FF'):show()
upperarm:color('#FFC1C1'):show()
--]]


local base_link = {}
local shoulder = {}
base_link[1] = base.m:copy()
base_link[2] = flank.m:copy():z((config.h_base - config.h_flank) * 1e-3)
for i = 1, #base_link do
    base_link[i]:color('#6495ED')
end
shoulder[1] = shell.m:copy():z((config.h_base + config.h_flank_reserve) * 1e-3)
shoulder[2] = motor4310:copy():z((config.h_base + config.h_flank_reserve + config.thickness) * 1e-3)
shoulder[3] = flank.m:copy():rot(-90, 0, -90):x((config.r_outer) * 1e-3)
    :z((config.h_base + config.h_flank_reserve + config.r_outer) * 1e-3)
for i = 1, #shoulder do
    shoulder[i]:color('#8470FF')
end

local upperarm = {}
upperarm[1] = shell.m:copy():rot(180, -90, 0)
    :x((config.r_outer + config.h_flank + config.h_flank_reserve) * 1e-3)
    :z((config.h_base + config.h_flank_reserve + config.r_outer) * 1e-3):show()
upperarm[2] = motor4340:copy():rot(180, -90, 0)
    :x((config.r_outer + config.h_flank + config.h_flank_reserve + config.thickness) * 1e-3)
    :z((config.h_base + config.h_flank_reserve + config.r_outer) * 1e-3):show()

upperarm[3] = shell.m:copy():rot(0, 90, 0)
    :x((config.r_outer + config.h_flank + config.h_flank_reserve) * 1e-3)
    :z((config.h_base + config.h_flank_reserve + 2 * config.r_outer + config.h_upper_arm + config.h_flank) * 1e-3):show()
upperarm[4] = motor4340:copy():rot(0, 90, 0)
    :x((config.r_outer + config.h_flank + config.h_flank_reserve + config.thickness) * 1e-3)
    :z((config.h_base + config.h_flank_reserve + 2 * config.r_outer + config.h_upper_arm + config.h_flank) * 1e-3):show()

for _, arr in ipairs({ base_link, shoulder }) do
    for _, value in ipairs(arr) do
        value:show()
    end
end

local linkage = require('linkage')
upperarm[5] = linkage.m1:copy()
    :x((2 * config.r_outer + config.h_flank + config.h_flank_reserve) * 1e-3)
    :z((config.h_base + config.h_flank_reserve + 2 * config.r_outer) * 1e-3):show()
