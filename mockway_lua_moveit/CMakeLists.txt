cmake_minimum_required(VERSION 3.16)
project(mockway_lua_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

# ── 基础依赖 ──────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  control_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_eigen
  trajectory_msgs
)
foreach(dep IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

# ── Lua ──────────────────────────────────────────────────────────────────────
# 优先查找 Lua 5.4，其次 5.3
find_package(Lua 5.4 QUIET)
if(NOT LUA_FOUND)
  find_package(Lua 5.3 REQUIRED)
endif()
message(STATUS "Found Lua ${LUA_VERSION_STRING}: ${LUA_LIBRARIES}")

# ── sol2 (header-only，FetchContent 自动拉取) ──────────────────────────────
include(FetchContent)
FetchContent_Declare(
  sol2
  GIT_REPOSITORY https://github.com/ThePhD/sol2.git
  GIT_TAG        v3.3.0
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(sol2)

# ── 可执行目标 ───────────────────────────────────────────────────────────────
add_executable(lua_moveit_node src/lua_moveit_node.cpp)

target_include_directories(lua_moveit_node PRIVATE
  ${LUA_INCLUDE_DIR}
)

target_link_libraries(lua_moveit_node
  ${LUA_LIBRARIES}
  sol2
)

ament_target_dependencies(lua_moveit_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# ── 安装 ─────────────────────────────────────────────────────────────────────
install(
  TARGETS lua_moveit_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY lua launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
