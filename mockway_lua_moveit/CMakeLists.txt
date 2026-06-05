cmake_minimum_required(VERSION 3.16)
project(mockway_lua_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

# ── 基础依赖 ──────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  ament_index_cpp
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

# ── sol2 (via ros-jazzy-sol-vendor) ───────────────────────────────────────
find_package(sol_vendor REQUIRED)

# ── cpp-httplib（shared library on Ubuntu） ───────────────────────────────
find_package(PkgConfig REQUIRED)
pkg_check_modules(HTTPLIB REQUIRED cpp-httplib)
message(STATUS "Found cpp-httplib: ${HTTPLIB_INCLUDE_DIRS}, libs: ${HTTPLIB_LIBRARIES}")

# ── nlohmann_json ─────────────────────────────────────────────────────────
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  find_path(NLOHMANN_JSON_INCLUDE_DIR NAMES nlohmann/json.hpp
    HINTS /usr/include /usr/local/include
    DOC "nlohmann/json.hpp header directory"
  )
  if(NOT NLOHMANN_JSON_INCLUDE_DIR)
    message(FATAL_ERROR
      "nlohmann/json.hpp not found. Install with:\n"
      "  sudo apt install nlohmann-json3-dev")
  endif()
  message(STATUS "Found nlohmann/json: ${NLOHMANN_JSON_INCLUDE_DIR}/nlohmann/json.hpp")
endif()

# ── 前端构建（可选） ────────────────────────────────────────────────────────
option(BUILD_FRONTEND "Build the mockway_lua_moveit web UI during colcon build" OFF)
if(BUILD_FRONTEND)
  find_program(NPM_EXECUTABLE npm)
  if(NPM_EXECUTABLE)
    add_custom_target(build_frontend ALL
      COMMAND "${NPM_EXECUTABLE}" install --prefer-offline
      COMMAND "${NPM_EXECUTABLE}" run build
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ui
      COMMENT "Building frontend UI..."
      VERBATIM
    )
  else()
    message(WARNING
      "npm not found — frontend will NOT be built.\n"
      "Install Node.js/npm first, or build the UI manually with: cd ui && npm install && npm run build")
  endif()
endif()

# ── 可执行目标 ───────────────────────────────────────────────────────────────
add_executable(lua_moveit_node
  src/main.cpp
  src/lua_moveit_node.cpp
  src/http_server.cpp
)

target_include_directories(lua_moveit_node PRIVATE
  include
  ${LUA_INCLUDE_DIR}
  ${HTTPLIB_INCLUDE_DIRS}
)
target_compile_options(lua_moveit_node PRIVATE ${HTTPLIB_CFLAGS_OTHER})

if(nlohmann_json_FOUND)
  target_link_libraries(lua_moveit_node
    ${LUA_LIBRARIES}
    ${HTTPLIB_LIBRARIES}
    nlohmann_json::nlohmann_json
  )
else()
  target_include_directories(lua_moveit_node PRIVATE ${NLOHMANN_JSON_INCLUDE_DIR})
  target_link_libraries(lua_moveit_node ${LUA_LIBRARIES} ${HTTPLIB_LIBRARIES})
endif()

ament_target_dependencies(lua_moveit_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  sol_vendor
)

# cpp-httplib 需要 pthread
target_link_libraries(lua_moveit_node pthread)

# ── 安装 ─────────────────────────────────────────────────────────────────────
install(
  TARGETS lua_moveit_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY lua launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装前端构建产物（若已构建）
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/ui/dist)
  install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ui/dist
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
