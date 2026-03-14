/**
 * http_server.cpp
 *
 * 前端 HTTP/SSE 后端服务实现。
 */

// lua_moveit_node.hpp must be included first to pull in Eigen before httplib.h,
// because resolv.h (included transitively by httplib.h) defines `#define _res (*__res_state())`
// which corrupts Eigen template parameter names.
#include "mockway_lua_moveit/lua_moveit_node.hpp"
#include "mockway_lua_moveit/http_server.hpp"

#include <nlohmann/json.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>

using json = nlohmann::json;

// ─────────────────────────── 辅助 ────────────────────────────────────────────
static void set_cors(httplib::Response& res)
{
  res.set_header("Access-Control-Allow-Origin",  "*");
  res.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  res.set_header("Access-Control-Allow-Headers", "Content-Type");
}

// ─────────────────────────── 构造 / 析构 ─────────────────────────────────────
HttpServer::HttpServer(std::shared_ptr<LuaMoveItNode> node, int port)
: node_(std::move(node)), port_(port)
{
  setup_routes();
}

HttpServer::~HttpServer()
{
  stop();
}

// ─────────────────────────── 路径辅助 ────────────────────────────────────────
std::string HttpServer::web_root() const
{
  try {
    return ament_index_cpp::get_package_share_directory("mockway_lua_moveit") + "/dist";
  } catch (...) {
    return "";
  }
}

std::string HttpServer::config_path() const
{
  try {
    return ament_index_cpp::get_package_share_directory("mockway_lua_moveit")
           + "/system_config.json";
  } catch (...) {
    return "system_config.json";
  }
}

// ─────────────────────────── 生命周期 ────────────────────────────────────────
void HttpServer::start()
{
  if (running_) return;
  running_ = true;
  thread_ = std::thread([this]() {
    RCLCPP_INFO(node_->get_logger(),
      "HTTP 服务器启动中，端口 %d，Web 根目录: %s", port_, web_root().c_str());
    if (!svr_.listen("0.0.0.0", port_)) {
      RCLCPP_ERROR(node_->get_logger(), "HTTP 服务器启动失败");
    }
    running_ = false;
  });
}

void HttpServer::stop()
{
  if (!running_) return;
  svr_.stop();
  if (thread_.joinable()) thread_.join();
}

// ─────────────────────────── 路由注册 ────────────────────────────────────────
void HttpServer::setup_routes()
{
  // ── 静态文件 ────────────────────────────────────────────────────────────
  const std::string root = web_root();
  if (!root.empty()) {
    svr_.set_mount_point("/", root);
  }

  // ── CORS 预检 ──────────────────────────────────────────────────────────
  svr_.Options(".*", [](const httplib::Request&, httplib::Response& res) {
    set_cors(res);
    res.status = 204;
  });

  // ══════════════════════════════════════════════════════════════════════
  // POST /api/lua — 执行 Lua 脚本，返回输出
  // 请求体:  { "script": "<lua code>" }
  // 响应:    { "success": bool, "output": str } | { "error": str }
  // ══════════════════════════════════════════════════════════════════════
  svr_.Post("/api/lua", [this](const httplib::Request& req, httplib::Response& res) {
    set_cors(res);
    res.set_header("Content-Type", "application/json");

    try {
      auto j = json::parse(req.body);

      if (!j.contains("script")) {
        res.status = 400;
        res.set_content(json{{"error", "Missing 'script' field"}}.dump(), "application/json");
        return;
      }

      auto [success, output] = node_->run_string_captured(j["script"].get<std::string>());

      json resp;
      resp["success"] = success;
      if (success) {
        resp["message"] = "Script executed successfully";
        resp["output"]  = output;
      } else {
        resp["message"] = "Script execution failed";
        resp["error"]   = output;
        res.status = 400;
      }
      res.set_content(resp.dump(), "application/json");

    } catch (const json::exception& e) {
      res.status = 400;
      res.set_content(
        json{{"error", std::string("JSON parse error: ") + e.what()}}.dump(),
        "application/json");
    } catch (const std::exception& e) {
      res.status = 500;
      res.set_content(
        json{{"error", std::string("Server error: ") + e.what()}}.dump(),
        "application/json");
    }
  });

  // ══════════════════════════════════════════════════════════════════════
  // GET /api/joints — SSE 实时关节/位姿数据流（100 ms 周期）
  // SSE 数据格式:
  //   joints        : [j1..j6]  (deg)
  //   commandJoints : [j1..j6]  (deg，当前与 joints 相同)
  //   pose          : [x, y, z, roll, pitch, yaw]  (m / rad)
  //   errorId       : int
  //   errorMessage  : string
  //   globalRatio   : number  (0~100)
  // ══════════════════════════════════════════════════════════════════════
  svr_.Get("/api/joints", [this](const httplib::Request&, httplib::Response& res) {
    set_cors(res);
    res.set_header("Content-Type",  "text/event-stream");
    res.set_header("Cache-Control", "no-cache");
    res.set_header("Connection",    "keep-alive");

    res.set_chunked_content_provider(
      "text/event-stream",
      [this](size_t /*offset*/, httplib::DataSink& sink) -> bool {
        if (!rclcpp::ok() || !running_) {
          sink.done();
          return false;
        }

        // 获取关节角（rad → deg）
        const auto joints_rad = node_->get_joint_positions_raw();
        std::vector<double> joints_deg(6, 0.0);
        if (!joints_rad.empty()) {
          for (size_t i = 0; i < std::min(joints_rad.size(), size_t{6}); ++i)
            joints_deg[i] = joints_rad[i] * 180.0 / M_PI;
        }

        // 获取末端位姿
        const auto pose = node_->get_end_pose_rpy_raw();
        std::vector<double> pose6(6, 0.0);
        if (!pose.empty())
          pose6 = std::vector<double>(pose.begin(), pose.begin() + std::min(pose.size(), size_t{6}));

        json data;
        data["joints"]        = joints_deg;
        data["commandJoints"] = joints_deg;   // 暂与实际值相同
        data["pose"]          = pose6;
        data["errorId"]       = 0;
        data["errorMessage"]  = "";
        data["globalRatio"]   = node_->get_global_ratio();

        const std::string msg = "data: " + data.dump() + "\n\n";
        if (!sink.write(msg.c_str(), msg.size())) {
          return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return true;
      }
    );
  });

  // ══════════════════════════════════════════════════════════════════════
  // GET /api/config — 读取系统配置
  // ══════════════════════════════════════════════════════════════════════
  svr_.Get("/api/config", [this](const httplib::Request&, httplib::Response& res) {
    set_cors(res);
    std::ifstream ifs(config_path());
    if (!ifs.is_open()) {
      res.status = 404;
      res.set_content(json{{"error", "Config file not found"}}.dump(), "application/json");
      return;
    }
    std::string content(
      (std::istreambuf_iterator<char>(ifs)),
       std::istreambuf_iterator<char>());
    res.set_content(content, "application/json");
  });

  // ══════════════════════════════════════════════════════════════════════
  // POST /api/config — 写入系统配置
  // ══════════════════════════════════════════════════════════════════════
  svr_.Post("/api/config", [this](const httplib::Request& req, httplib::Response& res) {
    set_cors(res);
    res.set_header("Content-Type", "application/json");
    try {
      // 验证为合法 JSON
      auto j = json::parse(req.body);

      std::ofstream ofs(config_path());
      if (!ofs.is_open()) {
        res.status = 500;
        res.set_content(
          json{{"error", "Cannot write config: " + config_path()}}.dump(),
          "application/json");
        return;
      }
      ofs << j.dump(4);
      res.set_content(json{{"success", true}}.dump(), "application/json");

    } catch (const json::exception& e) {
      res.status = 400;
      res.set_content(
        json{{"error", std::string("Invalid JSON: ") + e.what()}}.dump(),
        "application/json");
    }
  });
}
