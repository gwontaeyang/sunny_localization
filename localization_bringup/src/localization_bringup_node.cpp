#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <string>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <logger/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace
{
constexpr const char * kNodeName = "localization_bringup_node";
constexpr const char * kLoggerName = "localization_bringup";

enum class BootstrapState { Pending, Done, Ignored, Failed };

struct ManagedNode
{
  std::string name;
  std::string initial_state{"inactive"};
  std::string lifecycle_state{"unknown"};
  int check_count{0};
  BootstrapState state{BootstrapState::Pending};
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client;
};
}  // namespace

class LocalizationBringupNode : public rclcpp::Node
{
public:
  LocalizationBringupNode() : Node(kNodeName)
  {
    logger_ = gole::logger::Logger::createPackageLogger(kLoggerName);
    config_file_path_ = declare_parameter<std::string>("config_file_path", "");
    if (config_file_path_.empty()) {
      logger_->critical("config_file_path parameter is required");
      throw std::runtime_error("config_file_path parameter is required");
    }

    client_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    loadConfig(config_file_path_);
    node_states_pub_ = create_publisher<std_msgs::msg::String>("~/node_lifecycle_states", 10);

    const auto period = std::chrono::duration<double>(
      check_frequency_ > 0.0 ? 1.0 / check_frequency_ : 0.5);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LocalizationBringupNode::timerCallback, this));

    logger_->info(
      "{} lifecycle bootstrap started (nodes={}, check_frequency={:.2f}, max_check_count={})",
      kLoggerName, managed_nodes_.size(), check_frequency_, max_check_count_);
  }

private:
  void loadConfig(const std::string & path)
  {
    const auto root = YAML::LoadFile(path);
    const auto params = root[kNodeName]["ros__parameters"];
    check_frequency_ = params["check_frequency"] ? params["check_frequency"].as<double>() : 2.0;
    max_check_count_ = params["max_check_count"] ? params["max_check_count"].as<int>() : 10;

    const auto lifecycle_nodes = params["lifecycle_nodes"];
    if (!lifecycle_nodes || !lifecycle_nodes.IsMap()) {
      logger_->warn("No lifecycle_nodes configured in {}", path);
      return;
    }

    for (const auto entry : lifecycle_nodes) {
      ManagedNode node;
      node.name = entry.first.as<std::string>();
      const auto config = entry.second;
      node.initial_state = config["initial_state"] ? config["initial_state"].as<std::string>() : "inactive";
      if (node.initial_state != "active" && node.initial_state != "inactive") {
        logger_->warn("[{}] Invalid initial_state '{}'; using inactive", node.name, node.initial_state);
        node.initial_state = "inactive";
      }
      managed_nodes_.emplace(node.name, std::move(node));
    }
  }

  void timerCallback()
  {
    if (!bootstrap_completed_) {
      runBootstrapStep();
      if (allDone()) {
        bootstrap_completed_ = true;
        logger_->info("{} lifecycle bootstrap completed", kLoggerName);
      }
    } else {
      refreshLifecycleStates();
    }
    publishNodeStates();
  }

  void runBootstrapStep()
  {
    for (auto & entry : managed_nodes_) {
      auto & node = entry.second;
      if (node.state != BootstrapState::Pending) {
        continue;
      }
      if (!isNodeVisible(node.name)) {
        markMissingOrRetry(node);
        continue;
      }
      ensureClients(node);
      if (!node.get_state_client->wait_for_service(0ms) || !node.change_state_client->wait_for_service(0ms)) {
        markMissingOrRetry(node);
        continue;
      }
      bootstrapLifecycleNode(node);
    }
  }

  bool isNodeVisible(const std::string & name) const
  {
    const auto names = get_node_names();
    const auto absolute = "/" + name;
    for (const auto & visible : names) {
      if (visible == name || visible == absolute) {
        return true;
      }
    }
    return false;
  }

  void markMissingOrRetry(ManagedNode & node)
  {
    node.check_count++;
    if (max_check_count_ > 0 && node.check_count >= max_check_count_) {
      node.state = BootstrapState::Ignored;
      node.lifecycle_state = "ignored";
      logger_->info("[{}] Not detected after {} checks; ignored", node.name, node.check_count);
    }
  }

  void ensureClients(ManagedNode & node)
  {
    if (!node.get_state_client) {
      node.get_state_client = create_client<lifecycle_msgs::srv::GetState>(
        "/" + node.name + "/get_state", rmw_qos_profile_services_default, client_callback_group_);
    }
    if (!node.change_state_client) {
      node.change_state_client = create_client<lifecycle_msgs::srv::ChangeState>(
        "/" + node.name + "/change_state", rmw_qos_profile_services_default, client_callback_group_);
    }
  }

  void bootstrapLifecycleNode(ManagedNode & node)
  {
    const auto state = getState(node);
    if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN) {
      return;
    }
    if (node.initial_state == "active" && state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      node.state = BootstrapState::Done;
      logger_->info("[{}] Already active", node.name);
      return;
    }
    if (node.initial_state == "inactive" && state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      node.state = BootstrapState::Done;
      logger_->info("[{}] Already inactive", node.name);
      return;
    }
    if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
      if (!changeState(node, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, "configure")) {
        node.state = BootstrapState::Failed;
        node.lifecycle_state = "failed";
        return;
      }
      getState(node);
      if (node.initial_state == "inactive") {
        node.state = BootstrapState::Done;
        logger_->info("[{}] Configured and left inactive", node.name);
        return;
      }
    }
    if (node.initial_state == "active") {
      const auto current = getState(node);
      if (current != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
          current != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        node.state = BootstrapState::Failed;
        node.lifecycle_state = "failed";
        logger_->error("[{}] Cannot activate from state id {}", node.name, current);
        return;
      }
      if (current == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE ||
          changeState(node, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, "activate")) {
        getState(node);
        node.state = BootstrapState::Done;
        logger_->info("[{}] Activated", node.name);
      } else {
        node.state = BootstrapState::Failed;
        node.lifecycle_state = "failed";
      }
      return;
    }
    node.state = BootstrapState::Failed;
    node.lifecycle_state = "failed";
    logger_->error("[{}] Unsupported lifecycle state id {} for target '{}'", node.name, state, node.initial_state);
  }

  unsigned int getState(ManagedNode & node)
  {
    auto future = node.get_state_client->async_send_request(std::make_shared<lifecycle_msgs::srv::GetState::Request>());
    if (future.wait_for(2s) != std::future_status::ready) {
      logger_->warn("[{}] get_state timeout", node.name);
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
    try {
      const auto state = future.get()->current_state.id;
      node.lifecycle_state = lifecycleStateToString(state);
      return state;
    } catch (const std::exception & e) {
      logger_->error("[{}] get_state failed: {}", node.name, e.what());
      node.state = BootstrapState::Failed;
      node.lifecycle_state = "failed";
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
  }

  bool changeState(ManagedNode & node, unsigned int transition_id, const std::string & label)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition_id;
    request->transition.label = label;
    auto future = node.change_state_client->async_send_request(request);
    if (future.wait_for(5s) != std::future_status::ready) {
      logger_->warn("[{}] {} timeout", node.name, label);
      return false;
    }
    const auto response = future.get();
    if (!response->success) {
      logger_->error("[{}] {} transition rejected", node.name, label);
    }
    return response->success;
  }

  void refreshLifecycleStates()
  {
    for (auto & entry : managed_nodes_) {
      auto & node = entry.second;
      if (!isNodeVisible(node.name)) {
        continue;
      }
      ensureClients(node);
      if (node.get_state_client->wait_for_service(0ms)) {
        getState(node);
      }
    }
  }

  void publishNodeStates()
  {
    std_msgs::msg::String msg;
    bool first = true;
    for (const auto & entry : managed_nodes_) {
      if (!first) {
        msg.data += "|";
      }
      msg.data += entry.second.name + ";" + statusString(entry.second);
      first = false;
    }
    node_states_pub_->publish(msg);
  }

  std::string statusString(const ManagedNode & node) const
  {
    if (node.lifecycle_state != "unknown") {
      return node.lifecycle_state;
    }
    if (node.state == BootstrapState::Pending) {
      return "pending";
    }
    if (node.state == BootstrapState::Ignored) {
      return "ignored";
    }
    if (node.state == BootstrapState::Failed) {
      return "failed";
    }
    return node.initial_state;
  }

  std::string lifecycleStateToString(unsigned int id) const
  {
    switch (id) {
      case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED: return "unconfigured";
      case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE: return "inactive";
      case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE: return "active";
      case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED: return "finalized";
      default: return "unknown";
    }
  }

  bool allDone() const
  {
    for (const auto & entry : managed_nodes_) {
      if (entry.second.state == BootstrapState::Pending) {
        return false;
      }
    }
    return true;
  }

  std::string config_file_path_;
  double check_frequency_{2.0};
  int max_check_count_{10};
  bool bootstrap_completed_{false};
  std::map<std::string, ManagedNode> managed_nodes_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr node_states_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<spdlog::logger> logger_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<LocalizationBringupNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & e) {
    auto logger = gole::logger::Logger::getPackageLogger(kLoggerName);
    if (logger) {
      logger->error("Exception: {}", e.what());
    } else {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "Exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 1;
  }
  gole::logger::Logger::shutdown();
  rclcpp::shutdown();
  return 0;
}
