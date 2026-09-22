// ROS 2 서비스 클라이언트 BT 노드 기반 클래스 (BehaviorTree.CPP v3).
//
// ros_action_node.hpp 와 같은 원칙: 생성자에서 서버를 기다리지 않고, tick 은 블로킹하지 않으며,
// 응답은 콜백(노드 실행기)으로 받는다. 시간 초과·halt 시 대기 중 요청을 클라이언트에서 지운다.
// 파생 클래스는 setRequest() 를 구현하고, 필요하면 onResponse() 를 덮어쓴다.
#ifndef AMR_BEHAVIOR__BT_NODES__ROS_SERVICE_NODE_HPP_
#define AMR_BEHAVIOR__BT_NODES__ROS_SERVICE_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/ros_node_params.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace amr_behavior
{

template<class ServiceT>
class RosServiceNode : public BT::ActionNodeBase
{
public:
  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;
  using Client = rclcpp::Client<ServiceT>;

  RosServiceNode(
    const std::string & name, const BT::NodeConfiguration & config, RosNodeParams params,
    const std::string & default_service)
  : BT::ActionNodeBase(name, config),
    params_(std::move(params)),
    service_name_(default_service),
    logger_(rclcpp::get_logger("amr_behavior")),
    shared_(std::make_shared<Shared>())
  {
    std::string remapped;
    if (getInput("service_name", remapped) && !remapped.empty()) {
      service_name_ = remapped;
    }
    auto node = params_.node.lock();
    if (!node) {
      throw std::runtime_error("RosServiceNode: rclcpp 노드가 없다");
    }
    logger_ = node->get_logger();
    client_ = params_.clients->template service<ServiceT>(service_name_);
  }

  ~RosServiceNode() override
  {
    dropPending();
  }

  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("service_name", "서비스 이름 (비우면 노드 기본값, 상대 이름)"),
      BT::InputPort<unsigned>("server_timeout", 0, "응답 대기 [ms] (0 = 실행기 기본값)"),
    };
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  static BT::PortsList providedPorts() {return providedBasicPorts({});}

  const std::string & serviceName() const {return service_name_;}

  BT::NodeStatus tick() override
  {
    if (status() == BT::NodeStatus::IDLE) {
      setStatus(BT::NodeStatus::RUNNING);
      unsigned timeout_ms = 0;
      server_timeout_ = params_.server_timeout;
      if (getInput("server_timeout", timeout_ms) && timeout_ms > 0) {
        server_timeout_ = std::chrono::milliseconds(timeout_ms);
      }
      request_ = std::make_shared<Request>();
      if (!setRequest(*request_)) {
        return finish(BT::NodeStatus::FAILURE, "요청 입력이 유효하지 않다");
      }
      waiting_service_ = true;
      stage_start_ = SteadyClock::now();
    }

    if (waiting_service_) {
      if (!client_->service_is_ready()) {
        if (elapsed() > params_.wait_for_server_timeout) {
          return finish(BT::NodeStatus::FAILURE, "서비스 서버 없음");
        }
        return BT::NodeStatus::RUNNING;
      }
      sendRequest();
    }

    typename Response::SharedPtr response;
    {
      std::lock_guard<std::mutex> lock(shared_->mutex);
      response = shared_->response;
    }
    if (!response) {
      if (elapsed() > server_timeout_) {
        dropPending();
        return finish(BT::NodeStatus::FAILURE, "응답 시간 초과");
      }
      return BT::NodeStatus::RUNNING;
    }
    invalidate();
    pending_ = false;
    return onResponse(*response);
  }

  void halt() override
  {
    dropPending();
    waiting_service_ = false;
    setStatus(BT::NodeStatus::IDLE);
  }

protected:
  /// 요청을 채운다. false 면 보내지 않고 FAILURE.
  virtual bool setRequest(Request & request) = 0;

  /// 응답 → BT 상태. 기본: 응답이 오면 SUCCESS.
  virtual BT::NodeStatus onResponse(const Response & /*response*/)
  {
    return BT::NodeStatus::SUCCESS;
  }

  RosNodeParams params_;
  std::string service_name_;
  rclcpp::Logger logger_;

private:
  using SteadyClock = std::chrono::steady_clock;

  struct Shared
  {
    std::mutex mutex;
    uint64_t generation{0};
    typename Response::SharedPtr response;
  };

  std::chrono::milliseconds elapsed() const
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
      SteadyClock::now() - stage_start_);
  }

  BT::NodeStatus finish(BT::NodeStatus status, const std::string & reason)
  {
    RCLCPP_WARN(logger_, "[%s] %s: %s", name().c_str(), service_name_.c_str(), reason.c_str());
    waiting_service_ = false;
    return status;
  }

  void sendRequest()
  {
    uint64_t generation = 0;
    {
      std::lock_guard<std::mutex> lock(shared_->mutex);
      generation = ++shared_->generation;
      shared_->response.reset();
    }
    std::weak_ptr<Shared> weak = shared_;
    auto sent = client_->async_send_request(
      request_, [weak, generation](typename Client::SharedFuture future) {
        if (auto s = weak.lock()) {
          std::lock_guard<std::mutex> lock(s->mutex);
          if (s->generation == generation) {
            s->response = future.get();
          }
        }
      });
    request_id_ = sent.request_id;
    pending_ = true;
    waiting_service_ = false;
    stage_start_ = SteadyClock::now();
  }

  void invalidate()
  {
    std::lock_guard<std::mutex> lock(shared_->mutex);
    ++shared_->generation;
    shared_->response.reset();
  }

  /// 대기 중 요청이 있으면 클라이언트 표에서 지운다 (늦은 응답은 버려진다).
  void dropPending()
  {
    invalidate();
    if (pending_ && client_) {
      client_->remove_pending_request(request_id_);
    }
    pending_ = false;
  }

  typename Client::SharedPtr client_;
  std::shared_ptr<Shared> shared_;
  typename Request::SharedPtr request_;
  int64_t request_id_{0};
  bool pending_{false};
  bool waiting_service_{false};
  SteadyClock::time_point stage_start_;
  std::chrono::milliseconds server_timeout_{1000};
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__ROS_SERVICE_NODE_HPP_
