// ROS 2 액션 클라이언트 BT 노드 기반 클래스 (BehaviorTree.CPP v3).
//
// nav2_behavior_tree::BtActionNode 와 같은 역할이지만 다음이 다르다.
//  1) 생성자에서 서버를 기다리지 않는다 → 실행기가 Nav2/도킹 서버보다 먼저 떠도 트리를 만든다.
//     (BtActionNode 는 생성 시 wait_for_action_server 후 없으면 예외 → 트리 생성 실패)
//  2) tick 은 절대 블로킹하지 않는다. goal 응답·피드백·결과는 노드 실행기가 처리한 콜백으로
//     받는다.
//  3) 콜백은 weak_ptr + 세대 번호로 보호한다: halt/소멸 뒤 늦게 도착한 응답은 무시하고,
//     늦게 수락된 goal 은 즉시 취소한다 (고아 goal 방지).
// 파생 클래스는 setGoal() 만 구현하면 되고, 필요하면 onResult()/onFeedback() 를 덮어쓴다.
#ifndef AMR_BEHAVIOR__BT_NODES__ROS_ACTION_NODE_HPP_
#define AMR_BEHAVIOR__BT_NODES__ROS_ACTION_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/ros_node_params.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace amr_behavior
{

template<class ActionT>
class RosActionNode : public BT::ActionNodeBase
{
public:
  using Goal = typename ActionT::Goal;
  using Feedback = typename ActionT::Feedback;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename GoalHandle::WrappedResult;
  using Client = rclcpp_action::Client<ActionT>;

  RosActionNode(
    const std::string & name, const BT::NodeConfiguration & config, RosNodeParams params,
    const std::string & default_server)
  : BT::ActionNodeBase(name, config),
    params_(std::move(params)),
    server_name_(default_server),
    logger_(rclcpp::get_logger("amr_behavior")),
    shared_(std::make_shared<Shared>())
  {
    std::string remapped;
    if (getInput("server_name", remapped) && !remapped.empty()) {
      server_name_ = remapped;
    }
    auto node = params_.node.lock();
    if (!node) {
      throw std::runtime_error("RosActionNode: rclcpp 노드가 없다");
    }
    logger_ = node->get_logger();
    client_ = params_.clients->template action<ActionT>(server_name_);
  }

  ~RosActionNode() override
  {
    cancelActiveGoal();
  }

  /// 파생 클래스 providedPorts() 에서 호출해 공통 포트를 붙인다.
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("server_name", "액션 서버 이름 (비우면 노드 기본값, 상대 이름)"),
      BT::InputPort<unsigned>("server_timeout", 0, "goal 수락 대기 [ms] (0 = 실행기 기본값)"),
    };
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  static BT::PortsList providedPorts() {return providedBasicPorts({});}

  const std::string & serverName() const {return server_name_;}

  BT::NodeStatus tick() override
  {
    if (status() == BT::NodeStatus::IDLE) {
      setStatus(BT::NodeStatus::RUNNING);
      unsigned timeout_ms = 0;
      server_timeout_ = params_.server_timeout;
      if (getInput("server_timeout", timeout_ms) && timeout_ms > 0) {
        server_timeout_ = std::chrono::milliseconds(timeout_ms);
      }
      if (skipGoal()) {
        return finish(BT::NodeStatus::SUCCESS, "");
      }
      goal_ = Goal();
      if (!setGoal(goal_)) {
        return finish(BT::NodeStatus::FAILURE, "goal 입력이 유효하지 않다");
      }
      stage_ = Stage::kWaitServer;
      stage_start_ = SteadyClock::now();
    }

    if (stage_ == Stage::kWaitServer) {
      if (!client_->action_server_is_ready()) {
        if (elapsed() > params_.wait_for_server_timeout) {
          return finish(BT::NodeStatus::FAILURE, "액션 서버 없음");
        }
        return BT::NodeStatus::RUNNING;
      }
      sendGoal();
    }

    if (stage_ == Stage::kWaitAccept) {
      bool received = false;
      bool accepted = false;
      {
        std::lock_guard<std::mutex> lock(shared_->mutex);
        received = shared_->response_received;
        accepted = static_cast<bool>(shared_->handle);
      }
      if (!received) {
        if (elapsed() > server_timeout_) {
          invalidate();   // 늦게 수락되면 콜백에서 취소된다
          return finish(BT::NodeStatus::FAILURE, "goal 응답 시간 초과");
        }
        return BT::NodeStatus::RUNNING;
      }
      if (!accepted) {
        return finish(BT::NodeStatus::FAILURE, "goal 거절");
      }
      stage_ = Stage::kWaitResult;
    }

    // kWaitResult
    std::shared_ptr<const Feedback> feedback;
    bool done = false;
    WrappedResult result;
    {
      std::lock_guard<std::mutex> lock(shared_->mutex);
      feedback = std::move(shared_->feedback);
      shared_->feedback.reset();
      done = shared_->result_received;
      if (done) {
        result = shared_->result;
      }
    }
    if (feedback) {
      onFeedback(*feedback);
    }
    if (!done) {
      return BT::NodeStatus::RUNNING;
    }
    invalidate();
    stage_ = Stage::kIdle;
    return onResult(result);
  }

  void halt() override
  {
    if (status() == BT::NodeStatus::RUNNING) {
      cancelActiveGoal();
    }
    stage_ = Stage::kIdle;
    setStatus(BT::NodeStatus::IDLE);
  }

protected:
  /// true 면 서버에 보내지 않고 곧바로 SUCCESS (예: 양보 자세가 없을 때 제자리 대기).
  virtual bool skipGoal() {return false;}

  /// goal 을 채운다. false 면 서버에 보내지 않고 FAILURE.
  virtual bool setGoal(Goal & goal) = 0;

  /// 결과 → BT 상태. 기본: SUCCEEDED 만 SUCCESS.
  virtual BT::NodeStatus onResult(const WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_WARN(
      logger_, "[%s] %s 결과 %s", name().c_str(), server_name_.c_str(),
      result.code == rclcpp_action::ResultCode::ABORTED ? "ABORTED" : "CANCELED");
    return BT::NodeStatus::FAILURE;
  }

  virtual void onFeedback(const Feedback & /*feedback*/) {}

  RosNodeParams params_;
  std::string server_name_;
  rclcpp::Logger logger_;

private:
  using SteadyClock = std::chrono::steady_clock;
  enum class Stage {kIdle, kWaitServer, kWaitAccept, kWaitResult};

  /// 콜백과 공유하는 상태. BT 노드가 먼저 소멸해도 콜백은 weak_ptr 로 안전하다.
  struct Shared
  {
    std::mutex mutex;
    uint64_t generation{0};
    bool response_received{false};
    typename GoalHandle::SharedPtr handle;
    bool result_received{false};
    WrappedResult result;
    std::shared_ptr<const Feedback> feedback;
  };

  std::chrono::milliseconds elapsed() const
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
      SteadyClock::now() - stage_start_);
  }

  BT::NodeStatus finish(BT::NodeStatus status, const std::string & reason)
  {
    if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_WARN(logger_, "[%s] %s: %s", name().c_str(), server_name_.c_str(), reason.c_str());
    }
    stage_ = Stage::kIdle;
    return status;
  }

  void sendGoal()
  {
    uint64_t generation = 0;
    {
      std::lock_guard<std::mutex> lock(shared_->mutex);
      generation = ++shared_->generation;
      shared_->response_received = false;
      shared_->handle.reset();
      shared_->result_received = false;
      shared_->feedback.reset();
    }
    std::weak_ptr<Shared> weak = shared_;
    std::weak_ptr<Client> weak_client = client_;
    typename Client::SendGoalOptions options;
    options.goal_response_callback =
      [weak, weak_client, generation](typename GoalHandle::SharedPtr handle) {
        bool stale = true;
        if (auto s = weak.lock()) {
          std::lock_guard<std::mutex> lock(s->mutex);
          if (s->generation == generation) {
            s->response_received = true;
            s->handle = handle;
            stale = false;
          }
        }
        auto client = weak_client.lock();
        if (stale && handle && client) {
          try {
            client->async_cancel_goal(handle);
          } catch (const std::exception &) {
          }
        }
      };
    options.feedback_callback =
      [weak, generation](
      typename GoalHandle::SharedPtr, const std::shared_ptr<const Feedback> feedback) {
        if (auto s = weak.lock()) {
          std::lock_guard<std::mutex> lock(s->mutex);
          if (s->generation == generation) {
            s->feedback = feedback;
          }
        }
      };
    options.result_callback = [weak, generation](const WrappedResult & result) {
        if (auto s = weak.lock()) {
          std::lock_guard<std::mutex> lock(s->mutex);
          if (s->generation == generation) {
            s->result = result;
            s->result_received = true;
          }
        }
      };
    client_->async_send_goal(goal_, options);
    stage_ = Stage::kWaitAccept;
    stage_start_ = SteadyClock::now();
  }

  /// 이후 도착하는 콜백을 무시하게 만든다.
  void invalidate()
  {
    std::lock_guard<std::mutex> lock(shared_->mutex);
    ++shared_->generation;
    shared_->response_received = false;
    shared_->handle.reset();
    shared_->result_received = false;
    shared_->feedback.reset();
  }

  /// 실행 중인 goal 이 있으면 취소 요청(비블로킹)을 보내고 상태를 비운다.
  void cancelActiveGoal()
  {
    typename GoalHandle::SharedPtr handle;
    {
      std::lock_guard<std::mutex> lock(shared_->mutex);
      if (!shared_->result_received) {
        handle = shared_->handle;
      }
    }
    invalidate();
    if (handle && client_) {
      try {
        client_->async_cancel_goal(handle);
      } catch (const std::exception &) {
      }
    }
  }

  typename Client::SharedPtr client_;
  std::shared_ptr<Shared> shared_;
  Goal goal_;
  Stage stage_{Stage::kIdle};
  SteadyClock::time_point stage_start_;
  std::chrono::milliseconds server_timeout_{1000};
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__ROS_ACTION_NODE_HPP_
