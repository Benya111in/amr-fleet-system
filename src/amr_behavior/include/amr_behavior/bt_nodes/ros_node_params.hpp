// ROS BT 노드 공통 의존성: rclcpp 노드(약한 참조), 타임아웃, 클라이언트 캐시.
//
// 같은 액션/서비스를 쓰는 BT 노드 인스턴스가 여러 개(서브트리마다)여도 클라이언트는 이름당 하나만
// 만든다 (DDS 엔티티 수 절감 — 5대 운용 시 발견 트래픽).
#ifndef AMR_BEHAVIOR__BT_NODES__ROS_NODE_PARAMS_HPP_
#define AMR_BEHAVIOR__BT_NODES__ROS_NODE_PARAMS_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <typeinfo>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace amr_behavior
{

/// 액션/서비스 클라이언트를 (타입, 이름) 별로 한 번만 만든다.
class RosClientCache
{
public:
  explicit RosClientCache(const rclcpp::Node::SharedPtr & node)
  : node_(node) {}

  template<class ActionT>
  typename rclcpp_action::Client<ActionT>::SharedPtr action(const std::string & name)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::string key = std::string("action|") + typeid(ActionT).name() + "|" + name;
    auto it = clients_.find(key);
    if (it != clients_.end()) {
      return std::static_pointer_cast<rclcpp_action::Client<ActionT>>(it->second);
    }
    auto client = rclcpp_action::create_client<ActionT>(lockNode(), name);
    clients_[key] = client;
    return client;
  }

  template<class ServiceT>
  typename rclcpp::Client<ServiceT>::SharedPtr service(const std::string & name)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::string key = std::string("service|") + typeid(ServiceT).name() + "|" + name;
    auto it = clients_.find(key);
    if (it != clients_.end()) {
      return std::static_pointer_cast<rclcpp::Client<ServiceT>>(it->second);
    }
    auto client = lockNode()->create_client<ServiceT>(name);
    clients_[key] = client;
    return client;
  }

  std::size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return clients_.size();
  }

private:
  rclcpp::Node::SharedPtr lockNode() const
  {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error("RosClientCache: rclcpp 노드가 이미 해제되었다");
    }
    return node;
  }

  std::weak_ptr<rclcpp::Node> node_;
  mutable std::mutex mutex_;
  std::map<std::string, std::shared_ptr<void>> clients_;
};

/// ROS BT 노드 생성자에 주입하는 설정.
struct RosNodeParams
{
  std::weak_ptr<rclcpp::Node> node;
  std::shared_ptr<RosClientCache> clients;
  /// goal/요청 전송 후 서버 응답(수락) 대기 상한
  std::chrono::milliseconds server_timeout{1000};
  /// 서버가 발견되지 않을 때 기다리는 상한 (그 뒤 FAILURE). 실행기는 Nav2 보다 먼저 떠도 된다.
  std::chrono::milliseconds wait_for_server_timeout{5000};

  static RosNodeParams make(const rclcpp::Node::SharedPtr & node)
  {
    RosNodeParams p;
    p.node = node;
    p.clients = std::make_shared<RosClientCache>(node);
    return p;
  }
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__ROS_NODE_PARAMS_HPP_
