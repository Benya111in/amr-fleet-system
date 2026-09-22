// BT 노드 공통 도우미: 컨텍스트 포인터 별칭, 포트 기본값 읽기.
#ifndef AMR_BEHAVIOR__BT_NODES__BT_UTILS_HPP_
#define AMR_BEHAVIOR__BT_NODES__BT_UTILS_HPP_

#include <memory>
#include <string>

#include "amr_behavior/executor_context.hpp"
#include "behaviortree_cpp_v3/tree_node.h"

namespace amr_behavior
{

using ContextPtr = std::shared_ptr<ExecutorContext>;

/// 포트 값을 읽고, 없거나 변환에 실패하면 fallback 을 돌려준다.
template<typename T>
T inputOr(const BT::TreeNode & node, const std::string & key, const T & fallback)
{
  T value{};
  if (node.getInput<T>(key, value)) {
    return value;
  }
  return fallback;
}

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__BT_UTILS_HPP_
