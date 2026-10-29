#pragma once

#include <memory>

#include "amr_autonomy/bt_move_to_position.hpp"
#include "amr_navigation/navigator.hpp"
#include "behaviortree_cpp/bt_factory.h"

namespace amr_autonomy
{
    //helper function to register nodes to the BehaviorTreeFactory
    inline void register_nodes(BT::BehaviorTreeFactory &factory, const std::shared_ptr<amr_navigation::Navigator> &navigator)
    {
        factory.registerNodeType<MoveToPosition>("MoveToPosition", navigator);
    }

} // namespace amr_autonomy
