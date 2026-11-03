#pragma once

#include <memory>

#include "amr_autonomy/bt_clear_payload.hpp"
#include "amr_autonomy/bt_drop_off_item.hpp"
#include "amr_autonomy/bt_get_next_task.hpp"
#include "amr_autonomy/bt_is_item_on_board.hpp"
#include "amr_autonomy/bt_move_to_position.hpp"
#include "amr_autonomy/bt_pick_up_item.hpp"
#include "amr_autonomy/payload.hpp"

#include "amr_navigation/navigator.hpp"

#include "behaviortree_cpp/bt_factory.h"

namespace amr_autonomy
{
    // helper function to register nodes to the BehaviorTreeFactory
    inline void register_nodes(BT::BehaviorTreeFactory &factory
        , const std::shared_ptr<amr_navigation::Navigator> &navigator
        , const std::shared_ptr<TaskQueue> &task_queue
        , const std::shared_ptr<Payload> &payload)
    {
        factory.registerNodeType<MoveToPosition>("MoveToPosition", navigator);
        factory.registerNodeType<PickupItem>("PickupItem", payload);
        factory.registerNodeType<DropoffItem>("DropoffItem", payload);
        factory.registerNodeType<IsItemOnBoard>("IsItemOnBoard", payload);
        factory.registerNodeType<ClearPayload>("ClearPayload", payload);
        factory.registerNodeType<GetNextTask>("GetNextTask", task_queue);
    }

} // namespace amr_autonomy
