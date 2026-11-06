#pragma once

namespace amr_autonomy
{
    /*Enum used only by the behavior tree xml.*/
    enum AttemptResult : unsigned int
    {
        Attempt_Delivered,
        // Attempt failed, as long as it did not fail due to the goal being outside of the map, attempt to do it later
        Attempt_Retry,
        // Nav2 error 204: a goal is outside the map, so we will ignore / drop this attempt
        Attempt_GoalOutsideMap,
        // an attempt failed and the item could not be returned to the pickup station
        Attempt_ReturnFailed,
    };
}
