#pragma once

namespace amr_autonomy
{
    // A small class that is used to determine if the robot is carrying an item or not.
    class Payload
    {
    public:
        void load();
        void unload();
        bool is_loaded() const;

    private:
        bool loaded = false;
    };
}
