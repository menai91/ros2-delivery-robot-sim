#include "amr_autonomy/payload.hpp"

namespace amr_autonomy
{
    void Payload::load()
    {
        loaded = true;
    }

    void Payload::unload()
    {
        loaded = false;
    }

    bool Payload::is_loaded() const
    {
        return loaded;
    }
}
