#pragma once

#include <cstdint>
#include <optional>

enum class LinkGoal {IDLE, FLYING, LANDING};

inline const char* to_string(LinkGoal g)
{
    switch (g)
    {
    case LinkGoal::IDLE:
        return "IDLE";
    case LinkGoal::FLYING:
        return "FLYING";
    case LinkGoal::LANDING:
        return "LANDING";
    }
    
    return "UNKNOWN";
}

struct NedPoint {
    float x{0.F}; //North
    float y{0.F}; //East
    float z{0.F}; //Down (positive)
};


struct EnuPoint {
    float x{0.F}; //East
    float y{0.F}; //North
    float z{0.F}; //Up (positive)
};


enum class Freshness {NEVER_SEEN, FRESH, STALE};

struct StreamWatch
{
    std::optional<uint64_t> last_seen_tick {};

    void mark_seen(uint64_t tick)
    {
        last_seen_tick = tick;
    }

    Freshness check(uint64_t current_tick, uint64_t max_age) const
    {   
        if (!last_seen_tick.has_value()) { return Freshness::NEVER_SEEN; }
        if (current_tick - *last_seen_tick <= max_age)  { 
            return Freshness::FRESH; 
        } else { 
            return Freshness::STALE;
        }
    }
};     