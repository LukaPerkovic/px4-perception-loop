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
enum class TickStream {CMD, INTENT, STATUS, FAILSAFE, POS}

struct FreshnessStatus
{
    std::optional<uint64_t> last_cmd_tick      {};
    std::optional<uint64_t> last_intent_tick   {};
    std::optional<uint64_t> last_status_tick   {};
    std::optional<uint64_t> last_failsafe_tick {};
    std::optional<uint64_t> last_pos_tick      {};

    Freshness check(TickStream tstr, uint64_t current_tick, uint64_t max_age) const
    {   
        std::optional<uint64_t> val {};
        switch tstr
        {
            case TickStream::CMD:      val = last_cmd_tick;
            case TickStream::INTENT:   val = last_intent_tick;
            case TickStream::STATUS:   val = last_status_tick;
            case TickStream::FAILSAFE: val = last_failsafe_tick;
            case TickStream::POS:      val = last_pos_tick;

            default: return Freshness::NEVER_SEEN;
        }

        if (!val.has_value()) { return Freshness::NEVER_SEEN; }
        if (current_tick + val < max_age) { return Freshness::FRESH; }
        if (current_tick + val > max_age) { return Freshness::STALE; }
    }
};     