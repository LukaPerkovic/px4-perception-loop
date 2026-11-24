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