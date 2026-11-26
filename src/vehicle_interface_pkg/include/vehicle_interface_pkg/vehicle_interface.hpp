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