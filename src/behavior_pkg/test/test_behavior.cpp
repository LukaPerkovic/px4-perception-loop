#include <gtest/gtest.h>
#include <array>

#include "behavior_pkg/behavior.hpp"


TEST(BehaviorTest, Handles16Frames)
{
    std::array<bool, 16> frames = {false, false,
                            true, true, true, true, true, true, true,
                            false, false, false, false,
                            true, true, true
                                };
    FrameFilter F(3, 3);

    for (size_t i = 0; i < frames.size(); ++i)
    {
        if (i == 4){
            ASSERT_EQ(F.check_validity(frames[i]), Event::TARGET_CONFIRMED) << "frame index " << i;
        } else if (i == 11) {
            ASSERT_EQ(F.check_validity(frames[i]), Event::TARGET_LOST) << "frame index " << i;
        } else if (i == 15) {
            ASSERT_EQ(F.check_validity(frames[i]), Event::TARGET_CONFIRMED) << "frame index " << i;
        } else {
            ASSERT_EQ(F.check_validity(frames[i]), Event::NO_EVENT)  << "frame index " << i;
        }
    }                            
}


TEST(BehaviorTest, HandlesEmptySky)
{
    FrameFilter F(3, 3);

    ASSERT_EQ(F.check_validity(false), Event::NO_EVENT);
    ASSERT_EQ(F.check_validity(false), Event::NO_EVENT);
    ASSERT_EQ(F.check_validity(false), Event::NO_EVENT);
    ASSERT_EQ(F.check_validity(false), Event::NO_EVENT);
}


TEST(BehaviorTest, HandlesNextState)
{
    ASSERT_EQ(next_state(State::SEARCH, Event::TARGET_CONFIRMED), State::TRACK);
    ASSERT_EQ(next_state(State::SEARCH, Event::TARGET_LOST)     , State::SEARCH);
    ASSERT_EQ(next_state(State::SEARCH, Event::NO_EVENT)        , State::SEARCH);
    ASSERT_EQ(next_state(State::TRACK,  Event::TARGET_CONFIRMED), State::TRACK);
    ASSERT_EQ(next_state(State::TRACK,  Event::TARGET_LOST)     , State::SEARCH);
    ASSERT_EQ(next_state(State::TRACK,  Event::NO_EVENT)        , State::TRACK);
}