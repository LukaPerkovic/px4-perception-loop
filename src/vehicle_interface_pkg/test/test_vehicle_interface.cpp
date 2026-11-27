#include <gtest/gtest.h>
#include <array>

#include "vehicle_interface_pkg/vehicle_interface.hpp"


TEST(VehicleInterfaceTest, DuringFlight)
{   
    uint64_t tick = 3;
    uint64_t max_ticks = 6;

    StreamWatch test_watch;
    test_watch.mark_seen(tick);
    ++tick;
    ++tick;
    ASSERT_EQ(test_watch.check(tick, max_ticks), Freshness::FRESH);
    ++tick;
    ASSERT_EQ(test_watch.check(tick, max_ticks), Freshness::FRESH);
    ++tick;
    ++tick;
    ASSERT_EQ(test_watch.check(tick, max_ticks), Freshness::STALE);

}


TEST(VehicleInterfaceTest, NotStartedEdgeCase)
{
    uint64_t tick = 1;
    StreamWatch test_watch;
    uint64_t max_ticks = 6;


    ASSERT_EQ(test_watch.check(tick, max_ticks), Freshness::STALE);

}