#ifndef TEST
#define TEST
#endif
#define CATCH_CONFIG_MAIN

#include <stdint.h>
#include "catch.hpp"
#include "../pid_controller.h"

PID controller(100, 1, 0.5, 0.1, 1000, 2000, 1420);

TEST_CASE("Initial return") {
    REQUIRE(controller.Compute(30, 0) == 1420);
    increment_millis_by(1);
    REQUIRE(millis() == 1);
    REQUIRE(controller.Compute(30, 0) == 1420);
    increment_millis_by(99);
    REQUIRE(controller.Compute(30, 0) == 1451.5);
    increment_millis_by(100);
    REQUIRE(controller.Compute(30, 10) == 1432.5);
    controller.Reset(1420);
    REQUIRE(controller.Compute(30, 0) == 1420);
}

TEST_CASE("Reverse") {
    controller.Reset(1420);
    REQUIRE(controller.Compute(-30, 0) == 1420);
    increment_millis_by(1);
    REQUIRE(controller.Compute(-30, 0) == 1420);
    increment_millis_by(99);
    REQUIRE(controller.Compute(-30, 0) == 1388.5);
    increment_millis_by(100);
    REQUIRE(controller.Compute(-30, -10) == 1407.5);
    controller.Reset(1420);
    REQUIRE(controller.Compute(-30, 0) == 1420);
}