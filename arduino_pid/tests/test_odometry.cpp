#ifndef TEST
#define TEST
#endif
#define CATCH_CONFIG_MAIN

#define DIAMETER 0.1

#include <stdint.h>
#include "catch.hpp"
#include "../odometry.h"
#include <math.h>

TEST_CASE("Going straight ahead") {
    Odometry o(0, 0, 0, 0.1, DIAMETER, 40/(M_PI*2), 5);
    REQUIRE(o.GetX() == 0);
    REQUIRE(o.GetY() == 0);
    REQUIRE(o.GetOrientation() == 0);
    REQUIRE(o.GetDistance() == 0);

    o.Update(5, 0);
    double dist = DIAMETER*M_PI;
    REQUIRE(o.GetX() == Approx(dist));
    REQUIRE(o.GetY() == 0);
    REQUIRE(o.GetOrientation() == 0);
    REQUIRE(o.GetDistance() == dist);

    dist = 3*DIAMETER*M_PI;
    o.Update(15, 0);
    REQUIRE(o.GetX() == Approx(dist));
    REQUIRE(o.GetY() == 0);
    REQUIRE(o.GetOrientation() == 0);
    REQUIRE(o.GetDistance() == dist);
}

TEST_CASE("Going straight ahead backwards") {
    Odometry o(0, 0, 0, 0.1, DIAMETER, 40/(M_PI*2), 5);

    o.Update(-5, 0);
    double dist = DIAMETER*M_PI;
    REQUIRE(o.GetX() == Approx(-dist));
    REQUIRE(o.GetY() == 0);
    REQUIRE(o.GetOrientation() == 0);
    REQUIRE(o.GetDistance() == -dist);

    dist = 3*DIAMETER*M_PI;
    o.Update(-15, 0);
    REQUIRE(o.GetX() == Approx(-dist));
    REQUIRE(o.GetY() == 0);
    REQUIRE(o.GetOrientation() == 0);
    REQUIRE(o.GetDistance() == -dist);
}

TEST_CASE("Going straight 45 degrees left") {
    double angle = M_PI/4;
    Odometry o(0, 0, angle, 0.1, DIAMETER, 40/(M_PI*2), 5);

    o.Update(5, 0);
    double dist = DIAMETER*M_PI;
    REQUIRE(o.GetX() == Approx(1/sqrt(2)*dist));
    REQUIRE(o.GetY() == Approx(1/sqrt(2)*dist));
    REQUIRE(o.GetOrientation() == angle);
    REQUIRE(o.GetDistance() == dist);

    dist = 3*DIAMETER*M_PI;
    o.Update(15, 0);
    REQUIRE(o.GetX() == Approx(1/sqrt(2)*dist));
    REQUIRE(o.GetY() == Approx(1/sqrt(2)*dist));
    REQUIRE(o.GetOrientation() == angle);
    REQUIRE(o.GetDistance() == dist);
}

TEST_CASE("Going straight 90 degrees left") {
    double angle = M_PI/2;
    Odometry o(0, 0, angle, 0.1, DIAMETER, 40/(M_PI*2), 5);

    o.Update(5, 0);
    double dist = DIAMETER*M_PI;
    REQUIRE(round(o.GetX()) == round(0.0));
    REQUIRE(o.GetY() == Approx(dist));
    REQUIRE(o.GetOrientation() == angle);
    REQUIRE(o.GetDistance() == dist);

    dist = 3*DIAMETER*M_PI;
    o.Update(15, 0);
    REQUIRE(round(o.GetX()) == round(0.0));
    REQUIRE(o.GetY() == Approx(dist));
    REQUIRE(o.GetOrientation() == angle);
    REQUIRE(o.GetDistance() == dist);
}

TEST_CASE("Going straight backwards") {
    double angle = M_PI;
    Odometry o(0, 0, angle, 0.1, DIAMETER, 40/(M_PI*2), 5);

    o.Update(5, 0);
    double dist = DIAMETER*M_PI;
    REQUIRE(o.GetX() == Approx(-dist));
    REQUIRE(round(o.GetY()) == round(0.0));
    REQUIRE(o.GetOrientation() == angle);
    REQUIRE(o.GetDistance() == dist);

    dist = 3*DIAMETER*M_PI;
    o.Update(15, 0);
    REQUIRE(o.GetX() == Approx(-dist));
    REQUIRE(round(o.GetY()) == round(0.0));
    REQUIRE(o.GetOrientation() == angle);
    REQUIRE(o.GetDistance() == Approx(dist));
}

TEST_CASE("Going straight 90 degrees right") {
    double angle = 3*M_PI/2;
    Odometry o(0, 0, angle, 0.1, DIAMETER, 40/(M_PI*2), 5);

    o.Update(5, 0);
    double dist = DIAMETER*M_PI;
    REQUIRE(round(o.GetX()) == round(0.0));
    REQUIRE(o.GetY() == Approx(-dist));
    REQUIRE(o.GetOrientation() == angle);
    REQUIRE(o.GetDistance() == dist);

    dist = 3*DIAMETER*M_PI;
    o.Update(15, 0);
    REQUIRE(round(o.GetX()) == round(0.0));
    REQUIRE(o.GetY() == Approx(-dist));
    REQUIRE(o.GetOrientation() == angle);
    REQUIRE(o.GetDistance() == dist);
}