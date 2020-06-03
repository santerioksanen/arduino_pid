#ifndef TEST
#define TEST
#endif
#define CATCH_CONFIG_MAIN

#define DIAMETER 0.1
#define WHEELBASE 0.5

#include <stdint.h>
#include "catch.hpp"
#include "../odometry.h"
#include <math.h>

#define MAX_STEERING_ANGLE 0.6981317    // 40 deg in radians

TEST_CASE("Going straight ahead") {
    Odometry o(0, 0, 0, WHEELBASE, DIAMETER, MAX_STEERING_ANGLE, 5);
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
    Odometry o(0, 0, 0, WHEELBASE, DIAMETER, MAX_STEERING_ANGLE, 5);

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
    Odometry o(0, 0, angle, WHEELBASE, DIAMETER, MAX_STEERING_ANGLE, 5);

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
    Odometry o(0, 0, angle, WHEELBASE, DIAMETER, MAX_STEERING_ANGLE, 5);

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
    Odometry o(0, 0, angle, WHEELBASE, DIAMETER, MAX_STEERING_ANGLE, 5);

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
    Odometry o(0, 0, angle, WHEELBASE, DIAMETER, MAX_STEERING_ANGLE, 5);

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

TEST_CASE("Turning left with a steering angle of 20 degrees"){
    Odometry o(0, 0, 0, WHEELBASE, DIAMETER, MAX_STEERING_ANGLE, 5);

    o.Update(5, 0.5);
    REQUIRE(o.GetOrientation() == Approx(0.22869));
    REQUIRE(o.GetDistance() == Approx(DIAMETER*M_PI));
    REQUIRE(o.GetX() == Approx(0.3114275));
    REQUIRE(o.GetY() == Approx(0.035766));

    // We have travelled so that total turn is > 90 deg
    o.Update(45, 0.5);
    REQUIRE(o.GetOrientation() == Approx(2.05821));
    REQUIRE(o.GetDistance() == Approx(9*DIAMETER*M_PI));
    REQUIRE(o.GetX() == Approx(1.213767));
    REQUIRE(o.GetY() == Approx(2.0171));

    // We have travelled so that total turn is > 180 deg
    o.Update(90, 0.5);
    REQUIRE(o.GetOrientation() == Approx(4.11642));
    REQUIRE(o.GetDistance() == Approx(18*DIAMETER*M_PI));
    REQUIRE(o.GetX() == Approx(-1.1369));
    REQUIRE(o.GetY() == Approx(2.144848));

    // We have travelled so that total turn is > 270 deg
    o.Update(135, 0.5);
    REQUIRE(o.GetOrientation() == Approx(6.17463));
    REQUIRE(o.GetDistance() == Approx(27*DIAMETER*M_PI));
    REQUIRE(o.GetX() == Approx(-0.148861));
    REQUIRE(o.GetY() == Approx(0.0080893));
}

TEST_CASE("Turning right with a steering angle of 20 degrees"){
    Odometry o(0, 0, 0, WHEELBASE, DIAMETER, MAX_STEERING_ANGLE, 5);

    o.Update(5, -0.5);
    REQUIRE(o.GetOrientation() == Approx(2*M_PI-0.22869));
    REQUIRE(o.GetDistance() == Approx(DIAMETER*M_PI));
    REQUIRE(o.GetX() == Approx(0.3114275));
    REQUIRE(o.GetY() == Approx(-0.035766));

    // We have travelled so that total turn is > 90 deg
    o.Update(45, -0.5);
    REQUIRE(o.GetOrientation() == Approx(2*M_PI - 2.05821));
    REQUIRE(o.GetDistance() == Approx(9*DIAMETER*M_PI));
    REQUIRE(o.GetX() == Approx(1.213767));
    REQUIRE(o.GetY() == Approx(-2.0171));

    // We have travelled so that total turn is > 270 deg
    o.Update(90, -0.5);
    REQUIRE(o.GetOrientation() == Approx(2*M_PI - 4.11642));
    REQUIRE(o.GetDistance() == Approx(18*DIAMETER*M_PI));
    REQUIRE(o.GetX() == Approx(-1.1369));
    REQUIRE(o.GetY() == Approx(-2.144848));

    // We have travelled so that total turn is > 270 deg
    o.Update(135, -0.5);
    REQUIRE(o.GetOrientation() == Approx(0.1085757));
    REQUIRE(o.GetDistance() == Approx(27*DIAMETER*M_PI));
    REQUIRE(o.GetX() == Approx(-0.148861));
    REQUIRE(o.GetY() == Approx(-0.0080893));
}