#include <fmt/core.h>

#include <catch2/catch_test_macros.hpp>

#include "crater/core/collections/CircularBuffer.hpp"

using namespace crater::collections;

TEST_CASE("Push & pop", "[CircularBuffer]")
{
    CircularBuffer<int> cb(3);
    REQUIRE(cb.size() == 3);

    REQUIRE(cb.empty());
    REQUIRE(cb.count() == 0);

    REQUIRE(cb.pop() == std::nullopt);
    REQUIRE(cb.empty());
    REQUIRE(cb.count() == 0);

    cb.push(1);
    REQUIRE(cb.empty() == false);
    REQUIRE(cb.count() == 1);

    cb.push(2);
    REQUIRE(cb.empty() == false);
    REQUIRE(cb.count() == 2);

    cb.push(3);
    REQUIRE(cb.empty() == false);
    REQUIRE(cb.count() == 3);

    REQUIRE(cb.pop() == 1);
    REQUIRE(cb.empty() == false);
    REQUIRE(cb.count() == 2);

    REQUIRE(cb.pop() == 2);
    REQUIRE(cb.empty() == false);
    REQUIRE(cb.count() == 1);

    REQUIRE(cb.pop() == 3);
    REQUIRE(cb.empty() == true);
    REQUIRE(cb.count() == 0);

    REQUIRE(cb.pop() == std::nullopt);
}

TEST_CASE("Overflow", "[CircularBuffer]")
{
    CircularBuffer<int> cb(3);
    REQUIRE(cb.pop() == std::nullopt);

    cb.push(1);
    cb.push(2);
    cb.push(3);
    REQUIRE(cb.count() == 3);

    cb.push(4);
    REQUIRE(cb.count() == 3);

    REQUIRE(cb.pop() == 2);
    REQUIRE(cb.pop() == 3);
    REQUIRE(cb.pop() == 4);

    REQUIRE(cb.count() == 0);

    REQUIRE(cb.pop() == std::nullopt);
}

TEST_CASE("Tail after head", "[CircularBuffer]")
{
    CircularBuffer<int> cb(5);
    REQUIRE(cb.pop() == std::nullopt);

    cb.push(1);
    cb.push(2);
    cb.push(3);
    cb.push(4);
    cb.push(5);

    REQUIRE(cb.pop());
    REQUIRE(cb.pop());
    REQUIRE(cb.pop());

    REQUIRE(cb.count() == 2);

    cb.push(6);
    REQUIRE(cb.count() == 3);
    cb.push(7);
    REQUIRE(cb.count() == 4);
}

