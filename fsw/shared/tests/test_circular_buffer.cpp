#include <fmt/core.h>

#include <lest/lest.hpp>
#include <nonstd/expected.hpp>

#include "crater/core/collections/CircularBuffer.hpp"
#include "crater/Common.hpp"

#define CASE(name) lest_CASE(specification(), name)

extern lest::tests& specification();

using namespace crt::collections;

CASE(
    "Push & Pop"
    "[CircularBuffer]")
{
    CircularBuffer<int> cb(3);
    EXPECT(cb.size() == 3);

    EXPECT(cb.empty());
    EXPECT(cb.count() == 0);

    EXPECT(cb.pop() == std::nullopt);
    EXPECT(cb.empty());
    EXPECT(cb.count() == 0);

    cb.push(1);
    EXPECT(cb.empty() == false);
    EXPECT(cb.count() == 1);

    cb.push(2);
    EXPECT(cb.empty() == false);
    EXPECT(cb.count() == 2);

    cb.push(3);
    EXPECT(cb.empty() == false);
    EXPECT(cb.count() == 3);

    EXPECT(cb.pop() == 1);
    EXPECT(cb.empty() == false);
    EXPECT(cb.count() == 2);

    EXPECT(cb.pop() == 2);
    EXPECT(cb.empty() == false);
    EXPECT(cb.count() == 1);

    EXPECT(cb.pop() == 3);
    EXPECT(cb.empty() == true);
    EXPECT(cb.count() == 0);

    EXPECT(cb.pop() == std::nullopt);
}

CASE(
    "Push & pop"
    "[CircularBuffer]")
{
    CircularBuffer<int> cb(3);
    EXPECT(cb.size() == 3);

    EXPECT(cb.empty());
    EXPECT(cb.count() == 0);

    EXPECT(cb.pop() == std::nullopt);
    EXPECT(cb.empty());
    EXPECT(cb.count() == 0);

    cb.push(1);
    EXPECT(cb.empty() == false);
    EXPECT(cb.count() == 1);

    cb.push(2);
    EXPECT(cb.empty() == false);
    EXPECT(cb.count() == 2);

    cb.push(3);
    EXPECT(cb.empty() == false);
    EXPECT(cb.count() == 3);

    EXPECT(cb.pop() == 1);
    EXPECT(cb.empty() == false);
    EXPECT(cb.count() == 2);

    EXPECT(cb.pop() == 2);
    EXPECT(cb.empty() == false);
    EXPECT(cb.count() == 1);

    EXPECT(cb.pop() == 3);
    EXPECT(cb.empty() == true);
    EXPECT(cb.count() == 0);

    EXPECT(cb.pop() == std::nullopt);
}

CASE(
    "Overflow"
    "[CircularBuffer]")
{
    CircularBuffer<int> cb(3);
    EXPECT(cb.pop() == std::nullopt);

    cb.push(1);
    cb.push(2);
    cb.push(3);
    EXPECT(cb.count() == 3);

    cb.push(4);
    EXPECT(cb.count() == 3);

    EXPECT(cb.pop() == 2);
    EXPECT(cb.pop() == 3);
    EXPECT(cb.pop() == 4);

    EXPECT(cb.count() == 0);

    EXPECT(cb.pop() == std::nullopt);
}

CASE(
    "Tail after head"
    "[CircularBuffer]")
{
    CircularBuffer<int> cb(5);
    EXPECT(cb.pop() == std::nullopt);

    cb.push(1);
    cb.push(2);
    cb.push(3);
    cb.push(4);
    cb.push(5);

    EXPECT(cb.pop());
    EXPECT(cb.pop());
    EXPECT(cb.pop());

    EXPECT(cb.count() == 2);

    cb.push(6);
    EXPECT(cb.count() == 3);
    cb.push(7);
    EXPECT(cb.count() == 4);
}

