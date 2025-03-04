#include <catch2/catch_test_macros.hpp>
#include <nonstd/expected.hpp>
#include <string>

#include "crater/core/collections/CircularBuffer.hpp"
#include "crater/core/errors/Try.hpp"
#include <fmt/core.h>

nonstd::expected<int, std::string> test(bool ok)
{
    if (ok)
        return 1;
    else
        return nonstd::make_unexpected("Shit");
}

nonstd::expected<int, std::string> test2(bool ok)
{
    int val = TRY(test(ok));
    return val;
}

TEST_CASE("ABCD", "[Abcd]")
{
    REQUIRE(true);

    auto v = test(true);

    fmt::println("Hello world");
}
