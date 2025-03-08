#include "lest/lest.hpp"
#include <fmt/core.h>

#define CASE(name) lest_CASE(specification(), name)

lest::tests& specification()
{
    static lest::tests tests;
    return tests;
}

int main(int argc, char* argv[])
{
    int result = lest::run(specification(), argc, argv /*, std::cout */);

    if (result == 0) {
        fmt::println("Tests completed successfully");
    } else {
        fmt::println("Tests completed with ERRORS");
    }
}
