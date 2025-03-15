#include <fmt/base.h>
#include "lest/lest.hpp"

#define CASE(name) lest_CASE(specification(), name)

lest::tests& specification()
{
    static lest::tests tests;
    return tests;
}

int main(int argc, char* argv[])
{
    if(lest::run(specification(), argc, argv /*, std::cout */) == 0) {
        fmt::println("Tests completed successfully");
    }
}
