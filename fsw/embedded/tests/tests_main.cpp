#include <catch2/catch_session.hpp>

int main(int argc, char* argv[])
{
    int result = Catch::Session().run(argc, argv);

    while (true) {
        printf("Done.\n");
    }
}
