#include <miosix.h>

#include <chrono>
#include <cstdio>
#include <thread>


using namespace std;

#include <fmt/core.h>

int main(int argc, char *argv[])
{
    chrono::seconds sleep_duration(1);
    while (true)
    {
        fmt::println("Hello world {}", 1);
        this_thread::sleep_for(sleep_duration);
    }
}
