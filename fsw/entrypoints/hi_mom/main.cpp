#include <miosix.h>
#include <test/test.h>

#include <chrono>
#include <thread>

using namespace std;

int main()
{
    chrono::seconds sleep_duration(1);

    while (true)
    {
        print_salute();
        this_thread::sleep_for(sleep_duration);
    }
}
