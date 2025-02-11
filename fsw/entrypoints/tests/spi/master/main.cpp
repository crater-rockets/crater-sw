#include <miosix.h>

#include <chrono>
#include <cstdio>
#include <spi/master.hpp>
#include <thread>

using namespace std;
using namespace miosix;

int main()
{
    chrono::seconds sleep_duration(1);

    GpioPin sckPin(GPIOA_BASE, 5);
    GpioPin misoPin(GPIOA_BASE, 6);
    GpioPin mosiPin(GPIOA_BASE, 7);

    sckPin.mode(Mode::ALTERNATE);
    sckPin.alternateFunction(5);
    sckPin.speed(Speed::_100MHz);
    misoPin.mode(Mode::ALTERNATE);
    misoPin.alternateFunction(5);
    mosiPin.mode(Mode::ALTERNATE);
    mosiPin.alternateFunction(5);

    SPI::Config config;
    SPI::Master master(SPI1);

    printf("Configuring...\n");
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    master.configure(config);

    printf("Starting the loop...\n");
    while (true)
    {
        uint8_t buffer = master.transfer(0xAB);
        printf("Read: %02X\n", buffer);
        this_thread::sleep_for(sleep_duration);
    }
}
