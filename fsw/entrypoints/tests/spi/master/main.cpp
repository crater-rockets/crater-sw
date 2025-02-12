#include <miosix.h>

#include <chrono>
#include <cstdio>
#include <spi/master.hpp>
#include <thread>

using namespace std;
using namespace miosix;

chrono::seconds sleep_duration(1);

// SPI1 Master
GpioPin spi1_sck(GPIOA_BASE, 5);
GpioPin spi1_miso(GPIOA_BASE, 6);
GpioPin spi1_mosi(GPIOA_BASE, 7);

void set_spi_pins();

int main()
{
    set_spi_pins();

    // Enable SPI peripherals
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    SPI::Config config;
    SPI::Master master(SPI1);
    master.configure(config);

    while (true)
    {
        uint8_t buffer = master.transfer(0xAB);
        printf("Read: %02X\n", buffer);
        this_thread::sleep_for(sleep_duration);
    }
}

void set_spi_pins()
{
    spi1_sck.mode(Mode::ALTERNATE);
    spi1_sck.alternateFunction(5);
    spi1_sck.speed(Speed::_100MHz);
    spi1_miso.mode(Mode::ALTERNATE);
    spi1_miso.alternateFunction(5);
    spi1_mosi.mode(Mode::ALTERNATE);
    spi1_mosi.alternateFunction(5);
    spi1_sck.mode(Mode::ALTERNATE);
}
