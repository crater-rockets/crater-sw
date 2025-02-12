#include <miosix.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <spi/master.hpp>
#include <spi/slave.hpp>
#include <thread>
#include <vector>

#include "interfaces/interrupts.h"

using namespace std;
using namespace miosix;

chrono::seconds sleep_duration(1);

// SPI1 Master
GpioPin spi1_sck(GPIOA_BASE, 5);
GpioPin spi1_miso(GPIOA_BASE, 6);
GpioPin spi1_mosi(GPIOA_BASE, 7);
GpioPin spi1_cs(GPIOA_BASE, 4);

// SPI3 Slave
GpioPin spi3_sck(GPIOB_BASE, 3);
GpioPin spi3_miso(GPIOB_BASE, 4);
GpioPin spi3_mosi(GPIOB_BASE, 5);
GpioPin spi3_cs(GPIOD_BASE, 14);

SPI::Config config;
SPI::Slave slave(SPI1);
SPI::Master master(SPI3);

std::vector<uint8_t> master_buffer = {0x12, 0x34, 0x56};
std::vector<uint8_t> slave1_buffer = {0xAB, 0xCD, 0xEF};

void setSPIPins();
void printBuffer(std::vector<uint8_t>& buffer);
void IRQslave1();

int main()
{
    setSPIPins();

    // Enable SPI peripherals
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
    RCC_SYNC();

    // Configuring and enabling first the slave as suggested in the RM
    slave.configure(config);
    master.configure(config);

    printf("Before transfer:\n");
    printf("  master_buffer: ");
    printBuffer(master_buffer);
    printf("  slave1_buffer: ");
    printBuffer(slave1_buffer);

    spi3_cs.low();
    master_buffer.at(0) = master.transfer(master_buffer.at(0));
    master_buffer.at(1) = master.transfer(master_buffer.at(1));
    master_buffer.at(2) = master.transfer(master_buffer.at(2));
    spi3_cs.high();

    printf("After transfer:\n");
    printf("  master_buffer: ");
    printBuffer(master_buffer);
    printf("  slave1_buffer: ");
    printBuffer(slave1_buffer);
}

void setSPIPins()
{
    spi1_sck.mode(Mode::ALTERNATE);
    spi1_sck.alternateFunction(5);
    spi1_miso.mode(Mode::ALTERNATE);
    spi1_miso.alternateFunction(5);
    spi1_mosi.mode(Mode::ALTERNATE);
    spi1_mosi.alternateFunction(5);

    spi3_sck.mode(Mode::ALTERNATE_PULL_DOWN);
    spi3_sck.alternateFunction(6);
    spi3_miso.mode(Mode::ALTERNATE);
    spi3_miso.alternateFunction(6);
    spi3_mosi.mode(Mode::ALTERNATE);
    spi3_mosi.alternateFunction(6);
    spi3_cs.mode(Mode::OUTPUT);
    spi3_cs.high();

    // spi1_cs = PA4

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PA;
    EXTI->IMR |= 1 << 4;   // Unmask interrupt
    EXTI->RTSR |= 1 << 4;  // Enable rising edge trigger
    EXTI->FTSR |= 1 << 4;  // Enable falling edge trigger
    IRQregisterIrq(EXTI4_IRQn, IRQslave1);
}

void printBuffer(std::vector<uint8_t>& buffer)
{
    if (buffer.size() == 0)
        return;

    for (uint i = 0; i < buffer.size() - 1; i++)
        printf("%02X, ", buffer.at(i));
    printf("%02X\n", buffer.at(buffer.size() - 1));
}

void IRQslave1()
{
    EXTI->PR = EXTI_PR_PR4;
    if (spi1_cs.value() == 0)
        slave.IRQprepare(&slave1_buffer);
    else
        slave.IRQcleanup();
}
