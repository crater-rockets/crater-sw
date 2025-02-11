#pragma once

#include "defs.h"

namespace SPI
{

class Master
{
public:
    Master(SPI_TypeDef* spi) : spi(spi) {};

    ///< Delete copy/move contructors/operators.
    Master(const Master&)            = delete;
    Master& operator=(const Master&) = delete;
    Master(Master&&)                 = delete;
    Master& operator=(Master&&)      = delete;

    void configure(Config config)
    {
        // Wait until the peripheral is done before changing configuration
        while ((spi->SR & SPI_SR_TXE) == 0)
            ;
        while ((spi->SR & SPI_SR_BSY) > 0)
            ;

        // Disable the peripheral
        spi->CR1 &= ~SPI_CR1_SPE;

        // Configure clock polarity and phase
        spi->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);
        spi->CR1 |= static_cast<uint32_t>(config.mode);

        // Configure clock frequency
        spi->CR1 &= ~SPI_CR1_BR;
        spi->CR1 |= static_cast<uint32_t>(config.clock_divider);

        // Configure bit order
        spi->CR1 &= ~SPI_CR1_LSBFIRST;
        spi->CR1 |= static_cast<uint32_t>(config.bit_order);

        // Configure chip select and master mode
        spi->CR1 |= SPI_CR1_SSM;
        spi->CR1 |= SPI_CR1_SSI;
        spi->CR1 |= SPI_CR1_MSTR;

        /**
         * The SPI peripheral differs on stm32f7 microcontrollers. Refer to
         * AN4660 for a comprehensive differences list between different
         * peripherals versions.
         *
         * The main difference here is that on the f7 you can transmit between 4
         * and 16. There is also a 32bit fifo and a threshold that generates the
         * RXNE event. For this reason, on f7s we need to configure the 16 bit
         * frame format differently and change the fifo threshold level.
         */
#ifndef _ARCH_CORTEXM7_STM32F7
        spi->CR1 &= ~SPI_CR1_DFF;
#else
        spi->CR2 &= ~SPI_CR2_DS;
        spi->CR2 |= SPI_CR2_FRXTH;  // Set 8 bit RXNE
#endif

        // Enable the peripheral
        spi->CR1 |= SPI_CR1_SPE;
    }

    uint8_t transfer(uint8_t data)
    {
        /*
         * On STM32F7xx and STM32F4xx series chips, on SPI3 only, the RXNE flag
         * may be erroneously set at the beginning of the transaction with the
         * RX buffer containing garbage data.
         * On F7xx chips the issue can be reproduced by re-configuring the SPI
         * from Mode 0 (CPOL=0, CPHA=0) to Mode 3 (CPOL=1, CPHA=1), after
         * performing at least one transaction in Mode 0.
         *
         * We work around this issue by flushing the RX buffer at the beginning
         * of the transaction.
         */
        while ((spi->SR & SPI_SR_RXNE) != 0)
            spi->DR;

        // Wait until the peripheral is ready to transmit
        while ((spi->SR & SPI_SR_TXE) == 0)
            ;

        // Write the data item to transmit
        *(volatile uint8_t*)&spi->DR = static_cast<uint8_t>(data);

        // Make sure transmission is complete
        while ((spi->SR & SPI_SR_TXE) == 0)
            ;
        while ((spi->SR & SPI_SR_BSY) > 0)
            ;

        // Wait until data is received
        while ((spi->SR & SPI_SR_RXNE) == 0)
            ;

        // Read the received data item
        return static_cast<uint8_t>(spi->DR);
    }

private:
    SPI_TypeDef* spi;
};

}  // namespace SPI
