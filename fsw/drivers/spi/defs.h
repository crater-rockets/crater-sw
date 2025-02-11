#pragma once

#include <interfaces/arch_registers.h>
#include <stdint.h>

namespace SPI
{

enum class Order : uint16_t
{
    MSB_FIRST = 0,
    LSB_FIRST = SPI_CR1_LSBFIRST
};

/**
 * @brief SPI Clock divider.
 *
 * SPI clock frequency will be equal to the SPI peripheral bus clock speed
 * divided by the specified value.
 */
enum class ClockDivider : uint8_t
{
    DIV_2   = 0x00,
    DIV_4   = SPI_CR1_BR_0,
    DIV_8   = SPI_CR1_BR_1,
    DIV_16  = SPI_CR1_BR_1 | SPI_CR1_BR_0,
    DIV_32  = SPI_CR1_BR_2,
    DIV_64  = SPI_CR1_BR_2 | SPI_CR1_BR_0,
    DIV_128 = SPI_CR1_BR_2 | SPI_CR1_BR_1,
    DIV_256 = SPI_CR1_BR
};

enum class Mode : uint8_t
{
    /**
     * Clock low when idle, sample on first edge.
     * CPOL = 0, CPHA = 0
     */
    MODE_0 = 0,
    /**
     * Clock low when idle, sample on second edge.
     * CPOL = 0, CPHA = 1
     */
    MODE_1 = SPI_CR1_CPHA,
    /**
     * Clock high when idle, sample on first edge.
     * CPOL = 1, CPHA = 0
     */
    MODE_2 = SPI_CR1_CPOL,
    /**
     * Clock high when idle, sample on second edge.
     * CPOL = 1, CPHA = 1
     */
    MODE_3 = SPI_CR1_CPOL | SPI_CR1_CPHA
};

enum class WriteBit
{
    NORMAL,    ///< Normal write bit settings (0 for write, 1 for reads)
    INVERTED,  ///< Inverted write bit settings (1 for write, 0 for reads)
    DISABLED,  ///< Do not set write bit in any way
};

struct Config
{
    ClockDivider clock_divider;
    Mode mode;
    Order bit_order;
    WriteBit write_bit;

    Config(ClockDivider clock_divider = ClockDivider::DIV_256,
           Mode mode = Mode::MODE_0, Order bit_order = Order::MSB_FIRST,
           WriteBit write_bit         = WriteBit::NORMAL,
           unsigned int csSetupTimeUs = 0, unsigned int csHoldTimeUs = 0)
        : clock_divider(clock_divider), mode(mode), bit_order(bit_order),
          write_bit(write_bit)
    {
    }

    bool operator==(const Config& other) const
    {
        return clock_divider == other.clock_divider && mode == other.mode &&
               bit_order == other.bit_order;
    }

    bool operator!=(const Config& other) const { return !(*this == other); }
};

}  // namespace SPI
