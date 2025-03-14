#include <lest/lest.hpp>

#define CASE(name) lest_CASE(specification(), name)
extern lest::tests& specification();

#include "crater/core/Channel.hpp"

using namespace crt;

CASE(
    "single producer single consumer"
    "[Channel]")
{
    SETUP("an empty SPSC channel")
    {
        Channel<int> ch{};

        Sender tx   = ch.sender();
        Receiver rx = ch.receiver(5);

        SECTION("receiving an empty channel returns nullopt")
        {
            EXPECT(rx.count() == 0);
            EXPECT_NOT(rx.try_receive());
        }

        SECTION("receiving one element after sending")
        {
            tx.send(1);
            EXPECT(rx.count() == 1);
            EXPECT(rx.try_receive() == 1);
            EXPECT(rx.try_receive() == std::nullopt);
        }

        SECTION("receiving multiple elements 1 at a time")
        {
            for (int i = 0; i < 10; ++i) {
                tx.send(i);
                EXPECT(rx.try_receive() == i);
            }

            EXPECT(rx.try_receive() == std::nullopt);
        }

        SECTION("buffer wraps around if too many elements")
        {
            for (int i = 0; i < 10; ++i) {
                tx.send(i);
            }

            for (int i = 5; i < 10; ++i) {
                EXPECT(rx.try_receive() == i);
            }

            EXPECT(rx.try_receive() == std::nullopt);
        }
    }
}


CASE(
    "multi producer single consumer"
    "[Channel]")
{
    SETUP("an empty SPSC channel")
    {
        Channel<int> ch{};

        Sender tx1   = ch.sender();
        Sender tx2   = ch.sender();
        Receiver rx = ch.receiver(5);

        SECTION("sending one element from different senders")
        {
            tx1.send(1);
            tx2.send(2);

            EXPECT(rx.count() == 2);
            EXPECT(rx.try_receive() == 1);
            EXPECT(rx.try_receive() == 2);
            EXPECT(rx.try_receive() == std::nullopt);
        }
    }
}

CASE(
    "single producer multi consumer"
    "[Channel]")
{
    SETUP("an empty SPSC channel")
    {
        Channel<int> ch{};

        Sender tx   = ch.sender();
        Receiver rx1 = ch.receiver(5);
        Receiver rx2 = ch.receiver(3); // Different buffer size

        SECTION("sending one element causes both receivers to receive it")
        {
            tx.send(1);
            
            EXPECT(rx1.count() == 1);
            EXPECT(rx2.count() == 1);
            EXPECT(rx1.try_receive() == 1);
            EXPECT(rx2.try_receive() == 1);

            EXPECT(rx1.try_receive() == std::nullopt);
            EXPECT(rx2.try_receive() == std::nullopt);
        }

        SECTION("one receiver can overflow while the other does not")
        {
            for (int i = 0; i < 5; ++i) {
                tx.send(i);
            }
            
            EXPECT(rx1.count() == 5);
            EXPECT(rx2.count() == 3);

            for (int i = 0; i < 5; ++i) {
                EXPECT(rx1.try_receive() == i);
            }

            for (int i = 2; i < 5; ++i) {
                EXPECT(rx2.try_receive() == i);
            }

            EXPECT(rx1.try_receive() == std::nullopt);
            EXPECT(rx2.try_receive() == std::nullopt);
        }
    }
}