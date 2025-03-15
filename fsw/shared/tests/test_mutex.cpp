#include <lest/lest.hpp>

#define CASE(name) lest_CASE(specification(), name)
extern lest::tests& specification();

#include "crater/core/sync/Mutex.hpp"

using namespace crt::sync;

struct MockMutex {
    void lock()
    {
        locked = true;
    }

    void unlock()
    {
        locked = false;
    }

    bool try_lock()
    {
        locked = true;
        return true;
    }

    bool locked = false;
};

CASE(
    "Basic functionality"
    "[Mutex]")
{
    Mutex<int, MockMutex> m(123);

    EXPECT(m.mutex().locked == false);

    {
        auto val = m.lock();
        EXPECT(m.mutex().locked == true);

        EXPECT(*val == 123);
        *val = 321;
        EXPECT(*val == 321);
    }

    EXPECT(m.mutex().locked == false);
}

struct ImmovableType {
    int a;

    ImmovableType(int v)
        : a(v)
    {
    }

    ~ImmovableType() = default;

    ImmovableType(const ImmovableType& s) = delete;
    ImmovableType(ImmovableType&& s)      = delete;

    ImmovableType& operator=(const ImmovableType& other) = delete;
    ImmovableType& operator=(ImmovableType&& other)      = delete;
};

CASE(
    "Immovable Type"
    "[Mutex]")
{
    Mutex<ImmovableType, MockMutex> m(123);

    EXPECT(m.mutex().locked == false);

    {
        auto val = m.lock();
        EXPECT(m.mutex().locked == true);

        EXPECT(val->a == 123);
        ImmovableType& t = *val;
        EXPECT(t.a == 123);
    }

    EXPECT(m.mutex().locked == false);
}
