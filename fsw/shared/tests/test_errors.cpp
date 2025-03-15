#include <fmt/core.h>

#include <lest/lest.hpp>
#include <nonstd/expected.hpp>
#include <string>

#include "crater/core/errors/Error.hpp"
#include "crater/core/errors/Try.hpp"
#include "crater/core/types/NonZero.hpp"

#define CASE(name) lest_CASE(specification(), name)

extern lest::tests& specification();

struct TestErrorData {
    int additional_data;
};

template <>
struct crt::ErrorDataToString<TestErrorData> {
    static std::string data_string(const TestErrorData& data)
    {
        return fmt::format("Test data={}", data.additional_data);
    }
};

struct TestErrorData2 {
    float additional_data;
};

template <>
struct crt::ErrorDataToString<TestErrorData2> {
    static std::string data_string(const TestErrorData2& data)
    {
        return fmt::format("Probability of impact={}", data.additional_data);
    }
};

crt::Expected<int, TestErrorData> error1()
{
    return crt::err(crt::ErrorCode::MockError1, TestErrorData{.additional_data = 123});
}

crt::Expected<float, TestErrorData2> error2()
{
    return crt::err(crt::ErrorCode::MockError2, TestErrorData2{.additional_data = 123.0F});
}

crt::Expected<crt::NonZero<int>> error_void()
{
    return crt::err(crt::ErrorCode::MockError2);
}

crt::AnyExpected<int> error_aggregate(int selector)
{
    switch (selector) {
        case 1: {
            int v = TRY(error1());
            return v;
        }
        case 2: {
            float v = TRY(error2());
            return static_cast<int>(v);
        }
        case 3: {
            crt::NonZero<int> v = TRY(error_void());
            return v.v;
        }
        default:
            return selector;
    }

    return selector;
}

CASE(
    "Simple error"
    "[Errors]")
{
    crt::Error err(crt::ErrorCode::MockError1, TestErrorData{.additional_data = 123});

    EXPECT(err.code() == crt::ErrorCode::MockError1);
    EXPECT(err.code_str() == std::string{"MockError1"});

    EXPECT(err.data().additional_data == 123);
    EXPECT(err.message() == "Error MockError1:254 - Test data=123");
}

CASE(
    "Simple void error"
    "[Errors]")
{
    crt::Error err(crt::ErrorCode::MockError1);

    EXPECT(err.code() == crt::ErrorCode::MockError1);
    EXPECT(err.code_str() == std::string{"MockError1"});

    EXPECT(err.message() == "Error MockError1:254");
}

CASE(
    "Simple string error"
    "[Errors]")
{
    crt::Error err(crt::ErrorCode::MockError1, "Hello world");

    EXPECT(err.code() == crt::ErrorCode::MockError1);
    EXPECT(err.code_str() == std::string{"MockError1"});

    EXPECT(err.message() == "Error MockError1:254. Hello world");

    crt::Error err2(crt::ErrorCode::MockError1, std::string{"Hello world"});
    EXPECT(err2.message() == "Error MockError1:254. Hello world");
}

CASE(
    "Any Expected"
    "[Errors]")
{
    crt::AnyExpected<int> any1 = error_aggregate(100);
    EXPECT(any1 == 100);

    crt::AnyExpected<int> any2 = error_aggregate(1);
    EXPECT_NOT(any2);

    EXPECT(any2.error().code() == crt::ErrorCode::MockError1);
    EXPECT(any2.error().message() == "Error MockError1:254 - Test data=123");

    std::optional<crt::Error<TestErrorData>> error2 = any2.error().downcast<crt::Error<TestErrorData>>();
    EXPECT(error2);
    EXPECT(error2.value().data().additional_data == 123);
}

