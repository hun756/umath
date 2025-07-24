#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <umath/umath.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>


using namespace umath;

class MathTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        epsilon_f = std::numeric_limits<float>::epsilon() * 100.0f;
        epsilon_d = std::numeric_limits<double>::epsilon() * 100.0;
    }

    float epsilon_f;
    double epsilon_d;
};

TEST_F(MathTest, ArithmeticOverflowExactOperations)
{
    EXPECT_NO_THROW(Math<int>::addExact(100, 200));
    EXPECT_EQ(Math<int>::addExact(100, 200), 300);

    EXPECT_THROW(Math<int>::addExact(std::numeric_limits<int>::max(), 1), arithmetic_overflow);
    EXPECT_THROW(Math<int>::addExact(std::numeric_limits<int>::min(), -1), arithmetic_overflow);

    EXPECT_NO_THROW(Math<int>::subtractExact(200, 100));
    EXPECT_EQ(Math<int>::subtractExact(200, 100), 100);

    EXPECT_THROW(Math<int>::subtractExact(std::numeric_limits<int>::min(), 1), arithmetic_overflow);
    EXPECT_THROW(Math<int>::subtractExact(std::numeric_limits<int>::max(), -1),
                 arithmetic_overflow);

    EXPECT_NO_THROW(Math<int>::multiplyExact(100, 200));
    EXPECT_EQ(Math<int>::multiplyExact(100, 200), 20'000);

    EXPECT_THROW(Math<int>::multiplyExact(std::numeric_limits<int>::max(), 2), arithmetic_overflow);
    EXPECT_THROW(Math<int>::multiplyExact(std::numeric_limits<int>::min(), 2), arithmetic_overflow);
}

TEST_F(MathTest, AbsoluteValueFunction)
{
    EXPECT_EQ(Math<int>::abs(-5), 5);
    EXPECT_EQ(Math<int>::abs(5), 5);
    EXPECT_EQ(Math<int>::abs(0), 0);

    EXPECT_FLOAT_EQ(Math<float>::abs(-3.14f), 3.14f);
    EXPECT_FLOAT_EQ(Math<float>::abs(3.14f), 3.14f);
    EXPECT_FLOAT_EQ(Math<float>::abs(0.0f), 0.0f);

    EXPECT_DOUBLE_EQ(Math<double>::abs(-2.718), 2.718);
    EXPECT_DOUBLE_EQ(Math<double>::abs(2.718), 2.718);
    EXPECT_DOUBLE_EQ(Math<double>::abs(0.0), 0.0);
}

TEST_F(MathTest, MinMaxFunctions)
{
    EXPECT_EQ(Math<int>::max(5, 10), 10);
    EXPECT_EQ(Math<int>::max(10, 5), 10);
    EXPECT_EQ(Math<int>::max(-5, -10), -5);

    EXPECT_EQ(Math<int>::min(5, 10), 5);
    EXPECT_EQ(Math<int>::min(10, 5), 5);
    EXPECT_EQ(Math<int>::min(-5, -10), -10);

    EXPECT_FLOAT_EQ(Math<float>::max(3.14f, 2.71f), 3.14f);
    EXPECT_FLOAT_EQ(Math<float>::min(3.14f, 2.71f), 2.71f);

    EXPECT_DOUBLE_EQ(Math<double>::max(3.14159, 2.71828), 3.14159);
    EXPECT_DOUBLE_EQ(Math<double>::min(3.14159, 2.71828), 2.71828);
}

TEST_F(MathTest, SignumFunction)
{
    EXPECT_EQ(Math<int>::signum(5), 1);
    EXPECT_EQ(Math<int>::signum(-5), -1);
    EXPECT_EQ(Math<int>::signum(0), 0);

    EXPECT_FLOAT_EQ(Math<float>::signum(3.14f), 1.0f);
    EXPECT_FLOAT_EQ(Math<float>::signum(-3.14f), -1.0f);
    EXPECT_FLOAT_EQ(Math<float>::signum(0.0f), 0.0f);

    EXPECT_DOUBLE_EQ(Math<double>::signum(2.718), 1.0);
    EXPECT_DOUBLE_EQ(Math<double>::signum(-2.718), -1.0);
    EXPECT_DOUBLE_EQ(Math<double>::signum(0.0), 0.0);
}