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

TEST_F(MathTest, TrigonometricFunctions)
{
    constexpr float PI_F = 3.14159265358979323846f;
    constexpr double PI_D = 3.14159265358979323846;

    EXPECT_NEAR(Math<float>::sin(0.0f), 0.0f, epsilon_f);
    EXPECT_NEAR(Math<float>::sin(PI_F / 2.0f), 1.0f, epsilon_f);
    EXPECT_NEAR(Math<float>::sin(PI_F), 0.0f, epsilon_f);
    EXPECT_NEAR(Math<float>::sin(3.0f * PI_F / 2.0f), -1.0f, epsilon_f);

    EXPECT_NEAR(Math<double>::sin(0.0), 0.0, epsilon_d);
    EXPECT_NEAR(Math<double>::sin(PI_D / 2.0), 1.0, epsilon_d);
    EXPECT_NEAR(Math<double>::sin(PI_D), 0.0, epsilon_d);
    EXPECT_NEAR(Math<double>::sin(3.0 * PI_D / 2.0), -1.0, epsilon_d);

    EXPECT_NEAR(Math<float>::cos(0.0f), 1.0f, epsilon_f);
    EXPECT_NEAR(Math<float>::cos(PI_F / 2.0f), 0.0f, epsilon_f);
    EXPECT_NEAR(Math<float>::cos(PI_F), -1.0f, epsilon_f);
    EXPECT_NEAR(Math<float>::cos(3.0f * PI_F / 2.0f), 0.0f, epsilon_f);

    EXPECT_NEAR(Math<double>::cos(0.0), 1.0, epsilon_d);
    EXPECT_NEAR(Math<double>::cos(PI_D / 2.0), 0.0, epsilon_d);
    EXPECT_NEAR(Math<double>::cos(PI_D), -1.0, epsilon_d);
    EXPECT_NEAR(Math<double>::cos(3.0 * PI_D / 2.0), 0.0, epsilon_d);

    EXPECT_NEAR(Math<float>::tan(0.0f), 0.0f, epsilon_f);
    EXPECT_NEAR(Math<float>::tan(PI_F / 4.0f), 1.0f, epsilon_f);
    EXPECT_NEAR(Math<float>::tan(-PI_F / 4.0f), -1.0f, epsilon_f);

    EXPECT_NEAR(Math<double>::tan(0.0), 0.0, epsilon_d);
    EXPECT_NEAR(Math<double>::tan(PI_D / 4.0), 1.0, epsilon_d);
    EXPECT_NEAR(Math<double>::tan(-PI_D / 4.0), -1.0, epsilon_d);
}

TEST_F(MathTest, HyperbolicFunctions)
{
    EXPECT_NEAR(Math<float>::sinh(0.0f), 0.0f, epsilon_f);
    EXPECT_NEAR(Math<float>::sinh(1.0f), 1.1752011936f, 0.001f);
    EXPECT_NEAR(Math<float>::sinh(-1.0f), -1.1752011936f, 0.001f);

    EXPECT_NEAR(Math<double>::sinh(0.0), 0.0, epsilon_d);
    EXPECT_NEAR(Math<double>::sinh(1.0), 1.1752011936438014, 0.001);
    EXPECT_NEAR(Math<double>::sinh(-1.0), -1.1752011936438014, 0.001);

    EXPECT_NEAR(Math<float>::cosh(0.0f), 1.0f, epsilon_f);
    EXPECT_NEAR(Math<float>::cosh(1.0f), 1.5430806348f, 0.001f);
    EXPECT_NEAR(Math<float>::cosh(-1.0f), 1.5430806348f, 0.001f);

    EXPECT_NEAR(Math<double>::cosh(0.0), 1.0, epsilon_d);
    EXPECT_NEAR(Math<double>::cosh(1.0), 1.5430806348152437, 0.001);
    EXPECT_NEAR(Math<double>::cosh(-1.0), 1.5430806348152437, 0.001);

    EXPECT_NEAR(Math<float>::tanh(0.0f), 0.0f, epsilon_f);
    EXPECT_NEAR(Math<float>::tanh(1.0f), 0.7615941559f, 0.001f);
    EXPECT_NEAR(Math<float>::tanh(-1.0f), -0.7615941559f, 0.001f);

    EXPECT_NEAR(Math<double>::tanh(0.0), 0.0, epsilon_d);
    EXPECT_NEAR(Math<double>::tanh(1.0), 0.7615941559557649, 0.001);
    EXPECT_NEAR(Math<double>::tanh(-1.0), -0.7615941559557649, 0.001);
}

TEST_F(MathTest, InverseTrigonometricFunctions)
{
    constexpr float PI_F = 3.14159265358979323846f;
    constexpr double PI_D = 3.14159265358979323846;

    EXPECT_NEAR(Math<float>::atan(0.0f), 0.0f, epsilon_f);
    EXPECT_NEAR(Math<float>::atan(1.0f), PI_F / 4.0f, 0.001f);
    EXPECT_NEAR(Math<float>::atan(-1.0f), -PI_F / 4.0f, 0.001f);

    EXPECT_NEAR(Math<double>::atan(0.0), 0.0, epsilon_d);
    EXPECT_NEAR(Math<double>::atan(1.0), PI_D / 4.0, 0.001);
    EXPECT_NEAR(Math<double>::atan(-1.0), -PI_D / 4.0, 0.001);

    EXPECT_NEAR(Math<float>::atan2(1.0f, 1.0f), PI_F / 4.0f, 0.001f);
    EXPECT_NEAR(Math<float>::atan2(1.0f, 0.0f), PI_F / 2.0f, 0.001f);
    EXPECT_NEAR(Math<float>::atan2(0.0f, 1.0f), 0.0f, epsilon_f);
    EXPECT_NEAR(Math<float>::atan2(-1.0f, 0.0f), -PI_F / 2.0f, 0.001f);

    EXPECT_NEAR(Math<double>::atan2(1.0, 1.0), PI_D / 4.0, 0.001);
    EXPECT_NEAR(Math<double>::atan2(1.0, 0.0), PI_D / 2.0, 0.001);
    EXPECT_NEAR(Math<double>::atan2(0.0, 1.0), 0.0, epsilon_d);
    EXPECT_NEAR(Math<double>::atan2(-1.0, 0.0), -PI_D / 2.0, 0.001);
}

TEST_F(MathTest, AngleConversions)
{
    constexpr float PI_F = 3.14159265358979323846f;
    constexpr double PI_D = 3.14159265358979323846;

    EXPECT_NEAR(Math<float>::toRadians(0.0f), 0.0f, epsilon_f);
    EXPECT_NEAR(Math<float>::toRadians(90.0f), PI_F / 2.0f, 0.001f);
    EXPECT_NEAR(Math<float>::toRadians(180.0f), PI_F, 0.001f);
    EXPECT_NEAR(Math<float>::toRadians(360.0f), 2.0f * PI_F, 0.001f);

    EXPECT_NEAR(Math<double>::toRadians(0.0), 0.0, epsilon_d);
    EXPECT_NEAR(Math<double>::toRadians(90.0), PI_D / 2.0, 0.001);
    EXPECT_NEAR(Math<double>::toRadians(180.0), PI_D, 0.001);
    EXPECT_NEAR(Math<double>::toRadians(360.0), 2.0 * PI_D, 0.001);

    EXPECT_NEAR(Math<float>::toDegrees(0.0f), 0.0f, epsilon_f);
    EXPECT_NEAR(Math<float>::toDegrees(PI_F / 2.0f), 90.0f, 0.001f);
    EXPECT_NEAR(Math<float>::toDegrees(PI_F), 180.0f, 0.001f);
    EXPECT_NEAR(Math<float>::toDegrees(2.0f * PI_F), 360.0f, 0.001f);

    EXPECT_NEAR(Math<double>::toDegrees(0.0), 0.0, epsilon_d);
    EXPECT_NEAR(Math<double>::toDegrees(PI_D / 2.0), 90.0, 0.001);
    EXPECT_NEAR(Math<double>::toDegrees(PI_D), 180.0, 0.001);
    EXPECT_NEAR(Math<double>::toDegrees(2.0 * PI_D), 360.0, 0.001);
}
