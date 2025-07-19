#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <umath/vec2.hpp>

#include <cmath>
#include <limits>
#include <unordered_set>

using namespace umath;
using ::testing::_;
using ::testing::DoubleEq;
using ::testing::DoubleNear;
using ::testing::FloatEq;
using ::testing::FloatNear;

class Vector2Test : public ::testing::Test
{
protected:
    void SetUp() override
    {
        zero_vec = Vector2f::zero();
        one_vec = Vector2f::one();
        unit_x = Vector2f::unit_x();
        unit_y = Vector2f::unit_y();

        v1 = Vector2f(3.0f, 4.0f);
        v2 = Vector2f(1.0f, 2.0f);
        v3 = Vector2f(-2.0f, 1.0f);

        epsilon = std::numeric_limits<float>::epsilon() * 100.0f;
    }

    Vector2f zero_vec, one_vec, unit_x, unit_y;
    Vector2f v1, v2, v3;
    float epsilon;
};

TEST_F(Vector2Test, DefaultConstructor)
{
    Vector2f vec;
    EXPECT_FLOAT_EQ(vec.x, 0.0f);
    EXPECT_FLOAT_EQ(vec.y, 0.0f);
}

TEST_F(Vector2Test, SingleValueConstructor)
{
    Vector2f vec(5.0f);
    EXPECT_FLOAT_EQ(vec.x, 5.0f);
    EXPECT_FLOAT_EQ(vec.y, 5.0f);
}

TEST_F(Vector2Test, TwoValueConstructor)
{
    Vector2f vec(3.0f, 4.0f);
    EXPECT_FLOAT_EQ(vec.x, 3.0f);
    EXPECT_FLOAT_EQ(vec.y, 4.0f);
}

TEST_F(Vector2Test, CopyConstructor)
{
    Vector2f original(3.0f, 4.0f);
    Vector2f copy(original);
    EXPECT_FLOAT_EQ(copy.x, 3.0f);
    EXPECT_FLOAT_EQ(copy.y, 4.0f);
}

TEST_F(Vector2Test, TypeConversionConstructor)
{
    Vector2i int_vec(3, 4);
    Vector2f float_vec(int_vec);
    EXPECT_FLOAT_EQ(float_vec.x, 3.0f);
    EXPECT_FLOAT_EQ(float_vec.y, 4.0f);
}

TEST_F(Vector2Test, StaticConstants)
{
    EXPECT_EQ(Vector2f::zero(), Vector2f(0.0f, 0.0f));
    EXPECT_EQ(Vector2f::one(), Vector2f(1.0f, 1.0f));
    EXPECT_EQ(Vector2f::unit_x(), Vector2f(1.0f, 0.0f));
    EXPECT_EQ(Vector2f::unit_y(), Vector2f(0.0f, 1.0f));
    EXPECT_EQ(Vector2f::up(), Vector2f(0.0f, 1.0f));
    EXPECT_EQ(Vector2f::down(), Vector2f(0.0f, -1.0f));
    EXPECT_EQ(Vector2f::left(), Vector2f(-1.0f, 0.0f));
    EXPECT_EQ(Vector2f::right(), Vector2f(1.0f, 0.0f));
}

TEST_F(Vector2Test, CreateMethod)
{
    auto vec = Vector2f::create(2.0f, 3.0f);
    EXPECT_FLOAT_EQ(vec.x, 2.0f);
    EXPECT_FLOAT_EQ(vec.y, 3.0f);
}

TEST_F(Vector2Test, FromArrayMethod)
{
    std::array<float, 4> arr = {1.0f, 2.0f, 3.0f, 4.0f};
    auto vec = Vector2f::from_array(arr, 1);
    EXPECT_FLOAT_EQ(vec.x, 2.0f);
    EXPECT_FLOAT_EQ(vec.y, 3.0f);
}

TEST_F(Vector2Test, VectorAddition)
{
    auto result = v1 + v2;
    EXPECT_FLOAT_EQ(result.x, 4.0f);
    EXPECT_FLOAT_EQ(result.y, 6.0f);
}

TEST_F(Vector2Test, VectorSubtraction)
{
    auto result = v1 - v2;
    EXPECT_FLOAT_EQ(result.x, 2.0f);
    EXPECT_FLOAT_EQ(result.y, 2.0f);
}

TEST_F(Vector2Test, ScalarMultiplication)
{
    auto result = v1 * 2.0f;
    EXPECT_FLOAT_EQ(result.x, 6.0f);
    EXPECT_FLOAT_EQ(result.y, 8.0f);

    auto result2 = 2.0f * v1;
    EXPECT_FLOAT_EQ(result2.x, 6.0f);
    EXPECT_FLOAT_EQ(result2.y, 8.0f);
}

TEST_F(Vector2Test, ComponentWiseMultiplication)
{
    auto result = v1 * v2;
    EXPECT_FLOAT_EQ(result.x, 3.0f);
    EXPECT_FLOAT_EQ(result.y, 8.0f);
}

TEST_F(Vector2Test, ScalarDivision)
{
    auto result = v1 / 2.0f;
    EXPECT_FLOAT_EQ(result.x, 1.5f);
    EXPECT_FLOAT_EQ(result.y, 2.0f);
}

TEST_F(Vector2Test, ScalarDivisionByZero)
{
    EXPECT_THROW(v1 / 0.0f, std::runtime_error);
}

TEST_F(Vector2Test, ComponentWiseDivision)
{
    Vector2f divisor(2.0f, 4.0f);
    auto result = v1 / divisor;
    EXPECT_FLOAT_EQ(result.x, 1.5f);
    EXPECT_FLOAT_EQ(result.y, 1.0f);
}

TEST_F(Vector2Test, ComponentWiseDivisionByZero)
{
    Vector2f divisor(0.0f, 4.0f);
    EXPECT_THROW(v1 / divisor, std::runtime_error);
}

TEST_F(Vector2Test, UnaryMinus)
{
    auto result = -v1;
    EXPECT_FLOAT_EQ(result.x, -3.0f);
    EXPECT_FLOAT_EQ(result.y, -4.0f);
}

TEST_F(Vector2Test, UnaryPlus)
{
    auto result = +v1;
    EXPECT_FLOAT_EQ(result.x, 3.0f);
    EXPECT_FLOAT_EQ(result.y, 4.0f);
}
