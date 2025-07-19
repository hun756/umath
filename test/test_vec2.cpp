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

TEST_F(Vector2Test, AdditionAssignment)
{
    Vector2f vec = v1;
    vec += v2;
    EXPECT_FLOAT_EQ(vec.x, 4.0f);
    EXPECT_FLOAT_EQ(vec.y, 6.0f);
}

TEST_F(Vector2Test, SubtractionAssignment)
{
    Vector2f vec = v1;
    vec -= v2;
    EXPECT_FLOAT_EQ(vec.x, 2.0f);
    EXPECT_FLOAT_EQ(vec.y, 2.0f);
}

TEST_F(Vector2Test, MultiplicationAssignment)
{
    Vector2f vec = v1;
    vec *= 2.0f;
    EXPECT_FLOAT_EQ(vec.x, 6.0f);
    EXPECT_FLOAT_EQ(vec.y, 8.0f);
}

TEST_F(Vector2Test, DivisionAssignment)
{
    Vector2f vec = v1;
    vec /= 2.0f;
    EXPECT_FLOAT_EQ(vec.x, 1.5f);
    EXPECT_FLOAT_EQ(vec.y, 2.0f);
}

TEST_F(Vector2Test, DivisionAssignmentByZero)
{
    Vector2f vec = v1;
    EXPECT_THROW(vec /= 0.0f, std::runtime_error);
}

TEST_F(Vector2Test, StaticAdd)
{
    auto result = Vector2f::add(v1, v2);
    EXPECT_FLOAT_EQ(result.x, 4.0f);
    EXPECT_FLOAT_EQ(result.y, 6.0f);
}

TEST_F(Vector2Test, StaticAddScalar)
{
    auto result = Vector2f::add_scalar(v1, 2.0f);
    EXPECT_FLOAT_EQ(result.x, 5.0f);
    EXPECT_FLOAT_EQ(result.y, 6.0f);
}

TEST_F(Vector2Test, StaticSubtract)
{
    auto result = Vector2f::subtract(v1, v2);
    EXPECT_FLOAT_EQ(result.x, 2.0f);
    EXPECT_FLOAT_EQ(result.y, 2.0f);
}

TEST_F(Vector2Test, StaticNegate)
{
    auto result = Vector2f::negate(v1);
    EXPECT_FLOAT_EQ(result.x, -3.0f);
    EXPECT_FLOAT_EQ(result.y, -4.0f);
}

TEST_F(Vector2Test, StaticInverse)
{
    Vector2f vec(2.0f, 4.0f);
    auto result = Vector2f::inverse(vec);
    EXPECT_FLOAT_EQ(result.x, 0.5f);
    EXPECT_FLOAT_EQ(result.y, 0.25f);
}

TEST_F(Vector2Test, StaticInverseZero)
{
    Vector2f vec(0.0f, 4.0f);
    EXPECT_THROW(Vector2f::inverse(vec), std::runtime_error);
}

TEST_F(Vector2Test, StaticInverseSafe)
{
    Vector2f vec(0.0f, 4.0f);
    auto result = Vector2f::inverse_safe(vec, 1.0f);
    EXPECT_FLOAT_EQ(result.x, 1.0f);
    EXPECT_FLOAT_EQ(result.y, 0.25f);
}

TEST_F(Vector2Test, LengthSquared)
{
    EXPECT_FLOAT_EQ(v1.length_squared(), 25.0f);
}

TEST_F(Vector2Test, Length)
{
    EXPECT_FLOAT_EQ(v1.length(), 5.0f);
}

TEST_F(Vector2Test, FastLength)
{
    EXPECT_NEAR(v1.fast_length(), 5.0f, 0.5f);
}

TEST_F(Vector2Test, DotProduct)
{
    float dot = Vector2f::dot(v1, v2);
    EXPECT_FLOAT_EQ(dot, 11.0f);
}

TEST_F(Vector2Test, DotProductMemberFunction)
{
    float dot = v1.dot(v2);
    EXPECT_FLOAT_EQ(dot, 11.0f);
}

TEST_F(Vector2Test, CrossProduct)
{
    float cross = Vector2f::cross(v1, v2);
    EXPECT_FLOAT_EQ(cross, 2.0f);
}

TEST_F(Vector2Test, CrossProductMemberFunction)
{
    float cross = v1.cross(v2);
    EXPECT_FLOAT_EQ(cross, 2.0f);
}

TEST_F(Vector2Test, Normalization)
{
    auto normalized = Vector2f::normalize(v1);
    EXPECT_FLOAT_EQ(normalized.x, 0.6f);
    EXPECT_FLOAT_EQ(normalized.y, 0.8f);
    EXPECT_NEAR(normalized.length(), 1.0f, epsilon);
}

TEST_F(Vector2Test, NormalizationZeroVector)
{
    EXPECT_THROW(Vector2f::normalize(zero_vec), std::runtime_error);
}

TEST_F(Vector2Test, NormalizeMemberFunction)
{
    Vector2f vec = v1;
    vec.normalize();
    EXPECT_FLOAT_EQ(vec.x, 0.6f);
    EXPECT_FLOAT_EQ(vec.y, 0.8f);
    EXPECT_NEAR(vec.length(), 1.0f, epsilon);
}

TEST_F(Vector2Test, FastNormalization)
{
    auto normalized = Vector2f::normalize_fast(v1);
    EXPECT_NEAR(normalized.length(), 1.0f, 0.01f);
}

TEST_F(Vector2Test, IsNormalized)
{
    EXPECT_FALSE(v1.is_normalized());
    auto normalized = Vector2f::normalize(v1);
    EXPECT_TRUE(normalized.is_normalized());
}

TEST_F(Vector2Test, IsZero)
{
    EXPECT_TRUE(zero_vec.is_zero());
    EXPECT_FALSE(v1.is_zero());
}

TEST_F(Vector2Test, DistanceSquared)
{
    float dist_sq = Vector2f::distance_squared(v1, v2);
    EXPECT_FLOAT_EQ(dist_sq, 8.0f);  // (3-1)² + (4-2)² = 4 + 4 = 8
}

TEST_F(Vector2Test, Distance)
{
    float dist = Vector2f::distance(v1, v2);
    EXPECT_FLOAT_EQ(dist, std::sqrt(8.0f));
}

TEST_F(Vector2Test, DistanceMemberFunction)
{
    float dist = v1.distance(v2);
    EXPECT_FLOAT_EQ(dist, std::sqrt(8.0f));
}

TEST_F(Vector2Test, FastDistance)
{
    float fast_dist = Vector2f::distance_fast(v1, v2);
    EXPECT_NEAR(fast_dist, std::sqrt(8.0f), 0.5f);
}

TEST_F(Vector2Test, ManhattanDistance)
{
    float manhattan = Vector2f::manhattan_distance(v1, v2);
    EXPECT_FLOAT_EQ(manhattan, 4.0f);
}

TEST_F(Vector2Test, ChebyshevDistance)
{
    float chebyshev = Vector2f::chebyshev_distance(v1, v2);
    EXPECT_FLOAT_EQ(chebyshev, 2.0f);
}

TEST_F(Vector2Test, AngleBetween)
{
    Vector2f v_x(1.0f, 0.0f);
    Vector2f v_y(0.0f, 1.0f);
    float angle = Vector2f::angle_between(v_x, v_y);
    EXPECT_NEAR(angle, PI / 2.0f, epsilon);
}

TEST_F(Vector2Test, AngleBetweenZeroVector)
{
    EXPECT_THROW(Vector2f::angle_between(zero_vec, v1), std::runtime_error);
}

TEST_F(Vector2Test, VectorAngle)
{
    Vector2f v_45deg(1.0f, 1.0f);
    float angle = v_45deg.angle();
    EXPECT_NEAR(angle, PI / 4.0f, epsilon);
}

TEST_F(Vector2Test, FastAngle)
{
    Vector2f v_from(0.0f, 0.0f);
    Vector2f v_to(1.0f, 1.0f);
    float angle = Vector2f::fast_angle(v_from, v_to);
    EXPECT_NEAR(angle, PI / 4.0f, 0.1f);
}

TEST_F(Vector2Test, Rotation)
{
    Vector2f vec(1.0f, 0.0f);
    auto rotated = Vector2f::rotate(vec, PI / 2.0f);
    EXPECT_NEAR(rotated.x, 0.0f, epsilon);
    EXPECT_NEAR(rotated.y, 1.0f, epsilon);
}

TEST_F(Vector2Test, FastRotation)
{
    Vector2f vec(1.0f, 0.0f);
    auto rotated = Vector2f::rotate_fast(vec, PI / 2.0f);
    EXPECT_NEAR(rotated.x, 0.0f, 0.01f);
    EXPECT_NEAR(rotated.y, 1.0f, 0.01f);
}

TEST_F(Vector2Test, RotationAroundPivot)
{
    Vector2f vec(2.0f, 0.0f);
    Vector2f pivot(1.0f, 0.0f);
    auto rotated = Vector2f::rotate_around(vec, PI / 2.0f, pivot);
    EXPECT_NEAR(rotated.x, 1.0f, epsilon);
    EXPECT_NEAR(rotated.y, 1.0f, epsilon);
}

TEST_F(Vector2Test, Perpendicular)
{
    Vector2f vec(1.0f, 0.0f);
    auto perp = Vector2f::perpendicular(vec);
    EXPECT_FLOAT_EQ(perp.x, 0.0f);
    EXPECT_FLOAT_EQ(perp.y, 1.0f);
}

TEST_F(Vector2Test, PerpendicularCCW)
{
    Vector2f vec(1.0f, 0.0f);
    auto perp = Vector2f::perpendicular_ccw(vec);
    EXPECT_FLOAT_EQ(perp.x, 0.0f);
    EXPECT_FLOAT_EQ(perp.y, -1.0f);
}

TEST_F(Vector2Test, LinearInterpolation)
{
    Vector2f start(0.0f, 0.0f);
    Vector2f end(10.0f, 10.0f);
    auto result = Vector2f::lerp(start, end, 0.5f);
    EXPECT_FLOAT_EQ(result.x, 5.0f);
    EXPECT_FLOAT_EQ(result.y, 5.0f);
}

TEST_F(Vector2Test, LinearInterpolationClamping)
{
    Vector2f start(0.0f, 0.0f);
    Vector2f end(10.0f, 10.0f);
    auto result = Vector2f::lerp(start, end, 1.5f);
    EXPECT_FLOAT_EQ(result.x, 10.0f);
    EXPECT_FLOAT_EQ(result.y, 10.0f);
}

TEST_F(Vector2Test, LinearInterpolationUnclamped)
{
    Vector2f start(0.0f, 0.0f);
    Vector2f end(10.0f, 10.0f);
    auto result = Vector2f::lerp_unclamped(start, end, 1.5f);
    EXPECT_FLOAT_EQ(result.x, 15.0f);
    EXPECT_FLOAT_EQ(result.y, 15.0f);
}

TEST_F(Vector2Test, SphericalLinearInterpolation)
{
    Vector2f start(1.0f, 0.0f);
    Vector2f end(0.0f, 1.0f);
    auto result = Vector2f::slerp(start, end, 0.5f);
    float expected_length = (start.length() + end.length()) * 0.5f;
    EXPECT_NEAR(result.length(), expected_length, epsilon);
}

TEST_F(Vector2Test, SmoothStep)
{
    Vector2f start(0.0f, 0.0f);
    Vector2f end(10.0f, 10.0f);
    auto result = Vector2f::smooth_step(start, end, 0.5f);
    EXPECT_FLOAT_EQ(result.x, 5.0f);
    EXPECT_FLOAT_EQ(result.y, 5.0f);
}

TEST_F(Vector2Test, SmootherStep)
{
    Vector2f start(0.0f, 0.0f);
    Vector2f end(10.0f, 10.0f);
    auto result = Vector2f::smoother_step(start, end, 0.5f);
    EXPECT_FLOAT_EQ(result.x, 5.0f);
    EXPECT_FLOAT_EQ(result.y, 5.0f);
}
