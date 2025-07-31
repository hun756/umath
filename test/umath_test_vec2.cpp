#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <umath/vec2.hpp>

#include <cmath>
#include <limits>
#include <unordered_set>

using namespace umath;
using ::testing::AtLeast;
using ::testing::InSequence;
using ::testing::Return;

constexpr float PI_F = static_cast<float>(PI);

class IMathCalculator
{
public:
    virtual ~IMathCalculator() = default;
    virtual float calculateDistance(const Vector2f& a, const Vector2f& b) = 0;
    virtual float calculateAngle(const Vector2f& v) = 0;
    virtual Vector2f normalize(const Vector2f& v) = 0;
};

class MockMathCalculator : public IMathCalculator
{
public:
    MOCK_METHOD(float, calculateDistance, (const Vector2f& a, const Vector2f& b), (override));
    MOCK_METHOD(float, calculateAngle, (const Vector2f& v), (override));
    MOCK_METHOD(Vector2f, normalize, (const Vector2f& v), (override));
};

class VectorProcessor
{
private:
    std::unique_ptr<IMathCalculator> calculator_;

public:
    explicit VectorProcessor(std::unique_ptr<IMathCalculator> calc) : calculator_(std::move(calc))
    {
    }

    float processDistance(const Vector2f& a, const Vector2f& b)
    {
        return calculator_->calculateDistance(a, b);
    }

    Vector2f processNormalization(const Vector2f& v)
    {
        return calculator_->normalize(v);
    }

    float processAngle(const Vector2f& v)
    {
        return calculator_->calculateAngle(v);
    }
};

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
    EXPECT_THROW({ [[maybe_unused]] auto result = v1 / 0.0f; }, std::runtime_error);
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
    EXPECT_THROW({ [[maybe_unused]] auto result = v1 / divisor; }, std::runtime_error);
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
    EXPECT_NEAR(v1.approximate_length(), 5.0f, 0.5f);
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
    auto normalized = Vector2f::normalize_approximate(v1);
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
    float fast_dist = Vector2f::distance_approximate(v1, v2);
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
    EXPECT_NEAR(angle, PI_F / 2.0f, epsilon);
}

TEST_F(Vector2Test, AngleBetweenZeroVector)
{
    EXPECT_THROW(Vector2f::angle_between(zero_vec, v1), std::runtime_error);
}

TEST_F(Vector2Test, VectorAngle)
{
    Vector2f v_45deg(1.0f, 1.0f);
    float angle = v_45deg.angle();
    EXPECT_NEAR(angle, PI_F / 4.0f, epsilon);
}

TEST_F(Vector2Test, FastAngle)
{
    Vector2f v_from(0.0f, 0.0f);
    Vector2f v_to(1.0f, 1.0f);
    float angle = Vector2f::angle_approximate(v_from, v_to);
    EXPECT_NEAR(angle, PI_F / 4.0f, 0.1f);
}

TEST_F(Vector2Test, Rotation)
{
    Vector2f vec(1.0f, 0.0f);
    auto rotated = Vector2f::rotate(vec, PI_F / 2.0f);
    EXPECT_NEAR(rotated.x, 0.0f, epsilon);
    EXPECT_NEAR(rotated.y, 1.0f, epsilon);
}

TEST_F(Vector2Test, FastRotation)
{
    Vector2f vec(1.0f, 0.0f);
    auto rotated = Vector2f::rotate_optimized(vec, PI_F / 2.0f);
    EXPECT_NEAR(rotated.x, 0.0f, 0.01f);
    EXPECT_NEAR(rotated.y, 1.0f, 0.01f);
}

TEST_F(Vector2Test, RotationAroundPivot)
{
    Vector2f vec(2.0f, 0.0f);
    Vector2f pivot(1.0f, 0.0f);
    auto rotated = Vector2f::rotate_around(vec, PI_F / 2.0f, pivot);
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

TEST_F(Vector2Test, CubicBezier)
{
    Vector2f p0(0.0f, 0.0f);
    Vector2f p1(1.0f, 2.0f);
    Vector2f p2(2.0f, 2.0f);
    Vector2f p3(3.0f, 0.0f);

    auto start = Vector2f::cubic_bezier(p0, p1, p2, p3, 0.0f);
    EXPECT_FLOAT_EQ(start.x, 0.0f);
    EXPECT_FLOAT_EQ(start.y, 0.0f);

    auto end = Vector2f::cubic_bezier(p0, p1, p2, p3, 1.0f);
    EXPECT_FLOAT_EQ(end.x, 3.0f);
    EXPECT_FLOAT_EQ(end.y, 0.0f);
}

TEST_F(Vector2Test, HermiteInterpolation)
{
    Vector2f p0(0.0f, 0.0f);
    Vector2f m0(1.0f, 0.0f);
    Vector2f p1(1.0f, 1.0f);
    Vector2f m1(0.0f, 1.0f);

    auto start = Vector2f::hermite(p0, m0, p1, m1, 0.0f);
    EXPECT_FLOAT_EQ(start.x, 0.0f);
    EXPECT_FLOAT_EQ(start.y, 0.0f);

    auto end = Vector2f::hermite(p0, m0, p1, m1, 1.0f);
    EXPECT_FLOAT_EQ(end.x, 1.0f);
    EXPECT_FLOAT_EQ(end.y, 1.0f);
}

TEST_F(Vector2Test, CatmullRomSpline)
{
    Vector2f p0(0.0f, 0.0f);
    Vector2f p1(1.0f, 1.0f);
    Vector2f p2(2.0f, 1.0f);
    Vector2f p3(3.0f, 0.0f);

    auto start = Vector2f::catmull_rom(p0, p1, p2, p3, 0.0f);
    EXPECT_FLOAT_EQ(start.x, 1.0f);
    EXPECT_FLOAT_EQ(start.y, 1.0f);

    auto end = Vector2f::catmull_rom(p0, p1, p2, p3, 1.0f);
    EXPECT_FLOAT_EQ(end.x, 2.0f);
    EXPECT_FLOAT_EQ(end.y, 1.0f);
}


TEST_F(Vector2Test, RandomVector)
{
    auto random1 = Vector2f::random(1.0f);
    auto random2 = Vector2f::random(1.0f);

    // Random vectors should be different
    EXPECT_FALSE(random1.approximately_equals(random2, 0.001f));

    // Random vector magnitude should be reasonable (allow more tolerance)
    EXPECT_GT(random1.length(), 0.1f);
    EXPECT_LT(random1.length(), 2.0f);  // More lenient range
}

TEST_F(Vector2Test, RandomFastVector)
{
    auto random1 = Vector2f::random_quick(1.0f);
    auto random2 = Vector2f::random_quick(1.0f);

    EXPECT_FALSE(random1.approximately_equals(random2, 0.001f));
    EXPECT_NEAR(random1.length(), 1.0f, epsilon);
}

TEST_F(Vector2Test, RandomBox)
{
    auto random = Vector2f::random_box(-1.0f, 1.0f, -2.0f, 2.0f);

    EXPECT_GE(random.x, -1.0f);
    EXPECT_LE(random.x, 1.0f);
    EXPECT_GE(random.y, -2.0f);
    EXPECT_LE(random.y, 2.0f);
}

TEST_F(Vector2Test, EqualityOperator)
{
    Vector2f vec1(3.0f, 4.0f);
    Vector2f vec2(3.0f, 4.0f);
    Vector2f vec3(3.1f, 4.0f);

    EXPECT_TRUE(vec1 == vec2);
    EXPECT_FALSE(vec1 == vec3);
}

TEST_F(Vector2Test, InequalityOperator)
{
    Vector2f vec1(3.0f, 4.0f);
    Vector2f vec2(3.1f, 4.0f);

    EXPECT_TRUE(vec1 != vec2);
    EXPECT_FALSE(vec1 != vec1);
}

TEST_F(Vector2Test, ApproximatelyEquals)
{
    Vector2f vec1(3.0f, 4.0f);
    Vector2f vec2(3.0001f, 4.0001f);

    EXPECT_TRUE(vec1.approximately_equals(vec2, 0.001f));
    EXPECT_FALSE(vec1.approximately_equals(vec2, 0.00001f));
}

TEST_F(Vector2Test, ComparisonModes)
{
    Vector2f shorter(1.0f, 0.0f);
    Vector2f longer(2.0f, 0.0f);

    EXPECT_LT(Vector2f::compare(shorter, longer, ComparisonMode::LEXICOGRAPHIC), 0);

    EXPECT_LT(Vector2f::compare(shorter, longer, ComparisonMode::MAGNITUDE), 0);

    EXPECT_LT(Vector2f::compare(shorter, longer, ComparisonMode::MANHATTAN), 0);
}

TEST_F(Vector2Test, AbsoluteValue)
{
    Vector2f vec(-3.0f, -4.0f);
    auto abs_vec = vec.abs();
    EXPECT_FLOAT_EQ(abs_vec.x, 3.0f);
    EXPECT_FLOAT_EQ(abs_vec.y, 4.0f);
}

TEST_F(Vector2Test, Sign)
{
    Vector2f vec(-3.0f, 4.0f);
    auto sign_vec = vec.sign();
    EXPECT_FLOAT_EQ(sign_vec.x, -1.0f);
    EXPECT_FLOAT_EQ(sign_vec.y, 1.0f);
}

TEST_F(Vector2Test, MinMaxComponent)
{
    Vector2f vec(3.0f, 7.0f);
    EXPECT_FLOAT_EQ(vec.min_component(), 3.0f);
    EXPECT_FLOAT_EQ(vec.max_component(), 7.0f);
    EXPECT_EQ(vec.min_component_index(), 0);
    EXPECT_EQ(vec.max_component_index(), 1);
}

TEST_F(Vector2Test, SumAndProduct)
{
    Vector2f vec(3.0f, 4.0f);
    EXPECT_FLOAT_EQ(vec.sum(), 7.0f);
    EXPECT_FLOAT_EQ(vec.product(), 12.0f);
}

TEST_F(Vector2Test, ArrayAccess)
{
    Vector2f vec(3.0f, 4.0f);
    EXPECT_FLOAT_EQ(vec[0], 3.0f);
    EXPECT_FLOAT_EQ(vec[1], 4.0f);

    vec[0] = 5.0f;
    EXPECT_FLOAT_EQ(vec.x, 5.0f);
}

TEST_F(Vector2Test, DataAccess)
{
    Vector2f vec(3.0f, 4.0f);
    float* data = vec.data();
    EXPECT_FLOAT_EQ(data[0], 3.0f);
    EXPECT_FLOAT_EQ(data[1], 4.0f);
}

TEST_F(Vector2Test, ToArray)
{
    Vector2f vec(3.0f, 4.0f);
    auto arr = vec.to_array();
    EXPECT_FLOAT_EQ(arr[0], 3.0f);
    EXPECT_FLOAT_EQ(arr[1], 4.0f);
}

TEST_F(Vector2Test, Clone)
{
    Vector2f original(3.0f, 4.0f);
    auto cloned = original.clone();
    EXPECT_FLOAT_EQ(cloned.x, 3.0f);
    EXPECT_FLOAT_EQ(cloned.y, 4.0f);
}

TEST_F(Vector2Test, Swap)
{
    Vector2f vec1(3.0f, 4.0f);
    Vector2f vec2(1.0f, 2.0f);

    vec1.swap(vec2);

    EXPECT_FLOAT_EQ(vec1.x, 1.0f);
    EXPECT_FLOAT_EQ(vec1.y, 2.0f);
    EXPECT_FLOAT_EQ(vec2.x, 3.0f);
    EXPECT_FLOAT_EQ(vec2.y, 4.0f);
}

TEST_F(Vector2Test, HashFunction)
{
    Vector2f vec1(3.0f, 4.0f);
    Vector2f vec2(3.0f, 4.0f);
    Vector2f vec3(4.0f, 3.0f);

    EXPECT_EQ(vec1.hash(), vec2.hash());
    EXPECT_NE(vec1.hash(), vec3.hash());
}

TEST_F(Vector2Test, StdHashSpecialization)
{
    Vector2f vec1(3.0f, 4.0f);
    Vector2f vec2(3.0f, 4.0f);

    std::hash<Vector2f> hasher;
    EXPECT_EQ(hasher(vec1), hasher(vec2));

    std::unordered_set<Vector2f> vec_set;
    vec_set.insert(vec1);
    EXPECT_EQ(vec_set.size(), 1);
    vec_set.insert(vec2);
    EXPECT_EQ(vec_set.size(), 1);
}

TEST_F(Vector2Test, ImmutableVector2Creation)
{
    auto immutable = Vector2f::make_immutable(3.0f, 4.0f);
    EXPECT_FLOAT_EQ(immutable.x(), 3.0f);
    EXPECT_FLOAT_EQ(immutable.y(), 4.0f);
}

TEST_F(Vector2Test, ImmutableVector2FromVector2)
{
    Vector2f vec(3.0f, 4.0f);
    auto immutable = Vector2f::make_immutable(vec);
    EXPECT_FLOAT_EQ(immutable.x(), 3.0f);
    EXPECT_FLOAT_EQ(immutable.y(), 4.0f);
}

TEST_F(Vector2Test, ImmutableVector2Conversion)
{
    auto immutable = Vector2f::make_immutable(3.0f, 4.0f);
    Vector2f vec = immutable;
    EXPECT_FLOAT_EQ(vec.x, 3.0f);
    EXPECT_FLOAT_EQ(vec.y, 4.0f);
}

TEST_F(Vector2Test, ImmutableVector2TypeConversion)
{
    auto immutable_float = Vector2f::make_immutable(3.0f, 4.0f);
    auto immutable_double = immutable_float.template as<double>();
    EXPECT_DOUBLE_EQ(immutable_double.x(), 3.0);
    EXPECT_DOUBLE_EQ(immutable_double.y(), 4.0);
}

TEST_F(Vector2Test, ImmutableVector2StaticConstants)
{
    auto zero = Vector2f::zero_immutable();
    EXPECT_FLOAT_EQ(zero.x(), 0.0f);
    EXPECT_FLOAT_EQ(zero.y(), 0.0f);

    auto one = Vector2f::one_immutable();
    EXPECT_FLOAT_EQ(one.x(), 1.0f);
    EXPECT_FLOAT_EQ(one.y(), 1.0f);
}

TEST_F(Vector2Test, TypeAliases)
{
    Vec2f vec_f(1.0f, 2.0f);
    Vec2d vec_d(1.0, 2.0);
    Vec2i vec_i(1, 2);

    EXPECT_FLOAT_EQ(vec_f.x, 1.0f);
    EXPECT_DOUBLE_EQ(vec_d.x, 1.0);
    EXPECT_EQ(vec_i.x, 1);

    Vector2i8 vec_i8(1, 2);
    Vector2u16 vec_u16(1, 2);
    Vector2i32 vec_i32(1, 2);
    Vector2u64 vec_u64(1, 2);

    EXPECT_EQ(vec_i8.x, 1);
    EXPECT_EQ(vec_u16.x, 1);
    EXPECT_EQ(vec_i32.x, 1);
    EXPECT_EQ(vec_u64.x, 1);
}

// Todo: Uncomment when structured bindings are supported
// TEST_F(Vector2Test, StructuredBinding)
// {
//     Vector2f vec(3.0f, 4.0f);
//     auto [x, y] = vec;
//     EXPECT_FLOAT_EQ(x, 3.0f);
//     EXPECT_FLOAT_EQ(y, 4.0f);
// }

TEST_F(Vector2Test, StdGet)
{
    Vector2f vec(3.0f, 4.0f);
    EXPECT_FLOAT_EQ(std::get<0>(vec), 3.0f);
    EXPECT_FLOAT_EQ(std::get<1>(vec), 4.0f);
}

TEST_F(Vector2Test, TupleSize)
{
    EXPECT_EQ(std::tuple_size_v<Vector2f>, 2);
}

TEST_F(Vector2Test, InfinityHandling)
{
    if constexpr (std::is_floating_point_v<float>)
    {
        Vector2f inf_vec(std::numeric_limits<float>::infinity(),
                         std::numeric_limits<float>::infinity());
        EXPECT_TRUE(std::isinf(inf_vec.x));
        EXPECT_TRUE(std::isinf(inf_vec.y));
    }
}

TEST_F(Vector2Test, NaNHandling)
{
    if constexpr (std::is_floating_point_v<float>)
    {
        Vector2f nan_vec(std::numeric_limits<float>::quiet_NaN(), 0.0f);
        EXPECT_TRUE(std::isnan(nan_vec.x));
        EXPECT_FALSE(std::isnan(nan_vec.y));
    }
}

// Todo fix it!
TEST_F(Vector2Test, VerySmallNumbers)
{
    Vector2f tiny_vec(std::numeric_limits<float>::min() * 1000.0f,
                      std::numeric_limits<float>::min() * 1000.0f);
    EXPECT_GT(tiny_vec.length(), 0.0f);

    Vector2f small_vec(1e-20f, 1e-20f);
    EXPECT_GT(small_vec.length(), 0.0f);
}

TEST_F(Vector2Test, VeryLargeNumbers)
{
    float large_val = std::sqrt(std::numeric_limits<float>::max()) * 0.5f;
    Vector2f large_vec(large_val, large_val);

    EXPECT_TRUE(std::isfinite(large_vec.length()));
    EXPECT_FALSE(std::isinf(large_vec.length()));

    Vector2f edge_vec(std::numeric_limits<float>::max() * 0.1f,
                      std::numeric_limits<float>::max() * 0.1f);
    [[maybe_unused]] auto length = edge_vec.length();
}

class MockTests : public ::testing::Test
{
protected:
    void SetUp() override
    {
        mock_calculator = std::make_unique<MockMathCalculator>();
        mock_ptr = mock_calculator.get();
        processor = std::make_unique<VectorProcessor>(std::move(mock_calculator));

        vec_a = Vector2f(3.0f, 4.0f);
        vec_b = Vector2f(1.0f, 2.0f);
        zero_vec = Vector2f(0.0f, 0.0f);
    }

    void TearDown() override
    {
        processor.reset();
    }

    std::unique_ptr<MockMathCalculator> mock_calculator;
    MockMathCalculator* mock_ptr;
    std::unique_ptr<VectorProcessor> processor;
    Vector2f vec_a, vec_b, zero_vec;
};

TEST_F(MockTests, MockCalculatorDistance)
{
    EXPECT_CALL(*mock_ptr, calculateDistance(vec_a, vec_b)).Times(1).WillOnce(Return(5.0f));

    float result = processor->processDistance(vec_a, vec_b);

    EXPECT_FLOAT_EQ(result, 5.0f);
}

TEST_F(MockTests, MockCalculatorNormalization)
{
    Vector2f expected_result(0.6f, 0.8f);

    EXPECT_CALL(*mock_ptr, normalize(vec_a)).Times(1).WillOnce(Return(expected_result));

    Vector2f result = processor->processNormalization(vec_a);

    EXPECT_FLOAT_EQ(result.x, 0.6f);
    EXPECT_FLOAT_EQ(result.y, 0.8f);
}

TEST_F(MockTests, MockCalculatorAngle)
{
    EXPECT_CALL(*mock_ptr, calculateAngle(vec_a)).Times(AtLeast(1)).WillRepeatedly(Return(0.927f));

    float result1 = processor->processAngle(vec_a);
    float result2 = processor->processAngle(vec_a);

    EXPECT_FLOAT_EQ(result1, 0.927f);
    EXPECT_FLOAT_EQ(result2, 0.927f);
}

TEST_F(MockTests, MockCalculatorSequentialCalls)
{
    Vector2f norm1(0.6f, 0.8f);
    Vector2f norm2(0.707f, 0.707f);

    InSequence seq;
    EXPECT_CALL(*mock_ptr, normalize(vec_a)).WillOnce(Return(norm1));
    EXPECT_CALL(*mock_ptr, normalize(vec_b)).WillOnce(Return(norm2));

    Vector2f result1 = processor->processNormalization(vec_a);
    Vector2f result2 = processor->processNormalization(vec_b);

    EXPECT_FLOAT_EQ(result1.x, 0.6f);
    EXPECT_FLOAT_EQ(result1.y, 0.8f);
    EXPECT_FLOAT_EQ(result2.x, 0.707f);
    EXPECT_FLOAT_EQ(result2.y, 0.707f);
}

struct Vector2TestParam
{
    Vector2f input;
    float expected_length;
    std::string description;
};

class Vector2ParameterizedTest : public ::testing::TestWithParam<Vector2TestParam>
{
};

TEST_P(Vector2ParameterizedTest, LengthCalculation)
{
    const auto& param = GetParam();
    EXPECT_NEAR(param.input.length(), param.expected_length, 0.001f)
        << "Test case: " << param.description;
}

INSTANTIATE_TEST_SUITE_P(Vector2LengthTests,
                         Vector2ParameterizedTest,
                         ::testing::Values(
                             Vector2TestParam{
                                 {0.0f, 0.0f},
                                 0.0f,
                                 "Zero vector"
},
                             Vector2TestParam{{1.0f, 0.0f}, 1.0f, "Unit X vector"},
                             Vector2TestParam{{0.0f, 1.0f}, 1.0f, "Unit Y vector"},
                             Vector2TestParam{{3.0f, 4.0f}, 5.0f, "3-4-5 triangle"},
                             Vector2TestParam{{1.0f, 1.0f}, 1.414f, "45 degree vector"},
                             Vector2TestParam{{-3.0f, -4.0f}, 5.0f, "Negative 3-4-5 triangle"}));

class Vector2PropertyTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rng.seed(12'345);
    }

    std::mt19937 rng;

    Vector2f generateRandomVector(float min_val = -100.0f, float max_val = 100.0f)
    {
        std::uniform_real_distribution<float> dist(min_val, max_val);
        return Vector2f(dist(rng), dist(rng));
    }
};

TEST_F(Vector2PropertyTest, AdditionIsCommutative)
{
    for (int i = 0; i < 100; ++i)
    {
        Vector2f a = generateRandomVector();
        Vector2f b = generateRandomVector();

        Vector2f result1 = a + b;
        Vector2f result2 = b + a;

        EXPECT_TRUE(result1.approximately_equals(result2, 0.001f))
            << "Commutativity failed for a=(" << a.x << "," << a.y << ") b=(" << b.x << "," << b.y
            << ")";
    }
}

TEST_F(Vector2PropertyTest, ScalarMultiplicationIsAssociative)
{
    for (int i = 0; i < 100; ++i)
    {
        Vector2f v = generateRandomVector();
        std::uniform_real_distribution<float> scalar_dist(-10.0f, 10.0f);
        float a = scalar_dist(rng);
        float b = scalar_dist(rng);

        Vector2f result1 = (v * a) * b;
        Vector2f result2 = v * (a * b);

        EXPECT_TRUE(result1.approximately_equals(result2, 0.001f))
            << "Associativity failed for v=(" << v.x << "," << v.y << ") a=" << a << " b=" << b;
    }
}

TEST_F(Vector2PropertyTest, NormalizationPreservesDirection)
{
    for (int i = 0; i < 100; ++i)
    {
        Vector2f v = generateRandomVector(0.1f, 100.0f);

        if (v.length() > 0.001f)
        {
            Vector2f normalized = Vector2f::normalize(v);

            float dot = Vector2f::dot(v, normalized);
            EXPECT_GT(dot, 0.0f) << "Direction not preserved for v=(" << v.x << "," << v.y << ")";

            EXPECT_NEAR(normalized.length(), 1.0f, 0.001f)
                << "Normalized length incorrect for v=(" << v.x << "," << v.y << ")";
        }
    }
}


TEST_F(Vector2Test, FastOperationsConsistency)
{
    Vector2f vec(3.0f, 4.0f);

    float regular_length = vec.length();
    float fast_length = vec.approximate_length();
    EXPECT_NEAR(fast_length, regular_length, regular_length * 0.1f);

    auto regular_norm = Vector2f::normalize(vec);
    auto fast_norm = Vector2f::normalize_approximate(vec);
    EXPECT_NEAR(fast_norm.x, regular_norm.x, 0.01f);
    EXPECT_NEAR(fast_norm.y, regular_norm.y, 0.01f);

    float angle = PI_F / 6.0f;
    auto regular_rot = Vector2f::rotate(vec, angle);
    auto fast_rot = Vector2f::rotate_optimized(vec, angle);
    EXPECT_NEAR(fast_rot.x, regular_rot.x, 0.01f);
    EXPECT_NEAR(fast_rot.y, regular_rot.y, 0.01f);
}

TEST_F(Vector2Test, MemoryLayout)
{
    Vector2f vec(3.0f, 4.0f);
    float* ptr = reinterpret_cast<float*>(&vec);

    EXPECT_FLOAT_EQ(ptr[0], 3.0f);
    EXPECT_FLOAT_EQ(ptr[1], 4.0f);

    EXPECT_EQ(&vec.y, &vec.x + 1);
}

TEST_F(Vector2Test, Alignment)
{
    EXPECT_EQ(alignof(Vector2f), sizeof(float) * 2);
}

TEST_F(Vector2Test, StaticConstantsThreadSafety)
{
    const auto& zero1 = Vector2f::zero();
    const auto& zero2 = Vector2f::zero();

    EXPECT_EQ(&zero1, &zero2);
}

TEST_F(Vector2Test, ComplexOperationChaining)
{
    Vector2f start(1.0f, 0.0f);
    Vector2f end(0.0f, 1.0f);

    auto result = Vector2f::normalize(Vector2f::lerp(start, end, 0.5f) + Vector2f(0.1f, 0.1f));

    EXPECT_NEAR(result.length(), 1.0f, epsilon);
}

TEST_F(Vector2Test, GeometricOperations)
{
    Vector2f center(0.0f, 0.0f);
    Vector2f point(3.0f, 4.0f);

    auto rotated = Vector2f::rotate_around(point, PI_F / 2.0f, center);

    EXPECT_NEAR(center.distance(rotated), center.distance(point), epsilon);

    EXPECT_NEAR(Vector2f::dot(point, rotated), 0.0f, epsilon);
}
