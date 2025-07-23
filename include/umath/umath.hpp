#ifndef LIB_UMATH_HPP_p7q7hl
#define LIB_UMATH_HPP_p7q7hl

#include <simd/feature_check.hpp>

#include <bit>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <type_traits>

namespace umath
{
class arithmetic_overflow final : public std::runtime_error
{
public:
    explicit arithmetic_overflow(const char* message) noexcept : std::runtime_error(message) {}
};

template <typename T>
concept Arithmetic = std::is_arithmetic_v<T>;

template <typename T>
concept FloatingPoint = std::is_floating_point_v<T>;

template <typename T>
concept Integral = std::is_integral_v<T>;

template <typename T>
concept SignedIntegral = Integral<T> && std::is_signed_v<T>;

template <typename T>
struct FloatTraits
{
    static constexpr bool is_float = std::is_floating_point_v<T>;
    static constexpr T epsilon = std::numeric_limits<T>::epsilon();
    static constexpr T infinity = std::numeric_limits<T>::infinity();
    static constexpr T quiet_nan = std::numeric_limits<T>::quiet_NaN();
    static constexpr T signaling_nan = std::numeric_limits<T>::signaling_NaN();
    static constexpr int max_exponent = std::numeric_limits<T>::max_exponent;
    static constexpr int min_exponent = std::numeric_limits<T>::min_exponent;
};

namespace detail
{
template <typename T>
struct ArchSpecificOps
{
    [[nodiscard]] static inline T fast_sqrt(T x) noexcept
    {
        if constexpr (std::is_same_v<T, float> && simd::compile_time::has<simd::Feature::SSE>())
        {
            float result;
            __m128 in = _mm_set_ss(x);
            __m128 out = _mm_sqrt_ss(in);
            _mm_store_ss(&result, out);
            return result;
        }
        else if constexpr (std::is_same_v<T, double>
                           && simd::compile_time::has<simd::Feature::SSE2>())
        {
            double result;
            __m128d in = _mm_set_sd(x);
            __m128d out = _mm_sqrt_sd(in, in);
            _mm_store_sd(&result, out);
            return result;
        }
        else
        {
            return std::sqrt(x);
        }
    }

    [[nodiscard]] static inline T fast_rsqrt(T x) noexcept
    {
        if constexpr (std::is_same_v<T, float> && simd::compile_time::has<simd::Feature::SSE>())
        {
            float result;
            __m128 in = _mm_set_ss(x);
            __m128 out = _mm_rsqrt_ss(in);
            _mm_store_ss(&result, out);
            return result;
        }
        else
        {
            return T(1) / std::sqrt(x);
        }
    }
};

template <typename T>
struct OverflowChecker
{
    [[nodiscard]] static constexpr bool will_add_overflow(T x, T y) noexcept
    {
        if constexpr (std::is_integral_v<T>)
        {
            if (y > 0)
            {
                return x > std::numeric_limits<T>::max() - y;
            }
            return x < std::numeric_limits<T>::min() - y;
        }
        return false;
    }

    [[nodiscard]] static constexpr bool will_subtract_overflow(T x, T y) noexcept
    {
        if constexpr (std::is_integral_v<T>)
        {
            if (y > 0)
            {
                return x < std::numeric_limits<T>::min() + y;
            }
            return x > std::numeric_limits<T>::max() + y;
        }
        return false;
    }

    [[nodiscard]] static constexpr bool will_multiply_overflow(T x, T y) noexcept
    {
        if constexpr (std::is_integral_v<T>)
        {
            if (x == 0 || y == 0)
                return false;
            if (x > 0)
            {
                if (y > 0)
                {
                    return x > std::numeric_limits<T>::max() / y;
                }
                return y < std::numeric_limits<T>::min() / x;
            }
            if (y > 0)
            {
                return x < std::numeric_limits<T>::min() / y;
            }
            return x != -1 && y < std::numeric_limits<T>::max() / x;
        }
        return false;
    }
};

template <typename T>
struct FastOps
{
    [[nodiscard]] static constexpr T abs(T x) noexcept
    {
        if constexpr (std::is_unsigned_v<T>)
        {
            return x;
        }
        else
        {
            const T mask = x >> (std::numeric_limits<T>::digits - 1);
            return (x + mask) ^ mask;
        }
    }

    [[nodiscard]] static constexpr T signum(T x) noexcept
    {
        return (T(0) < x) - (x < T(0));
    }

    template <typename U = T>
    [[nodiscard]] static constexpr std::enable_if_t<std::is_integral_v<U>, U> fast_mod(U x,
                                                                                       U y) noexcept
    {
        if constexpr (std::has_single_bit(y))
        {
            return x & (y - 1);
        }
        else
        {
            return x % y;
        }
    }
};

}  // namespace detail

template <typename T>
requires Arithmetic<T>
class Math final
{
private:
    static constexpr T PI = std::numbers::pi_v<T>;
    static constexpr T E = std::numbers::e_v<T>;

    using Checker = detail::OverflowChecker<T>;
    using FastOp = detail::FastOps<T>;
    using ArchOp = detail::ArchSpecificOps<T>;

public:
    Math() = delete;
    ~Math() = delete;
    Math(const Math&) = delete;
    Math& operator=(const Math&) = delete;
    Math(Math&&) = delete;
    Math& operator=(Math&&) = delete;

    static constexpr T RAD_TO_DEG = T(180) / PI;
    static constexpr T DEG_TO_RAD = PI / T(180);

    [[nodiscard]] static constexpr T addExact(T x, T y)
    {
        if (Checker::will_add_overflow(x, y))
        {
            throw arithmetic_overflow("Addition overflow");
        }
        return x + y;
    }

    [[nodiscard]] static constexpr T subtractExact(T x, T y)
    {
        if (Checker::will_subtract_overflow(x, y))
        {
            throw arithmetic_overflow("Subtraction overflow");
        }
        return x - y;
    }

    [[nodiscard]] static constexpr T multiplyExact(T x, T y)
    {
        if (Checker::will_multiply_overflow(x, y))
        {
            throw arithmetic_overflow("Multiplication overflow");
        }
        return x * y;
    }

    [[nodiscard]] static constexpr T abs(T x) noexcept
    {
        return FastOp::abs(x);
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U cos(U x) noexcept
    {
        if constexpr (simd::compile_time::has<simd::Feature::AVX>())
        {
            if (Math::abs(x) < U(0.01))
            {
                U x2 = x * x;
                return U(1) - x2 * (U(0.5) - x2 * (U(1) / U(24)));
            }
        }
        return std::cos(x);
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U sin(U x) noexcept
    {
        if constexpr (simd::compile_time::has<simd::Feature::AVX>())
        {
            if (Math::abs(x) < U(0.01))
            {
                U x2 = x * x;
                U x3 = x2 * x;
                U x5 = x3 * x2;
                return x - x3 / U(6) + x5 / U(120);
            }
        }
        return std::sin(x);
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U ceil(U x) noexcept
    {
        if constexpr (std::is_same_v<U, float> && simd::compile_time::has<simd::Feature::SSE41>)
        {
            float result;
            __m128 in = _mm_set_ss(x);
            __m128 rounded = _mm_ceil_ss(in, in);
            _mm_store_ss(&result, rounded);
            return result;
        }
        else if constexpr (std::is_same_v<U, double>
                           && simd::compile_time::has<simd::Feature::SSE41>)
        {
            double result;
            __m128d in = _mm_set_sd(x);
            __m128d rounded = _mm_ceil_sd(in, in);
            _mm_store_sd(&result, rounded);
            return result;
        }
        return std::ceil(x);
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U floor(U x) noexcept
    {
        if constexpr (std::is_same_v<U, float> && simd::compile_time::has<simd::Feature::SSE41>)
        {
            float result;
            __m128 in = _mm_set_ss(x);
            __m128 rounded = _mm_floor_ss(in, in);
            _mm_store_ss(&result, rounded);
            return result;
        }
        else if constexpr (std::is_same_v<U, double>
                           && simd::compile_time::has<simd::Feature::SSE41>)
        {
            double result;
            __m128d in = _mm_set_sd(x);
            __m128d rounded = _mm_floor_sd(in, in);
            _mm_store_sd(&result, rounded);
            return result;
        }
        return std::floor(x);
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U round(U x) noexcept
    {
        if constexpr (std::is_same_v<U, float> && simd::compile_time::has<simd::Feature::SSE41>)
        {
            float result;
            __m128 in = _mm_set_ss(x);
            __m128 rounded = _mm_round_ss(in, in, _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC);
            _mm_store_ss(&result, rounded);
            return result;
        }
        else if constexpr (std::is_same_v<U, double>
                           && simd::compile_time::has<simd::Feature::SSE41>)
        {
            double result;
            __m128d in = _mm_set_sd(x);
            __m128d rounded = _mm_round_sd(in, in, _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC);
            _mm_store_sd(&result, rounded);
            return result;
        }
        return std::round(x);
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U cbrt(U x) noexcept
    {
        if (x == U(0) || std::isnan(x) || std::isinf(x))
        {
            return x;
        }

        U y;
        if constexpr (std::is_same_v<U, float>)
        {
            std::int32_t ix;
            std::memcpy(&ix, &x, sizeof(float));
            ix = ix / 3 + 709'921'077;
            std::memcpy(&y, &ix, sizeof(float));
        }
        else
        {
            int exp;
            U mant = std::frexp(std::abs(x), &exp);
            y = std::ldexp(std::pow(U(2), U(exp % 3)) * std::pow(mant, U(1) / U(3)), exp / 3);
        }

        if constexpr (simd::compile_time::has<simd::Feature::FMA>())
        {
            for (int i = 0; i < (std::is_same_v<U, float> ? 2 : 3); ++i)
            {
                U y2 = y * y;
                U y3 = y * y2;
                y = std::fma(y, U(2), x / y2) * U(1) / U(3);
            }
        }
        else
        {
            for (int i = 0; i < (std::is_same_v<U, float> ? 2 : 3); ++i)
            {
                U y2 = y * y;
                y = (U(2) * y + x / y2) * U(1) / U(3);
            }
        }

        return std::signbit(x) ? -y : y;
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U hypot(U x, U y) noexcept
    {
        if (std::isnan(x) || std::isnan(y))
            return std::numeric_limits<U>::quiet_NaN();
        if (std::isinf(x) || std::isinf(y))
            return std::numeric_limits<U>::infinity();

        x = std::abs(x);
        y = std::abs(y);

        if (x > y)
            std::swap(x, y);
        if (y == U(0))
            return U(0);

        const U ratio = x / y;

        if constexpr (simd::compile_time::has<simd::Feature::FMA>())
        {
            return y * std::sqrt(std::fma(ratio, ratio, U(1)));
        }
        else
        {
            return y * std::sqrt(U(1) + ratio * ratio);
        }
    }
};

}  // namespace umath

#endif  // #end of include guard LIB_UMATH_HPP_p7q7hl