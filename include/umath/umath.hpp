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

    template <typename U>
    [[nodiscard]] static U impl_atan_core(U x) noexcept
    {
        static constexpr std::array<U, 11> C = {U(1.0),
                                                U(-0.3333333333333333333333333333333),
                                                U(0.2),
                                                U(-0.1428571428571428571428571428571),
                                                U(0.1111111111111111111111111111111),
                                                U(-0.0909090909090909090909090909091),
                                                U(0.0769230769230769230769230769231),
                                                U(-0.0666666666666666666666666666667),
                                                U(0.0588235294117647058823529411765),
                                                U(-0.0526315789473684210526315789474),
                                                U(0.0476190476190476190476190476190)};

        const U x2 = x * x;
        U result = U(0);

        if constexpr (simd::compile_time::has<simd::Feature::FMA>())
        {
            result = C[10];
            for (int i = 9; i >= 0; --i)
            {
                result = std::fma(result, x2, C[i]);
            }
        }
        else
        {
            // standard Horner
            result = C[10];
            for (int i = 9; i >= 0; --i)
            {
                result = result * x2 + C[i];
            }
        }

        return result * x;
    }

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

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U log10(U x) noexcept
    {
        if (std::isnan(x) || x < U(0))
            return std::numeric_limits<U>::quiet_NaN();
        if (x == U(0))
            return -std::numeric_limits<U>::infinity();
        if (x == std::numeric_limits<U>::infinity())
            return std::numeric_limits<U>::infinity();
        if (x == U(1))
            return U(0);

        static constexpr U LOG10_2 = U(0.301029995663981195213738894724);

        int exp;
        U mantissa = std::frexp(x, &exp);

        if (mantissa < U(0.707106781186547524))
        {
            mantissa *= U(2);
            exp--;
        }

        mantissa -= U(1);

        if constexpr (simd::compile_time::has<simd::Feature::FMA>())
        {
            static constexpr std::array<U, 7> C = {U(1.44269504088896340735992468100),
                                                   U(-0.721347520444482321375724860100),
                                                   U(0.479562288154233437462321694088),
                                                   U(-0.343919840191426222699399068339),
                                                   U(0.262203599088046952191325362389),
                                                   U(-0.206803849532505428332945754492),
                                                   U(0.168498995576053976569765001550)};

            U sum = C[6];
            for (int i = 5; i >= 0; --i)
            {
                sum = std::fma(sum, mantissa, C[i]);
            }
            return sum * mantissa * LOG10_2 + static_cast<U>(exp) * LOG10_2;
        }
        else
        {
            static constexpr std::array<U, 6> C = {U(2.885390081777926774316924906839),
                                                   U(-0.961796693925975860332579281749),
                                                   U(0.577078017761894161436831772056),
                                                   U(-0.412116952651449828911216326591),
                                                   U(0.308539341038336403451042436809),
                                                   U(-0.237806477670409415221410025403)};

            U sum = C[5];
            for (int i = 4; i >= 0; --i)
            {
                sum = sum * mantissa + C[i];
            }
            return (mantissa * sum + static_cast<U>(exp)) * LOG10_2;
        }
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U tan(U x) noexcept
    {
        if constexpr (simd::compile_time::has<simd::Feature::AVX>())
        {
            if (std::abs(x) < U(0.01))
            {
                U x2 = x * x;
                return x * (U(1) + x2 * (U(1) / U(3) + x2 * (U(2) / U(15))));
            }
        }
        return std::tan(x);
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U sqrt(U x) noexcept
    {
        return ArchOp::fast_sqrt(x);
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U rsqrt(U x) noexcept
    {
        return ArchOp::fast_rsqrt(x);
    }

    [[nodiscard]] static constexpr T max(T a, T b) noexcept
    {
        if constexpr (std::is_same_v<T, float> && simd::compile_time::has<simd::Feature::SSE>())
        {
            float result;
            __m128 va = _mm_set_ss(a);
            __m128 vb = _mm_set_ss(b);
            __m128 vr = _mm_max_ss(va, vb);
            _mm_store_ss(&result, vr);
            return result;
        }
        else if constexpr (std::is_same_v<T, double>
                           && simd::compile_time::has<simd::Feature::SSE2>())
        {
            double result;
            __m128d va = _mm_set_sd(a);
            __m128d vb = _mm_set_sd(b);
            __m128d vr = _mm_max_sd(va, vb);
            _mm_store_sd(&result, vr);
            return result;
        }
        return (a > b) ? a : b;
    }

    [[nodiscard]] static constexpr T min(T a, T b) noexcept
    {
        if constexpr (std::is_same_v<T, float> && simd::compile_time::has<simd::Feature::SSE>())
        {
            float result;
            __m128 va = _mm_set_ss(a);
            __m128 vb = _mm_set_ss(b);
            __m128 vr = _mm_min_ss(va, vb);
            _mm_store_ss(&result, vr);
            return result;
        }
        else if constexpr (std::is_same_v<T, double>
                           && simd::compile_time::has<simd::Feature::SSE2>())
        {
            double result;
            __m128d va = _mm_set_sd(a);
            __m128d vb = _mm_set_sd(b);
            __m128d vr = _mm_min_sd(va, vb);
            _mm_store_sd(&result, vr);
            return result;
        }
        return (a < b) ? a : b;
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U exp(U x) noexcept
    {
        if constexpr (simd::compile_time::has<simd::Feature::AVX>())
        {
            if (std::abs(x) < U(0.01))
            {
                U sum = U(1) + x;
                U term = x;
                for (int i = 2; i < 6; ++i)
                {
                    term *= x / i;
                    sum += term;
                }
                return sum;
            }
        }
        return std::exp(x);
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static constexpr U toRadians(U degrees) noexcept
    {
        return degrees * DEG_TO_RAD;
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static constexpr U toDegrees(U radians) noexcept
    {
        return radians * RAD_TO_DEG;
    }

    [[nodiscard]] static constexpr T signum(T x) noexcept
    {
        return FastOp::signum(x);
    }

    template <typename U = T>
    requires Integral<U>
    [[nodiscard]] static constexpr U floorDiv(U x, U y)
    {
        if (y == 0)
        {
            throw std::domain_error("Division by zero in floorDiv");
        }
        U q = x / y;
        U r = x % y;
        if ((r != 0) && ((r < 0) != (y < 0)))
            --q;
        return q;
    }

    template <typename U = T>
    requires Integral<U>
    [[nodiscard]] static constexpr U floorMod(U x, U y)
    {
        if (y == 0)
        {
            throw std::domain_error("Division by zero in floorMod");
        }
        U r = x % y;
        if ((r != 0) && ((r < 0) != (y < 0)))
            r += y;
        return r;
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static int getExponent(U d) noexcept
    {
        if (d == U(0))
        {
            return std::numeric_limits<U>::min_exponent - 1;
        }
        int exponent;
        std::frexp(std::abs(d), &exponent);
        return exponent - 1;
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U IEEEremainder(U x, U y) noexcept
    {
        if (std::isnan(x) || std::isnan(y) || std::isinf(x) || y == U(0))
        {
            return std::numeric_limits<U>::quiet_NaN();
        }
        if (std::isinf(y))
        {
            return x;
        }
        U r = std::remainder(x, y);
        if (r == U(0))
        {
            r = U(0) * x;
        }
        return r;
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U expm1(U x) noexcept
    {
        if (std::abs(x) < U(1e-5))
        {
            U x2 = x * x;
            return x + x2 * (U(0.5) + x * (U(1.0) / U(6.0) + x * U(1.0) / U(24.0)));
        }
        return std::expm1(x);
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U log1p(U x) noexcept
    {
        if (std::abs(x) < U(1e-5))
        {
            U x2 = x * x;
            return x - x2 * U(0.5) + x * x2 * U(1.0) / U(3.0);
        }
        return std::log1p(x);
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U pow(U base, U exponent) noexcept
    {
        if constexpr (simd::compile_time::has<simd::Feature::AVX>())
        {
            if (exponent == U(2))
                return base * base;
            if (exponent == U(3))
                return base * base * base;
            if (exponent == U(0.5))
                return sqrt(base);
        }
        return std::pow(base, exponent);
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U scalb(U d, int scaleFactor) noexcept
    {
        if (std::isnan(d) || std::isinf(d) || d == U(0))
            return d;
        if (scaleFactor == 0)
            return d;
        if (scaleFactor >= std::numeric_limits<int>::max() / 2)
        {
            return d * std::numeric_limits<U>::infinity();
        }
        if (scaleFactor <= std::numeric_limits<int>::min() / 2)
        {
            return d * U(0);
        }
        return std::scalbn(d, scaleFactor);
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U nextAfter(U start, U direction) noexcept
    {
        if (std::isnan(start) || std::isnan(direction))
            return std::numeric_limits<U>::quiet_NaN();
        if (start == direction)
            return direction;
        if (std::isinf(start))
        {
            return direction < start ? std::numeric_limits<U>::max()
                                     : std::numeric_limits<U>::infinity();
        }
        if (start == U(0))
        {
            return direction < U(0) ? -std::numeric_limits<U>::denorm_min()
                                    : std::numeric_limits<U>::denorm_min();
        }
        return std::nextafter(start, direction);
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U sinh(U x) noexcept
    {
        if (Math::abs(x) < U(0.01))
        {
            U x2 = x * x;
            return x * (U(1) + x2 * (U(1) / U(6) + x2 * (U(1) / U(120))));
        }
        return std::sinh(x);
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U cosh(U x) noexcept
    {
        if (Math::abs(x) < U(0.01))
        {
            U x2 = x * x;
            return U(1) + x2 * (U(0.5) + x2 * (U(1) / U(24)));
        }
        return std::cosh(x);
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U tanh(U x) noexcept
    {
        if (Math::abs(x) < U(0.01))
        {
            U x2 = x * x;
            return x * (U(1) - x2 * (U(1) / U(3)));
        }
        return std::tanh(x);
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U atan(U x) noexcept
    {
        if (std::isnan(x))
            return std::numeric_limits<U>::quiet_NaN();
        if (x == U(0))
            return U(0);
        if (std::isinf(x))
            return std::copysign(PI / U(2), x);

        // taylor series approximation for small x
        if (std::abs(x) < U(0.01))
        {
            U x2 = x * x;
            return x * (U(1) - x2 * (U(1) / U(3) - x2 * (U(1) / U(5) - x2 * U(1) / U(7))));
        }

        // atan(x) calculating strategy:
        // 1. if |x| > 1 atan(x) = π/2 - atan(1/x)
        // 2. if |x| > tan(π/12) atan(x) = π/6 + atan((x*√3-1)/(x+√3))
        const bool negate = x < U(0);
        x = Math::abs(x);

        U result;
        if (x > U(1))
        {
            // |x| > 1 statement
            x = U(1) / x;
            result = PI / U(2) - impl_atan_core(x);
        }
        else if (x > U(0.26794919243112270647))
        {
            // tan(π/12)
            // π/6 reduction
            static constexpr U SQRT3 = U(1.7320508075688772935274463415059);
            result = PI / U(6) + impl_atan_core((x * SQRT3 - U(1)) / (x + SQRT3));
        }
        else
        {
            result = impl_atan_core(x);
        }

        return negate ? -result : result;
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U atan2(U y, U x) noexcept
    {
        if (x == U(0))
        {
            if (y > U(0))
            {
                return PI / U(2);
            }
            if (y < U(0))
            {
                return -PI / U(2);
            }
        }

        return std::atan2(y, x);
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U asin(U x) noexcept
    {
        if (std::abs(x) < U(0.01))
        {
            U x2 = x * x;
            return x * (U(1) + x2 * (U(1) / U(6) + x2 * (U(3) / U(40))));
        }
        return std::asin(x);
    }

    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static U acos(U x) noexcept
    {
        if (x > U(0.999))
        {
            U delta = U(1) - x;
            return std::sqrt(U(2) * delta);
        }
        if (x < U(-0.999))
        {
            U delta = U(-1) - x;
            return PI - std::sqrt(U(-2) * delta);
        }
        return std::acos(x);
    }
};

}  // namespace umath

#endif  // #end of include guard LIB_UMATH_HPP_p7q7hl