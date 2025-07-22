#ifndef LIB_UMATH_HPP_p7q7hl
#define LIB_UMATH_HPP_p7q7hl

#include <simd/feature_check.hpp>

#include <cmath>
#include <limits>
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
};

}  // namespace detail

}  // namespace umath

#endif  // #end of include guard LIB_UMATH_HPP_p7q7hl