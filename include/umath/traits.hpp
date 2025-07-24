#ifndef LIB_UMATH_TRAITS_HPP_x9k2m4
#define LIB_UMATH_TRAITS_HPP_x9k2m4

#include <algorithm>
#include <limits>
#include <type_traits>

#if __cplusplus >= 202'002L
    #include <concepts>
    #include <numbers>
    #define UMATH_CONCEPTS_ENABLED
#endif

namespace umath
{

#ifdef UMATH_CONCEPTS_ENABLED
template <typename T>
concept Arithmetic = std::is_arithmetic_v<T>;

template <typename T>
concept FloatingPoint = std::is_floating_point_v<T>;

template <typename T>
concept Integral = std::is_integral_v<T>;

template <typename T>
concept SignedIntegral = Integral<T> && std::is_signed_v<T>;

template <typename T>
concept UnsignedIntegral = Integral<T> && std::is_unsigned_v<T>;
#else
template <typename T>
using Arithmetic = std::enable_if_t<std::is_arithmetic_v<T>, bool>;

template <typename T>
using FloatingPoint = std::enable_if_t<std::is_floating_point_v<T>, bool>;

template <typename T>
using Integral = std::enable_if_t<std::is_integral_v<T>, bool>;

template <typename T>
using SignedIntegral = std::enable_if_t<std::is_integral_v<T> && std::is_signed_v<T>, bool>;

template <typename T>
using UnsignedIntegral = std::enable_if_t<std::is_integral_v<T> && std::is_unsigned_v<T>, bool>;
#endif

template <typename T>
struct numeric_traits
{
    static constexpr bool is_valid =
        std::is_arithmetic_v<T> && !std::is_same_v<T, bool> && !std::is_same_v<T, char>;

    using precision_type = std::conditional_t<std::is_floating_point_v<T>, T, double>;

    static constexpr T epsilon =
        std::is_floating_point_v<T> ? std::numeric_limits<T>::epsilon() * T{100} : T{};

    static constexpr bool is_floating_point = std::is_floating_point_v<T>;
    static constexpr bool is_integral = std::is_integral_v<T>;
    static constexpr bool is_signed = std::is_signed_v<T>;
    static constexpr bool is_unsigned = std::is_unsigned_v<T>;
};

template <typename T>
struct float_traits
{
    static constexpr bool is_float = std::is_floating_point_v<T>;
    static constexpr T epsilon = std::numeric_limits<T>::epsilon();
    static constexpr T infinity = std::numeric_limits<T>::infinity();
    static constexpr T quiet_nan = std::numeric_limits<T>::quiet_NaN();
    static constexpr T signaling_nan = std::numeric_limits<T>::signaling_NaN();
    static constexpr int max_exponent = std::numeric_limits<T>::max_exponent;
    static constexpr int min_exponent = std::numeric_limits<T>::min_exponent;
    static constexpr T max_value = std::numeric_limits<T>::max();
    static constexpr T min_value = std::numeric_limits<T>::lowest();
    static constexpr T denorm_min = std::numeric_limits<T>::denorm_min();
};

template <typename T>
struct integer_traits
{
    static constexpr bool is_integer = std::is_integral_v<T>;
    static constexpr T max_value = std::numeric_limits<T>::max();
    static constexpr T min_value = std::numeric_limits<T>::min();
    static constexpr int digits = std::numeric_limits<T>::digits;
    static constexpr int digits10 = std::numeric_limits<T>::digits10;
    static constexpr bool is_signed = std::is_signed_v<T>;
    static constexpr bool is_unsigned = std::is_unsigned_v<T>;
};

namespace constants
{
#if __cplusplus >= 202'002L
template <typename T>
constexpr T pi = std::numbers::pi_v<T>;

template <typename T>
constexpr T e = std::numbers::e_v<T>;

template <typename T>
constexpr T sqrt2 = std::numbers::sqrt2_v<T>;

template <typename T>
constexpr T sqrt3 = std::numbers::sqrt3_v<T>;

template <typename T>
constexpr T ln2 = std::numbers::ln2_v<T>;

template <typename T>
constexpr T ln10 = std::numbers::ln10_v<T>;
#else
template <typename T>
constexpr T pi = T(3.14159265358979323846264338327950288);

template <typename T>
constexpr T e = T(2.71828182845904523536028747135266250);

template <typename T>
constexpr T sqrt2 = T(1.41421356237309504880168872420969808);

template <typename T>
constexpr T sqrt3 = T(1.73205080756887729352744634150587237);

template <typename T>
constexpr T ln2 = T(0.693147180559945309417232121458176568);

template <typename T>
constexpr T ln10 = T(2.30258509299404568401799145468436421);
#endif

template <typename T>
constexpr T half_pi = pi<T> / T(2);

template <typename T>
constexpr T two_pi = pi<T> * T(2);

template <typename T>
constexpr T deg_to_rad = pi<T> / T(180);

template <typename T>
constexpr T rad_to_deg = T(180) / pi<T>;
}  // namespace constants

namespace detail
{
template <typename T>
constexpr bool approximately_equal(T a, T b, T tolerance = numeric_traits<T>::epsilon) noexcept
{
    if constexpr (std::is_floating_point_v<T>)
    {
        return std::abs(a - b) <= tolerance * std::max({T{1}, std::abs(a), std::abs(b)});
    }
    else
    {
        return a == b;
    }
}

template <typename T>
constexpr bool is_power_of_two(T x) noexcept
{
    static_assert(std::is_integral_v<T>, "is_power_of_two requires integral type");
    return x > 0 && (x & (x - 1)) == 0;
}

template <typename T>
constexpr T fast_abs(T x) noexcept
{
    if constexpr (std::is_unsigned_v<T>)
    {
        return x;
    }
    else if constexpr (std::is_integral_v<T>)
    {
        const T mask = x >> (std::numeric_limits<T>::digits - 1);
        return (x + mask) ^ mask;
    }
    else
    {
        return std::abs(x);
    }
}

template <typename T>
constexpr T signum(T x) noexcept
{
    return (T(0) < x) - (x < T(0));
}
}  // namespace detail

using f32 = float;
using f64 = double;
using i8 = std::int8_t;
using i16 = std::int16_t;
using i32 = std::int32_t;
using i64 = std::int64_t;
using u8 = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;
using usize = std::size_t;

}  // namespace umath

#endif  // End of include guard: LIB_UMATH_TRAITS_HPP_x9k2m4