#ifndef LIB_UMATH_HPP_p7q7hl
#define LIB_UMATH_HPP_p7q7hl

#include <stdexcept>
#include <type_traits>
#include <limits>

namespace umath
{
class arithmetic_overflow final : public std::runtime_error
{
public:
    explicit arithmetic_overflow(const char* message) noexcept
        : std::runtime_error(message)
    {
    }
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

// Todo!: continue with our simd library

} // namespace umath

#endif // #end of include guard LIB_UMATH_HPP_p7q7hl