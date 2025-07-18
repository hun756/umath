#ifndef LIB_UMATH_VEC2_HPP_a5enmn
#define LIB_UMATH_VEC2_HPP_a5enmn

#include <umath/umath.hpp>

#include <cmath>
#include <random>
#include <type_traits>

namespace umath
{
namespace detail
{

template <typename T>
struct numeric_traits
{
    static constexpr bool is_valid =
        std::is_arithmetic_v<T> && !std::is_same_v<T, bool> && !std::is_same_v<T, char>;
    using precision_type = std::conditional_t<std::is_floating_point_v<T>, T, double>;
    static constexpr T epsilon =
        std::is_floating_point_v<T> ? std::numeric_limits<T>::epsilon() * T{100} : T{};
};

template <typename T, typename ValueType = void>
struct is_vector2_like_impl
{
    template <typename U>
    static auto test(U* p) -> decltype(p->x,
                                       p->y,
                                       std::is_assignable_v<decltype(p->x)&, ValueType>,
                                       std::is_assignable_v<decltype(p->y)&, ValueType>,
                                       std::true_type{});
    static std::false_type test(...);

    static constexpr bool value = decltype(test(static_cast<T*>(nullptr)))::value;
};

template <typename T>
struct is_vector2_like_impl<T, void>
{
    template <typename U>
    static auto test(U* p) -> decltype(p->x, p->y, std::true_type{});
    static std::false_type test(...);

    static constexpr bool value = decltype(test(static_cast<T*>(nullptr)))::value;
};

template <typename T, typename ValueType = void>
inline constexpr bool is_vector2_like_v = is_vector2_like_impl<T, ValueType>::value;

template <typename T>
constexpr T fast_inverse_sqrt(T x) noexcept
{
    if constexpr (std::is_same_v<T, float>)
    {
        if (std::is_constant_evaluated())
        {
            return T{1} / std::sqrt(x);
        }
        else
        {
            union
            {
                float f;
                uint32_t i;
            } conv = {x};
            conv.i = 0x5f'37'59'df - (conv.i >> 1);
            conv.f *= T{1.5} - (x * T{0.5} * conv.f * conv.f);
            conv.f *= T{1.5} - (x * T{0.5} * conv.f * conv.f);
            return conv.f;
        }
    }
    else
    {
        return T{1} / std::sqrt(x);
    }
}

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

inline std::random_device rd;
inline std::mt19937 gen{rd()};
inline std::normal_distribution<double> normal_dist{0.0, 1.0};
inline std::uniform_real_distribution<double> uniform_dist{0.0, 1.0};
}  // namespace detail

constexpr double PI = 3.14159265358979323846;
constexpr double HALF_PI = PI * 0.5;
constexpr double TWO_PI = PI * 2.0;

#ifdef MATH_CONCEPTS_ENABLED
template <typename T>
concept Arithmetic =
    std::is_arithmetic_v<T> && !std::is_same_v<T, bool> && !std::is_same_v<T, char>;

template <typename T>
concept FloatingPoint = std::is_floating_point_v<T>;

template <typename T, typename ValueType = void>
concept Vector2Like =
    requires(T t) {
        t.x;
        t.y;
    }
    && (std::is_void_v<ValueType>
        || (std::is_assignable_v<decltype(std::declval<T>().x)&, ValueType>
            && std::is_assignable_v<decltype(std::declval<T>().y)&, ValueType>));
#endif

enum class ComparisonMode
{
    LEXICOGRAPHIC,
    MAGNITUDE,
    ANGLE,
    MANHATTAN
};

template <typename T
#ifdef MATH_CONCEPTS_ENABLED
          >
requires Arithmetic<T>
#else
          ,
          typename = std::enable_if_t<detail::numeric_traits<T>::is_valid>>
#endif
class alignas(sizeof(T) * 2) Vector2
{
    using value_type = T;
    using reference = T&;
    using const_reference = const T&;
    using precision_type = typename detail::numeric_traits<T>::precision_type;
    using size_type = std::size_t;

    static constexpr size_type dimensions = 2;
    static constexpr T epsilon = detail::numeric_traits<T>::epsilon;

    constexpr Vector2() noexcept : x{}, y{} {}

    constexpr Vector2(const Vector2&) noexcept = default;
    constexpr Vector2(Vector2&&) noexcept = default;

    constexpr Vector2& operator=(const Vector2&) noexcept = default;
    constexpr Vector2& operator=(Vector2&&) noexcept = default;

    explicit constexpr Vector2(T value) noexcept : x{value}, y{value} {}
    constexpr Vector2(T x_val, T y_val) noexcept : x{x_val}, y{y_val} {}

    template <typename U
#ifdef MATH_CONCEPTS_ENABLED
              >
    requires Arithmetic<U>
                 && std::convertible_to<U, T>
#else
              ,
              typename = std::enable_if_t<detail::numeric_traits<U>::is_valid
                                          && std::is_convertible_v<U, T>>
#endif
                        >
             explicit constexpr Vector2(const Vector2<U>& other) noexcept
        : x{static_cast<T>(other.x)},
    y{static_cast<T>(other.y)}
    {
    }

    template <typename ValueType = T
#ifdef MATH_CONCEPTS_ENABLED
        > requires Arithmetic<ValueType> && std::convertible_to<ValueType, T>
#else
        , typename = std::enable_if_t<detail::numeric_traits<ValueType>::is_valid &&
                                      std::is_convertible_v<ValueType, T>>
        >
#endif
    class ImmutableVector2
    {
    private:
        const ValueType x_, y_;
    };

    // --
    T x, y;
};

}  // namespace umath

#endif  // End of include guard: LIB_UMATH_VEC2_HPP_a5enmn