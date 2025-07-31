#ifndef LIB_UMATH_VEC2_HPP_a5enmn
#define LIB_UMATH_VEC2_HPP_a5enmn

#include <umath/traits.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <random>
#include <stdexcept>
#include <utility>

namespace umath
{
namespace detail
{

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
constexpr T approximate_inverse_sqrt(T x) noexcept
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


inline std::random_device rd;
inline std::mt19937 gen{rd()};
inline std::normal_distribution<double> normal_dist{0.0, 1.0};
inline std::uniform_real_distribution<double> uniform_dist{0.0, 1.0};
}  // namespace detail

constexpr double PI = constants::pi<double>;
constexpr double HALF_PI = constants::half_pi<double>;
constexpr double TWO_PI = constants::two_pi<double>;

template<typename T>
constexpr T pi_v = constants::pi<T>;
template<typename T>
constexpr T half_pi_v = constants::half_pi<T>;
template<typename T>
constexpr T two_pi_v = constants::two_pi<T>;

#ifdef UMATH_CONCEPTS_ENABLED
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

#ifdef UMATH_CONCEPTS_ENABLED
template <typename T>
requires Arithmetic<T>
class alignas(sizeof(T) * 2) Vector2
#else
template <typename T, typename = std::enable_if_t<numeric_traits<T>::is_valid>>
class alignas(sizeof(T) * 2) Vector2
#endif
{
public:
    using value_type = T;
    using reference = T&;
    using const_reference = const T&;
    using precision_type = typename numeric_traits<T>::precision_type;
    using size_type = std::size_t;

    static constexpr size_type dimensions = 2;
    static constexpr T epsilon = numeric_traits<T>::epsilon;

    constexpr Vector2() noexcept : x{}, y{} {}

    constexpr Vector2(const Vector2&) noexcept = default;
    constexpr Vector2(Vector2&&) noexcept = default;

    constexpr Vector2& operator=(const Vector2&) noexcept = default;
    constexpr Vector2& operator=(Vector2&&) noexcept = default;

    explicit constexpr Vector2(T value) noexcept : x{value}, y{value} {}
    constexpr Vector2(T x_val, T y_val) noexcept : x{x_val}, y{y_val} {}

#ifdef UMATH_CONCEPTS_ENABLED
    template <typename U>
    requires Arithmetic<U> && std::convertible_to<U, T>
    explicit constexpr Vector2(const Vector2<U>& other) noexcept
#else
    template <
        typename U,
        typename = std::enable_if_t<numeric_traits<U>::is_valid && std::is_convertible_v<U, T>>>
    explicit constexpr Vector2(const Vector2<U>& other) noexcept
#endif
        : x{static_cast<T>(other.x)}, y{static_cast<T>(other.y)}
    {
    }

#ifdef UMATH_CONCEPTS_ENABLED
    template <typename ValueType = T>
    requires Arithmetic<ValueType> && std::convertible_to<ValueType, T>
    class ImmutableVector2
#else
    template <typename ValueType = T,
              typename = std::enable_if_t<numeric_traits<ValueType>::is_valid
                                          && std::is_convertible_v<ValueType, T>>>
    class ImmutableVector2
#endif
    {
    private:
        const ValueType x_, y_;

    public:
        using value_type = ValueType;
        using const_reference = const ValueType&;

#ifdef UMATH_CONCEPTS_ENABLED
        template <typename X, typename Y>
        requires Arithmetic<X> && Arithmetic<Y> && std::convertible_to<X, ValueType>
                     && std::convertible_to<Y, ValueType>
        constexpr ImmutableVector2(X&& x_val, Y&& y_val) noexcept
#else
        template <
            typename X,
            typename Y,
            typename = std::enable_if_t<numeric_traits<X>::is_valid && numeric_traits<Y>::is_valid
                                        && std::is_convertible_v<X, ValueType>
                                        && std::is_convertible_v<Y, ValueType>>>
        constexpr ImmutableVector2(X&& x_val, Y&& y_val) noexcept
#endif
            : x_{static_cast<ValueType>(std::forward<X>(x_val))},
              y_{static_cast<ValueType>(std::forward<Y>(y_val))}
        {
        }

#ifdef UMATH_CONCEPTS_ENABLED
        template <typename U>
        requires Arithmetic<U> && std::convertible_to<U, ValueType>
        constexpr explicit ImmutableVector2(const Vector2<U>& other) noexcept
#else
        template <typename U,
                  typename = std::enable_if_t<numeric_traits<U>::is_valid
                                              && std::is_convertible_v<U, ValueType>>>
        constexpr explicit ImmutableVector2(const Vector2<U>& other) noexcept
#endif
            : x_{static_cast<ValueType>(other.x)}, y_{static_cast<ValueType>(other.y)}
        {
        }

#ifdef UMATH_CONCEPTS_ENABLED
        template <typename U>
        requires Arithmetic<U> && std::convertible_to<U, ValueType>
        constexpr explicit ImmutableVector2(const ImmutableVector2<U>& other) noexcept
#else
        template <typename U,
                  typename = std::enable_if_t<numeric_traits<U>::is_valid
                                              && std::is_convertible_v<U, ValueType>>>
        constexpr explicit ImmutableVector2(const ImmutableVector2<U>& other) noexcept
#endif
            : x_{static_cast<ValueType>(other.x())}, y_{static_cast<ValueType>(other.y())}
        {
        }

        [[nodiscard]] constexpr const_reference x() const noexcept
        {
            return x_;
        }
        [[nodiscard]] constexpr const_reference y() const noexcept
        {
            return y_;
        }

#ifdef UMATH_CONCEPTS_ENABLED
        template <typename U = T>
        requires Arithmetic<U> && std::convertible_to<ValueType, U>
        [[nodiscard]] constexpr operator Vector2<U>() const noexcept
#else
        template <typename U = T,
                  typename = std::enable_if_t<numeric_traits<U>::is_valid
                                              && std::is_convertible_v<ValueType, U>>>
        [[nodiscard]] constexpr operator Vector2<U>() const noexcept
#endif
        {
            return {static_cast<U>(x_), static_cast<U>(y_)};
        }

#ifdef UMATH_CONCEPTS_ENABLED
        template <typename U>
        requires Arithmetic<U>
        [[nodiscard]] constexpr auto as() const noexcept -> ImmutableVector2<U>
#else
        template <typename U, typename = std::enable_if_t<numeric_traits<U>::is_valid>>
        [[nodiscard]] constexpr auto as() const noexcept -> ImmutableVector2<U>
#endif
        {
            return ImmutableVector2<U>{static_cast<U>(x_), static_cast<U>(y_)};
        }

        [[nodiscard]] constexpr auto data() const noexcept -> const ValueType*
        {
            return &x_;
        }
        [[nodiscard]] constexpr auto hash() const noexcept -> std::size_t
        {
            constexpr std::size_t golden_ratio = 0x9e'37'79'b9;
            const auto h1 = std::hash<ValueType>{}(x_);
            const auto h2 = std::hash<ValueType>{}(y_);
            return h1 ^ (h2 + golden_ratio + (h1 << 6) + (h1 >> 2));
        }

        template <typename U>
        [[nodiscard]] constexpr bool operator==(const ImmutableVector2<U>& other) const noexcept
        {
            return detail::approximately_equal(static_cast<T>(x_), static_cast<T>(other.x()))
                   && detail::approximately_equal(static_cast<T>(y_), static_cast<T>(other.y()));
        }

        template <typename U>
        [[nodiscard]] constexpr bool operator==(const Vector2<U>& other) const noexcept
        {
            return detail::approximately_equal(static_cast<T>(x_), static_cast<T>(other.x))
                   && detail::approximately_equal(static_cast<T>(y_), static_cast<T>(other.y));
        }

        [[nodiscard]] constexpr auto operator[](std::size_t index) const noexcept -> const_reference
        {
            return index == 0 ? x_ : y_;
        }
    };

    template <typename X, typename Y>
    ImmutableVector2(X, Y) -> ImmutableVector2<std::common_type_t<X, Y>>;

    template <typename U>
    ImmutableVector2(const Vector2<U>&) -> ImmutableVector2<U>;

    template <typename X, typename Y>
    [[nodiscard]] static constexpr auto make_immutable(X&& x_val, Y&& y_val) noexcept
        -> ImmutableVector2<std::common_type_t<std::decay_t<X>, std::decay_t<Y>>>
    {
        return {std::forward<X>(x_val), std::forward<Y>(y_val)};
    }

    template <typename U>
    [[nodiscard]] static constexpr auto make_immutable(const Vector2<U>& vec) noexcept
        -> ImmutableVector2<U>
    {
        return ImmutableVector2<U>{vec};
    }

    inline static const ImmutableVector2<T> ZERO_IMMUTABLE{T{}, T{}};
    inline static const ImmutableVector2<T> ONE_IMMUTABLE{T{1}, T{1}};
    inline static const ImmutableVector2<T> NEG_ONE_IMMUTABLE{T{-1}, T{-1}};
    inline static const ImmutableVector2<T> UNIT_X_IMMUTABLE{T{1}, T{}};
    inline static const ImmutableVector2<T> UNIT_Y_IMMUTABLE{T{}, T{1}};
    inline static const ImmutableVector2<T> UP_IMMUTABLE{T{}, T{1}};
    inline static const ImmutableVector2<T> DOWN_IMMUTABLE{T{}, T{-1}};
    inline static const ImmutableVector2<T> LEFT_IMMUTABLE{T{-1}, T{}};
    inline static const ImmutableVector2<T> RIGHT_IMMUTABLE{T{1}, T{}};

#ifdef UMATH_CONCEPTS_ENABLED
    template <typename U = T>
    requires FloatingPoint<U>
    inline static const ImmutableVector2<U> INFINITY_IMMUTABLE{std::numeric_limits<U>::infinity(),
                                                               std::numeric_limits<U>::infinity()};
#else
    template <typename U = T, std::enable_if_t<std::is_floating_point_v<U>, int> = 0>
    inline static const ImmutableVector2<U> INFINITY_IMMUTABLE{std::numeric_limits<U>::infinity(),
                                                               std::numeric_limits<U>::infinity()};
#endif
    inline static const Vector2 ZERO{T{}, T{}};
    inline static const Vector2 ONE{T{1}, T{1}};
    inline static const Vector2 NEG_ONE{T{-1}, T{-1}};
    inline static const Vector2 UNIT_X{T{1}, T{}};
    inline static const Vector2 UNIT_Y{T{}, T{1}};
    inline static const Vector2 UP{T{}, T{1}};
    inline static const Vector2 DOWN{T{}, T{-1}};
    inline static const Vector2 LEFT{T{-1}, T{}};
    inline static const Vector2 RIGHT{T{1}, T{}};
    inline static const Vector2 INFINITY_VEC{std::numeric_limits<T>::infinity(),
                                             std::numeric_limits<T>::infinity()};


    template <typename U = T>
    [[nodiscard]] static constexpr auto zero_immutable() noexcept -> const ImmutableVector2<U>&
    {
        if constexpr (std::is_same_v<U, T>)
        {
            return ZERO_IMMUTABLE;
        }
        else
        {
#if defined(__cpp_constexpr) && __cpp_constexpr >= 202'211L
            static const ImmutableVector2<U> zero_typed{U{}, U{}};
            return zero_typed;
#else
            // C++20/17: static local in constexpr not allowed, return by value (not address-stable)
            return ImmutableVector2<U>{U{}, U{}};
#endif
        }
    }


    template <typename U = T>
    [[nodiscard]] static constexpr auto one_immutable() noexcept -> const ImmutableVector2<U>&
    {
        if constexpr (std::is_same_v<U, T>)
        {
            return ONE_IMMUTABLE;
        }
        else
        {
#if defined(__cpp_constexpr) && __cpp_constexpr >= 202'211L
            static const ImmutableVector2<U> one_typed{U{1}, U{1}};
            return one_typed;
#else
            return ImmutableVector2<U>{U{1}, U{1}};
#endif
        }
    }


    template <typename U = T>
    [[nodiscard]] static constexpr auto neg_one_immutable() noexcept -> const ImmutableVector2<U>&
    {
        if constexpr (std::is_same_v<U, T>)
        {
            return NEG_ONE_IMMUTABLE;
        }
        else
        {
#if defined(__cpp_constexpr) && __cpp_constexpr >= 202'211L
            static const ImmutableVector2<U> neg_one_typed{U{-1}, U{-1}};
            return neg_one_typed;
#else
            return ImmutableVector2<U>{U{-1}, U{-1}};
#endif
        }
    }


    template <typename U = T>
    [[nodiscard]] static constexpr auto unit_x_immutable() noexcept -> const ImmutableVector2<U>&
    {
        if constexpr (std::is_same_v<U, T>)
        {
            return UNIT_X_IMMUTABLE;
        }
        else
        {
#if defined(__cpp_constexpr) && __cpp_constexpr >= 202'211L
            static const ImmutableVector2<U> unit_x_typed{U{1}, U{}};
            return unit_x_typed;
#else
            return ImmutableVector2<U>{U{1}, U{}};
#endif
        }
    }


    template <typename U = T>
    [[nodiscard]] static constexpr auto unit_y_immutable() noexcept -> const ImmutableVector2<U>&
    {
        if constexpr (std::is_same_v<U, T>)
        {
            return UNIT_Y_IMMUTABLE;
        }
        else
        {
#if defined(__cpp_constexpr) && __cpp_constexpr >= 202'211L
            static const ImmutableVector2<U> unit_y_typed{U{}, U{1}};
            return unit_y_typed;
#else
            return ImmutableVector2<U>{U{}, U{1}};
#endif
        }
    }

    template <typename U = T>
    [[nodiscard]] static constexpr auto up_immutable() noexcept -> const ImmutableVector2<U>&
    {
        return unit_y_immutable<U>();
    }

    template <typename U = T>
    [[nodiscard]] static constexpr auto down_immutable() noexcept -> const ImmutableVector2<U>&
    {
        if constexpr (std::is_same_v<U, T>)
        {
            return DOWN_IMMUTABLE;
        }
        else
        {
#if defined(__cpp_constexpr) && __cpp_constexpr >= 202'211L
            static const ImmutableVector2<U> down_typed{U{}, U{-1}};
            return down_typed;
#else
            return ImmutableVector2<U>{U{}, U{-1}};
#endif
        }
    }

    template <typename U = T>
    [[nodiscard]] static constexpr auto left_immutable() noexcept -> const ImmutableVector2<U>&
    {
        if constexpr (std::is_same_v<U, T>)
        {
            return LEFT_IMMUTABLE;
        }
        else
        {
#if defined(__cpp_constexpr) && __cpp_constexpr >= 202'211L
            static const ImmutableVector2<U> left_typed{U{-1}, U{}};
            return left_typed;
#else
            return ImmutableVector2<U>{U{-1}, U{}};
#endif
        }
    }

    template <typename U = T>
    [[nodiscard]] static constexpr auto right_immutable() noexcept -> const ImmutableVector2<U>&
    {
        if constexpr (std::is_same_v<U, T>)
        {
            return RIGHT_IMMUTABLE;
        }
        else
        {
#if defined(__cpp_constexpr) && __cpp_constexpr >= 202'211L
            static const ImmutableVector2<U> right_typed{U{1}, U{}};
            return right_typed;
#else
            return ImmutableVector2<U>{U{1}, U{}};
#endif
        }
    }

#ifdef UMATH_CONCEPTS_ENABLED
    template <typename U = T>
    requires FloatingPoint<U>
    [[nodiscard]] static constexpr auto infinity_immutable() noexcept -> const ImmutableVector2<U>&
#else
    template <typename U = T, typename = std::enable_if_t<std::is_floating_point_v<U>>>
    [[nodiscard]] static constexpr auto infinity_immutable() noexcept -> const ImmutableVector2<U>&
#endif
    {
#if defined(__cpp_constexpr) && __cpp_constexpr >= 202'211L
        static const ImmutableVector2<U> infinity_typed{std::numeric_limits<U>::infinity(),
                                                        std::numeric_limits<U>::infinity()};
        return infinity_typed;
#else
        return ImmutableVector2<U>{std::numeric_limits<U>::infinity(),
                                   std::numeric_limits<U>::infinity()};
#endif
    }

    static constexpr const Vector2& zero() noexcept
    {
        return ZERO;
    }

    static constexpr const Vector2& one() noexcept
    {
        return ONE;
    }

    static constexpr const Vector2& neg_one() noexcept
    {
        return NEG_ONE;
    }

    static constexpr const Vector2& unit_x() noexcept
    {
        return UNIT_X;
    }

    static constexpr const Vector2& unit_y() noexcept
    {
        return UNIT_Y;
    }

    static constexpr const Vector2& up() noexcept
    {
        return UP;
    }

    static constexpr const Vector2& down() noexcept
    {
        return DOWN;
    }

    static constexpr const Vector2& left() noexcept
    {
        return LEFT;
    }

    static constexpr const Vector2& right() noexcept
    {
        return RIGHT;
    }

    static constexpr const Vector2& infinity_vec() noexcept
    {
        return INFINITY_VEC;
    }

    template <typename U>
    static constexpr Vector2 from(const Vector2<U>& other) noexcept
    {
        return Vector2{static_cast<T>(other.x), static_cast<T>(other.y)};
    }

    template <typename Container>
    static constexpr Vector2 from_array(const Container& arr, size_type offset = 0)
    {
        return Vector2{static_cast<T>(arr[offset]), static_cast<T>(arr[offset + 1])};
    }

    static constexpr Vector2 create(T x_val = T{}, T y_val = T{}) noexcept
    {
        return Vector2{x_val, y_val};
    }

    constexpr void set(T x_val, T y_val) noexcept
    {
        x = x_val;
        y = y_val;
    }

    constexpr void set_x(T value) noexcept
    {
        x = value;
    }

    constexpr void set_y(T value) noexcept
    {
        y = value;
    }

    constexpr Vector2 clone() const noexcept
    {
        return *this;
    }

#ifdef UMATH_CONCEPTS_ENABLED
    template <typename OutputVector = Vector2>
    requires Vector2Like<OutputVector, T>
    static constexpr OutputVector add(const Vector2& a,
                                      const Vector2& b,
                                      OutputVector* out = nullptr) noexcept
#else
    template <typename OutputVector = Vector2,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>>
    static constexpr OutputVector add(const Vector2& a,
                                      const Vector2& b,
                                      OutputVector* out = nullptr) noexcept
#endif
    {
        const auto result = OutputVector{a.x + b.x, a.y + b.y};
        if (out)
            *out = result;
        return result;
    }

#ifdef UMATH_CONCEPTS_ENABLED
    template <typename OutputVector = Vector2>
    requires Vector2Like<OutputVector, T>
    static constexpr OutputVector add_scalar(const Vector2& a,
                                             T scalar,
                                             OutputVector* out = nullptr) noexcept
#else
    template <typename OutputVector = Vector2,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>>
    static constexpr OutputVector add_scalar(const Vector2& a,
                                             T scalar,
                                             OutputVector* out = nullptr) noexcept
#endif
    {
        const auto result = OutputVector{a.x + scalar, a.y + scalar};
        if (out)
            *out = result;
        return result;
    }

#ifdef UMATH_CONCEPTS_ENABLED
    template <typename OutputVector = Vector2>
    requires Vector2Like<OutputVector, T>
    static constexpr OutputVector subtract(const Vector2& a,
                                           const Vector2& b,
                                           OutputVector* out = nullptr) noexcept
#else
    template <typename OutputVector = Vector2,
              std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>* = nullptr>
    static constexpr OutputVector subtract(const Vector2& a,
                                           const Vector2& b,
                                           OutputVector* out = nullptr) noexcept
#endif
    {
        const auto result = OutputVector{a.x - b.x, a.y - b.y};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2
#ifndef UMATH_CONCEPTS_ENABLED
              ,
              std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>* = nullptr
#endif
              >
#ifdef UMATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static constexpr OutputVector subtract_scalar(const Vector2& a,
                                                  T scalar,
                                                  OutputVector* out = nullptr) noexcept
    {
        const auto result = OutputVector{a.x - scalar, a.y - scalar};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2
#ifndef UMATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef UMATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static constexpr OutputVector multiply(const Vector2& a,
                                           const Vector2& b,
                                           OutputVector* out = nullptr) noexcept
    {
        const auto result = OutputVector{a.x * b.x, a.y * b.y};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2
#ifndef UMATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef UMATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static constexpr OutputVector multiply_scalar(const Vector2& a,
                                                  T scalar,
                                                  OutputVector* out = nullptr) noexcept
    {
        const auto result = OutputVector{a.x * scalar, a.y * scalar};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2
#ifndef UMATH_CONCEPTS_ENABLED
              ,
              std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>* = nullptr
#endif
              >
#ifdef UMATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static constexpr OutputVector divide(const Vector2& a,
                                         const Vector2& b,
                                         OutputVector* out = nullptr)
    {
        if (std::abs(b.x) < epsilon || std::abs(b.y) < epsilon)
        {
            throw std::runtime_error("Division by zero or near-zero value");
        }
        const auto result = OutputVector{a.x / b.x, a.y / b.y};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2
#ifndef UMATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef UMATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static constexpr OutputVector divide_scalar(const Vector2& a,
                                                T scalar,
                                                OutputVector* out = nullptr)
    {
        if (std::abs(scalar) < epsilon)
        {
            throw std::runtime_error("Division by zero or near-zero value");
        }
        const auto result = OutputVector{a.x / scalar, a.y / scalar};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2
#ifndef UMATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef UMATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static constexpr OutputVector negate(const Vector2& a, OutputVector* out = nullptr) noexcept
    {
        const auto result = OutputVector{-a.x, -a.y};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2
#ifndef UMATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef UMATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static constexpr OutputVector inverse(const Vector2& a, OutputVector* out = nullptr)
    {
        if (std::abs(a.x) < epsilon || std::abs(a.y) < epsilon)
        {
            throw std::runtime_error("Inversion of zero or near-zero value");
        }
        const auto result = OutputVector{T{1} / a.x, T{1} / a.y};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2
#ifndef UMATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef UMATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static constexpr OutputVector inverse_safe(const Vector2& a,
                                               T default_value = T{},
                                               OutputVector* out = nullptr) noexcept
    {
        const auto result = OutputVector{std::abs(a.x) < epsilon ? default_value : T{1} / a.x,
                                         std::abs(a.y) < epsilon ? default_value : T{1} / a.y};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2
#ifndef UMATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef UMATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static constexpr OutputVector perpendicular(const Vector2& a,
                                                OutputVector* out = nullptr) noexcept
    {
        const auto result = OutputVector{-a.y, a.x};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2
#ifndef UMATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef UMATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static constexpr OutputVector perpendicular_ccw(const Vector2& a,
                                                    OutputVector* out = nullptr) noexcept
    {
        const auto result = OutputVector{a.y, -a.x};
        if (out)
            *out = result;
        return result;
    }

    [[nodiscard]] constexpr T length_squared() const noexcept
    {
        return x * x + y * y;
    }

    [[nodiscard]] constexpr precision_type length() const noexcept
    {
        if constexpr (std::is_floating_point_v<T>)
        {
            return std::sqrt(length_squared());
        }
        else
        {
            return std::sqrt(static_cast<precision_type>(length_squared()));
        }
    }

    [[nodiscard]] constexpr precision_type approximate_length() const noexcept
    {
        const auto min_comp = std::min(std::abs(x), std::abs(y));
        const auto max_comp = std::max(std::abs(x), std::abs(y));
        return static_cast<precision_type>(max_comp + static_cast<precision_type>(0.3) * min_comp);
    }

    static constexpr T dot(const Vector2& a, const Vector2& b) noexcept
    {
        return a.x * b.x + a.y * b.y;
    }

    static constexpr T cross(const Vector2& a, const Vector2& b) noexcept
    {
        return a.x * b.y - a.y * b.x;
    }

    static constexpr T length_squared(const Vector2& a) noexcept
    {
        return a.length_squared();
    }

    static constexpr precision_type length(const Vector2& a) noexcept
    {
        return a.length();
    }

    static constexpr precision_type approximate_length(const Vector2& a) noexcept
    {
        return a.approximate_length();
    }

    template <typename OutputVector = Vector2
#ifndef UMATH_CONCEPTS_ENABLED
              ,
              std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>* = nullptr
#endif
              >
#ifdef UMATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static constexpr OutputVector normalize(const Vector2& a, OutputVector* out = nullptr)
    {
        const auto len = a.length();
        if (len < epsilon)
        {
            throw std::runtime_error("Cannot normalize zero-length vector");
        }
        const auto inv_len = T{1} / static_cast<T>(len);
        const auto result = OutputVector{a.x * inv_len, a.y * inv_len};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2
#ifndef UMATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef UMATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static constexpr OutputVector normalize_approximate(const Vector2& a,
                                                        OutputVector* out = nullptr)
    {
        if constexpr (std::is_floating_point_v<T>)
        {
            const auto len_sq = a.length_squared();
            if (len_sq < epsilon * epsilon)
            {
                throw std::runtime_error("Cannot normalize zero-length vector");
            }
            const auto inv_len = detail::approximate_inverse_sqrt(len_sq);
            const auto result = OutputVector{a.x * inv_len, a.y * inv_len};
            if (out)
                *out = result;
            return result;
        }
        else
        {
            return normalize(a, out);
        }
    }

    static constexpr T distance_squared(const Vector2& a, const Vector2& b) noexcept
    {
        const auto dx = a.x - b.x;
        const auto dy = a.y - b.y;
        return dx * dx + dy * dy;
    }

    static constexpr precision_type distance(const Vector2& a, const Vector2& b) noexcept
    {
        if constexpr (std::is_floating_point_v<T>)
        {
            return std::sqrt(distance_squared(a, b));
        }
        else
        {
            return std::sqrt(static_cast<precision_type>(distance_squared(a, b)));
        }
    }

    static constexpr precision_type distance_approximate(const Vector2& a,
                                                         const Vector2& b) noexcept
    {
        const auto dx = std::abs(a.x - b.x);
        const auto dy = std::abs(a.y - b.y);
        const auto min_d = std::min(dx, dy);
        const auto max_d = std::max(dx, dy);
        return static_cast<precision_type>(max_d + static_cast<precision_type>(0.3) * min_d);
    }

    static constexpr T manhattan_distance(const Vector2& a, const Vector2& b) noexcept
    {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }

    static constexpr T chebyshev_distance(const Vector2& a, const Vector2& b) noexcept
    {
        return std::max(std::abs(a.x - b.x), std::abs(a.y - b.y));
    }

    static constexpr precision_type angle_between(const Vector2& a, const Vector2& b)
    {
        const auto dot_prod = dot(a, b);
        const auto len_a = a.length();
        const auto len_b = b.length();

        if (len_a < epsilon || len_b < epsilon)
        {
            throw std::runtime_error("Cannot calculate angle with zero-length vector");
        }

        const auto cos_theta = dot_prod / (static_cast<T>(len_a) * static_cast<T>(len_b));
        return std::acos(std::clamp(static_cast<precision_type>(cos_theta),
                                    precision_type{-1},
                                    precision_type{1}));
    }

    static constexpr precision_type angle_approximate(const Vector2& a, const Vector2& b) noexcept
    {
        const auto dx = b.x - a.x;
        const auto dy = b.y - a.y;

        if (dx == T{})
            return dy > T{} ? half_pi_v<precision_type> : -half_pi_v<precision_type>;

        const auto abs_y = std::abs(dy);
        const auto abs_x = std::abs(dx);
        const auto a_val = abs_x > abs_y ? abs_y / abs_x : abs_x / abs_y;
        const auto s = a_val * a_val;
        auto r = ((static_cast<precision_type>(-0.0464964749) * s + static_cast<precision_type>(0.15931422)) * s - static_cast<precision_type>(0.327622764)) * s * a_val + a_val;

        if (abs_y > abs_x)
            r = half_pi_v<precision_type> - r;
        if (dx < T{})
            r = pi_v<precision_type> - r;
        if (dy < T{})
            r = -r;

        return r;
    }

    constexpr precision_type angle() const noexcept
    {
        const auto angle_val =
            std::atan2(static_cast<precision_type>(y), static_cast<precision_type>(x));
        return angle_val < precision_type{} ? angle_val + two_pi_v<precision_type> : angle_val;
    }

    template <typename OutputVector = Vector2
#ifndef UMATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef UMATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static constexpr OutputVector rotate(const Vector2& v,
                                         precision_type angle_rad,
                                         OutputVector* out = nullptr) noexcept
    {
        const auto cos_a = std::cos(angle_rad);
        const auto sin_a = std::sin(angle_rad);
        const auto result = OutputVector{static_cast<T>(v.x * cos_a - v.y * sin_a),
                                         static_cast<T>(v.x * sin_a + v.y * cos_a)};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2
#ifndef UMATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef UMATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static constexpr OutputVector rotate_optimized(const Vector2& v,
                                                   precision_type angle_rad,
                                                   OutputVector* out = nullptr) noexcept
    {
        if (std::abs(angle_rad - pi_v<precision_type>) < epsilon)
        {
            const auto result = OutputVector{-v.x, -v.y};
            if (out)
                *out = result;
            return result;
        }

        if (std::abs(angle_rad - half_pi_v<precision_type>) < epsilon)
        {
            const auto result = OutputVector{-v.y, v.x};
            if (out)
                *out = result;
            return result;
        }

        if (std::abs(angle_rad + half_pi_v<precision_type>) < epsilon)
        {
            const auto result = OutputVector{v.y, -v.x};
            if (out)
                *out = result;
            return result;
        }

        if (std::abs(angle_rad) < static_cast<precision_type>(0.1))
        {
            const auto theta2_2 = (angle_rad * angle_rad) / 2;
            const auto s = angle_rad;
            const auto c = 1 - theta2_2;
            const auto result =
                OutputVector{static_cast<T>(v.x * c - v.y * s), static_cast<T>(v.x * s + v.y * c)};
            if (out)
                *out = result;
            return result;
        }

        return rotate(v, angle_rad, out);
    }

    template <typename OutputVector = Vector2
#ifndef UMATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef UMATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static constexpr OutputVector rotate_around(const Vector2& v,
                                                precision_type angle_rad,
                                                const Vector2& pivot,
                                                OutputVector* out = nullptr) noexcept
    {
        const auto cos_a = std::cos(angle_rad);
        const auto sin_a = std::sin(angle_rad);
        const auto dx = v.x - pivot.x;
        const auto dy = v.y - pivot.y;
        const auto result = OutputVector{static_cast<T>(dx * cos_a - dy * sin_a + pivot.x),
                                         static_cast<T>(dx * sin_a + dy * cos_a + pivot.y)};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2
#ifndef UMATH_CONCEPTS_ENABLED
              ,
              std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>* = nullptr
#endif
              >
#ifdef UMATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static constexpr OutputVector lerp(const Vector2& a,
                                       const Vector2& b,
                                       T t,
                                       OutputVector* out = nullptr) noexcept
    {
        const auto clamped_t = std::clamp(t, T{}, T{1});
        const auto result =
            OutputVector{a.x + (b.x - a.x) * clamped_t, a.y + (b.y - a.y) * clamped_t};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2
#ifndef UMATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef UMATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static constexpr OutputVector lerp_unclamped(const Vector2& a,
                                                 const Vector2& b,
                                                 T t,
                                                 OutputVector* out = nullptr) noexcept
    {
        const auto result = OutputVector{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2
#ifndef UMATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef UMATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static constexpr OutputVector slerp(const Vector2& a,
                                        const Vector2& b,
                                        T t,
                                        OutputVector* out = nullptr)
    {
        const auto clamped_t = std::clamp(t, T{}, T{1});
        const auto angle_a = a.angle();
        const auto angle_b = b.angle();
        auto angle_diff = angle_a - angle_b;

        if (angle_diff < 0)
            angle_diff += two_pi_v<precision_type>;
        if (angle_diff > pi_v<precision_type>)
            angle_diff -= two_pi_v<precision_type>;

        const auto result_angle = angle_a + angle_diff * clamped_t;
        const auto len_a = a.length();
        const auto len_b = b.length();
        const auto result_length = len_a + (len_b - len_a) * clamped_t;

        const auto result = OutputVector{static_cast<T>(result_length * std::cos(result_angle)),
                                         static_cast<T>(result_length * std::sin(result_angle))};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2>
    static constexpr OutputVector smooth_step(const Vector2& a,
                                              const Vector2& b,
                                              T t,
                                              OutputVector* out = nullptr) noexcept
    {
        const auto clamped_t = std::clamp(t, T{}, T{1});
        const auto smooth_t = clamped_t * clamped_t * (T{3} - T{2} * clamped_t);
        const auto result =
            OutputVector{a.x + (b.x - a.x) * smooth_t, a.y + (b.y - a.y) * smooth_t};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2>
    static constexpr OutputVector smoother_step(const Vector2& a,
                                                const Vector2& b,
                                                T t,
                                                OutputVector* out = nullptr) noexcept
    {
        const auto clamped_t = std::clamp(t, T{}, T{1});
        const auto t3 = clamped_t * clamped_t * clamped_t;
        const auto smooth_t = t3 * (T{10} - T{15} * clamped_t + T{6} * clamped_t * clamped_t);
        const auto result =
            OutputVector{a.x + (b.x - a.x) * smooth_t, a.y + (b.y - a.y) * smooth_t};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2>
    static constexpr OutputVector cubic_bezier(const Vector2& p0,
                                               const Vector2& p1,
                                               const Vector2& p2,
                                               const Vector2& p3,
                                               T t,
                                               OutputVector* out = nullptr) noexcept
    {
        const auto clamped_t = std::clamp(t, T{}, T{1});
        const auto one_minus_t = T{1} - clamped_t;
        const auto one_minus_t2 = one_minus_t * one_minus_t;
        const auto t2 = clamped_t * clamped_t;

        const auto one_minus_t3 = one_minus_t2 * one_minus_t;
        const auto t3 = t2 * clamped_t;
        const auto one_minus_t2_3t = one_minus_t2 * T{3} * clamped_t;
        const auto one_minus_t_3t2 = one_minus_t * T{3} * t2;

        const auto result = OutputVector{
            one_minus_t3 * p0.x + one_minus_t2_3t * p1.x + one_minus_t_3t2 * p2.x + t3 * p3.x,
            one_minus_t3 * p0.y + one_minus_t2_3t * p1.y + one_minus_t_3t2 * p2.y + t3 * p3.y};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2>
    static constexpr OutputVector hermite(const Vector2& p0,
                                          const Vector2& m0,
                                          const Vector2& p1,
                                          const Vector2& m1,
                                          T t,
                                          OutputVector* out = nullptr) noexcept
    {
        const auto clamped_t = std::clamp(t, T{}, T{1});
        const auto t2 = clamped_t * clamped_t;
        const auto t3 = t2 * clamped_t;

        const auto h00 = T{2} * t3 - T{3} * t2 + T{1};
        const auto h10 = t3 - T{2} * t2 + clamped_t;
        const auto h01 = -T{2} * t3 + T{3} * t2;
        const auto h11 = t3 - t2;

        const auto result = OutputVector{h00 * p0.x + h10 * m0.x + h01 * p1.x + h11 * m1.x,
                                         h00 * p0.y + h10 * m0.y + h01 * p1.y + h11 * m1.y};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2>
    static constexpr OutputVector catmull_rom(const Vector2& p0,
                                              const Vector2& p1,
                                              const Vector2& p2,
                                              const Vector2& p3,
                                              T t,
                                              T tension = T{0.5},
                                              OutputVector* out = nullptr) noexcept
    {
        const auto clamped_t = std::clamp(t, T{}, T{1});

        if (clamped_t == T{})
        {
            const auto result = OutputVector{p1.x, p1.y};
            if (out)
                *out = result;
            return result;
        }

        if (clamped_t == T{1})
        {
            const auto result = OutputVector{p2.x, p2.y};
            if (out)
                *out = result;
            return result;
        }

        const auto t2 = clamped_t * clamped_t;
        const auto t3 = t2 * clamped_t;

        const auto h00 = T{2} * t3 - T{3} * t2 + T{1};
        const auto h10 = t3 - T{2} * t2 + clamped_t;
        const auto h01 = -T{2} * t3 + T{3} * t2;
        const auto h11 = t3 - t2;

        const auto alpha = (T{1} - tension) / T{2};

        const auto m0x = alpha * (p2.x - p0.x);
        const auto m0y = alpha * (p2.y - p0.y);
        const auto m1x = alpha * (p3.x - p1.x);
        const auto m1y = alpha * (p3.y - p1.y);

        const auto result = OutputVector{h00 * p1.x + h10 * m0x + h01 * p2.x + h11 * m1x,
                                         h00 * p1.y + h10 * m0y + h01 * p2.y + h11 * m1y};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2
#ifndef UMATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef UMATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static OutputVector random(precision_type scale = precision_type{1},
                               OutputVector* out = nullptr)
    {
        const auto x = static_cast<T>(detail::normal_dist(detail::gen) * static_cast<double>(scale));
        const auto y = static_cast<T>(detail::normal_dist(detail::gen) * static_cast<double>(scale));
        const auto result = OutputVector{x, y};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2
#ifndef UMATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef UMATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static OutputVector random_quick(precision_type scale = precision_type{1},
                                     OutputVector* out = nullptr)
    {
        const auto angle = detail::uniform_dist(detail::gen) * two_pi_v<double>;
        const auto result = OutputVector{static_cast<T>(std::cos(angle) * static_cast<double>(scale)),
                                         static_cast<T>(std::sin(angle) * static_cast<double>(scale))};
        if (out)
            *out = result;
        return result;
    }

#ifdef UMATH_CONCEPTS_ENABLED
    template <typename OutputVector = Vector2>
    requires Vector2Like<OutputVector, T>
    static OutputVector random_box(T min_x, T max_x, T min_y, T max_y, OutputVector* out = nullptr)
#else
    template <typename OutputVector = Vector2,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>>
    static OutputVector random_box(T min_x, T max_x, T min_y, T max_y, OutputVector* out = nullptr)
#endif
    {
        std::uniform_real_distribution<precision_type> dist_x{static_cast<precision_type>(min_x),
                                                              static_cast<precision_type>(max_x)};
        std::uniform_real_distribution<precision_type> dist_y{static_cast<precision_type>(min_y),
                                                              static_cast<precision_type>(max_y)};
        const auto result =
            OutputVector{static_cast<T>(dist_x(detail::gen)), static_cast<T>(dist_y(detail::gen))};
        if (out)
            *out = result;
        return result;
    }

    [[nodiscard]] constexpr bool is_normalized(T tolerance = epsilon) const noexcept
    {
        return detail::approximately_equal(length_squared(), T{1}, tolerance);
    }

    [[nodiscard]] constexpr bool is_zero(T tolerance = epsilon) const noexcept
    {
        if constexpr (std::is_floating_point_v<T>)
        {
            return length_squared() <= tolerance * tolerance;
        }
        else
        {
            return x == T{} && y == T{};
        }
    }

    [[nodiscard]] constexpr bool approximately_equals(const Vector2& other,
                                                      T tolerance = epsilon) const noexcept
    {
        return detail::approximately_equal(x, other.x, tolerance)
               && detail::approximately_equal(y, other.y, tolerance);
    }

    constexpr Vector2& normalize()
    {
        const auto len = length();
        if (len < epsilon)
        {
            throw std::runtime_error("Cannot normalize zero-length vector");
        }
        const auto inv_len = T{1} / static_cast<T>(len);
        x *= inv_len;
        y *= inv_len;
        return *this;
    }

    constexpr Vector2& normalize_approximate()
    {
        if constexpr (std::is_floating_point_v<T>)
        {
            const auto len_sq = length_squared();
            if (len_sq < epsilon * epsilon)
            {
                throw std::runtime_error("Cannot normalize zero-length vector");
            }
            const auto inv_len = detail::approximate_inverse_sqrt(len_sq);
            x *= inv_len;
            y *= inv_len;
        }
        else
        {
            normalize();
        }
        return *this;
    }

    constexpr Vector2& inverse()
    {
        if (std::abs(x) < epsilon || std::abs(y) < epsilon)
        {
            throw std::runtime_error("Inversion of zero or near-zero value");
        }
        x = T{1} / x;
        y = T{1} / y;
        return *this;
    }

    constexpr Vector2& inverse_safe(T default_value = T{}) noexcept
    {
        x = std::abs(x) < epsilon ? default_value : T{1} / x;
        y = std::abs(y) < epsilon ? default_value : T{1} / y;
        return *this;
    }

    constexpr T dot(const Vector2& other) const noexcept
    {
        return x * other.x + y * other.y;
    }

    constexpr T cross(const Vector2& other) const noexcept
    {
        return x * other.y - y * other.x;
    }

    constexpr T distance_squared(const Vector2& other) const noexcept
    {
        return distance_squared(*this, other);
    }

    constexpr precision_type distance(const Vector2& other) const noexcept
    {
        return distance(*this, other);
    }

    constexpr precision_type distance_approximate(const Vector2& other) const noexcept
    {
        return distance_approximate(*this, other);
    }

    constexpr T manhattan_distance(const Vector2& other) const noexcept
    {
        return manhattan_distance(*this, other);
    }

    constexpr T chebyshev_distance(const Vector2& other) const noexcept
    {
        return chebyshev_distance(*this, other);
    }

    constexpr precision_type angle_between(const Vector2& other) const
    {
        return angle_between(*this, other);
    }

    constexpr precision_type angle_to(const Vector2& other) const noexcept
    {
        return angle_approximate(*this, other);
    }

    static constexpr int compare(const Vector2& a,
                                 const Vector2& b,
                                 ComparisonMode mode = ComparisonMode::LEXICOGRAPHIC) noexcept
    {
        switch (mode)
        {
            case ComparisonMode::LEXICOGRAPHIC:
                if (detail::approximately_equal(a.x, b.x, epsilon))
                {
                    if (detail::approximately_equal(a.y, b.y, epsilon))
                        return 0;
                    return a.y < b.y ? -1 : 1;
                }
                return a.x < b.x ? -1 : 1;

            case ComparisonMode::MAGNITUDE:
            {
                const auto len_a = a.length_squared();
                const auto len_b = b.length_squared();
                if (detail::approximately_equal(len_a, len_b, epsilon))
                    return 0;
                return len_a < len_b ? -1 : 1;
            }

            case ComparisonMode::ANGLE:
            {
                const auto angle_a = a.angle();
                const auto angle_b = b.angle();
                if (detail::approximately_equal(static_cast<T>(angle_a),
                                                static_cast<T>(angle_b),
                                                epsilon))
                    return 0;
                return angle_a < angle_b ? -1 : 1;
            }

            case ComparisonMode::MANHATTAN:
            {
                const auto dist_a = std::abs(a.x) + std::abs(a.y);
                const auto dist_b = std::abs(b.x) + std::abs(b.y);
                if (detail::approximately_equal(dist_a, dist_b, epsilon))
                    return 0;
                return dist_a < dist_b ? -1 : 1;
            }
        }
        return 0;
    }

    [[nodiscard]] constexpr Vector2 abs() const noexcept
    {
        return {std::abs(x), std::abs(y)};
    }

    [[nodiscard]] constexpr Vector2 sign() const noexcept
    {
        return {x > T{} ? T{1} : (x < T{} ? T{-1} : T{}), y > T{} ? T{1} : (y < T{} ? T{-1} : T{})};
    }

    [[nodiscard]] constexpr T min_component() const noexcept
    {
        return std::min(x, y);
    }

    [[nodiscard]] constexpr T max_component() const noexcept
    {
        return std::max(x, y);
    }

    [[nodiscard]] constexpr size_type min_component_index() const noexcept
    {
        return x <= y ? 0 : 1;
    }

    [[nodiscard]] constexpr size_type max_component_index() const noexcept
    {
        return x >= y ? 0 : 1;
    }

    [[nodiscard]] constexpr T sum() const noexcept
    {
        return x + y;
    }

    [[nodiscard]] constexpr T product() const noexcept
    {
        return x * y;
    }

    constexpr Vector2& operator+=(const Vector2& rhs) noexcept
    {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    constexpr Vector2& operator+=(T scalar) noexcept
    {
        x += scalar;
        y += scalar;
        return *this;
    }

    constexpr Vector2& operator-=(const Vector2& rhs) noexcept
    {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    constexpr Vector2& operator-=(T scalar) noexcept
    {
        x -= scalar;
        y -= scalar;
        return *this;
    }

    constexpr Vector2& operator*=(T scalar) noexcept
    {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    constexpr Vector2& operator*=(const Vector2& rhs) noexcept
    {
        x *= rhs.x;
        y *= rhs.y;
        return *this;
    }

    constexpr Vector2& operator/=(T scalar)
    {
        if (std::abs(scalar) < epsilon)
        {
            throw std::runtime_error("Division by zero or near-zero value");
        }
        x /= scalar;
        y /= scalar;
        return *this;
    }

    constexpr Vector2& operator/=(const Vector2& rhs)
    {
        if (std::abs(rhs.x) < epsilon || std::abs(rhs.y) < epsilon)
        {
            throw std::runtime_error("Division by zero or near-zero value");
        }
        x /= rhs.x;
        y /= rhs.y;
        return *this;
    }

    [[nodiscard]] constexpr Vector2 operator+() const noexcept
    {
        return *this;
    }

    [[nodiscard]] constexpr Vector2 operator-() const noexcept
    {
        return {-x, -y};
    }

    [[nodiscard]] constexpr T& operator[](size_type index) noexcept
    {
        return index == 0 ? x : y;
    }

    [[nodiscard]] constexpr const T& operator[](size_type index) const noexcept
    {
        return index == 0 ? x : y;
    }

    [[nodiscard]] constexpr T* data() noexcept
    {
        return &x;
    }

    [[nodiscard]] constexpr const T* data() const noexcept
    {
        return &x;
    }

    [[nodiscard]] constexpr std::array<T, 2> to_array() const noexcept
    {
        return {x, y};
    }

    constexpr void swap(Vector2& other) noexcept
    {
        std::swap(x, other.x);
        std::swap(y, other.y);
    }

    [[nodiscard]] constexpr std::size_t hash() const noexcept
    {
        constexpr std::size_t golden_ratio = 0x9e'37'79'b9;
        const auto h1 = std::hash<T>{}(x);
        const auto h2 = std::hash<T>{}(y);
        return h1 ^ (h2 + golden_ratio + (h1 << 6) + (h1 >> 2));
    }

    T x, y;
};

template <typename T>
[[nodiscard]] constexpr Vector2<T> operator+(const Vector2<T>& lhs, const Vector2<T>& rhs) noexcept
{
    return {lhs.x + rhs.x, lhs.y + rhs.y};
}

template <typename T>
[[nodiscard]] constexpr Vector2<T> operator+(const Vector2<T>& vec, T scalar) noexcept
{
    return {vec.x + scalar, vec.y + scalar};
}

template <typename T>
[[nodiscard]] constexpr Vector2<T> operator+(T scalar, const Vector2<T>& vec) noexcept
{
    return {scalar + vec.x, scalar + vec.y};
}

template <typename T>
[[nodiscard]] constexpr Vector2<T> operator-(const Vector2<T>& lhs, const Vector2<T>& rhs) noexcept
{
    return {lhs.x - rhs.x, lhs.y - rhs.y};
}

template <typename T>
[[nodiscard]] constexpr Vector2<T> operator-(const Vector2<T>& vec, T scalar) noexcept
{
    return {vec.x - scalar, vec.y - scalar};
}

template <typename T>
[[nodiscard]] constexpr Vector2<T> operator-(T scalar, const Vector2<T>& vec) noexcept
{
    return {scalar - vec.x, scalar - vec.y};
}

template <typename T>
[[nodiscard]] constexpr Vector2<T> operator*(const Vector2<T>& vec, T scalar) noexcept
{
    return {vec.x * scalar, vec.y * scalar};
}

template <typename T>
[[nodiscard]] constexpr Vector2<T> operator*(T scalar, const Vector2<T>& vec) noexcept
{
    return {scalar * vec.x, scalar * vec.y};
}

template <typename T>
[[nodiscard]] constexpr Vector2<T> operator*(const Vector2<T>& lhs, const Vector2<T>& rhs) noexcept
{
    return {lhs.x * rhs.x, lhs.y * rhs.y};
}

template <typename T>
[[nodiscard]] constexpr Vector2<T> operator/(const Vector2<T>& vec, T scalar)
{
    if (std::abs(scalar) < Vector2<T>::epsilon)
    {
        throw std::runtime_error("Division by zero or near-zero value");
    }
    return {vec.x / scalar, vec.y / scalar};
}

template <typename T>
[[nodiscard]] constexpr Vector2<T> operator/(const Vector2<T>& lhs, const Vector2<T>& rhs)
{
    if (std::abs(rhs.x) < Vector2<T>::epsilon || std::abs(rhs.y) < Vector2<T>::epsilon)
    {
        throw std::runtime_error("Division by zero or near-zero value");
    }
    return {lhs.x / rhs.x, lhs.y / rhs.y};
}

template <typename T>
[[nodiscard]] constexpr bool operator==(const Vector2<T>& lhs, const Vector2<T>& rhs) noexcept
{
    return lhs.approximately_equals(rhs);
}

template <typename T>
[[nodiscard]] constexpr bool operator!=(const Vector2<T>& lhs, const Vector2<T>& rhs) noexcept
{
    return !(lhs == rhs);
}

template <typename T>
[[nodiscard]] constexpr bool operator<(const Vector2<T>& lhs, const Vector2<T>& rhs) noexcept
{
    return Vector2<T>::compare(lhs, rhs) < 0;
}

template <typename T>
[[nodiscard]] constexpr bool operator<=(const Vector2<T>& lhs, const Vector2<T>& rhs) noexcept
{
    return Vector2<T>::compare(lhs, rhs) <= 0;
}

template <typename T>
[[nodiscard]] constexpr bool operator>(const Vector2<T>& lhs, const Vector2<T>& rhs) noexcept
{
    return Vector2<T>::compare(lhs, rhs) > 0;
}

template <typename T>
[[nodiscard]] constexpr bool operator>=(const Vector2<T>& lhs, const Vector2<T>& rhs) noexcept
{
    return Vector2<T>::compare(lhs, rhs) >= 0;
}

template <typename T>
constexpr void swap(Vector2<T>& lhs, Vector2<T>& rhs) noexcept
{
    lhs.swap(rhs);
}

using Vector2f = Vector2<float>;
using Vector2d = Vector2<double>;
using Vector2i = Vector2<int>;
using Vector2u = Vector2<unsigned int>;
using Vector2i8 = Vector2<int8_t>;
using Vector2u8 = Vector2<uint8_t>;
using Vector2i16 = Vector2<int16_t>;
using Vector2u16 = Vector2<uint16_t>;
using Vector2i32 = Vector2<int32_t>;
using Vector2u32 = Vector2<uint32_t>;
using Vector2i64 = Vector2<int64_t>;
using Vector2u64 = Vector2<uint64_t>;

using Vec2f = Vector2f;
using Vec2d = Vector2d;
using Vec2i = Vector2i;
using Vec2u = Vector2u;

}  // namespace umath

namespace std
{
template <typename T>
struct hash<umath::Vector2<T>>
{
    constexpr std::size_t operator()(const umath::Vector2<T>& vec) const noexcept
    {
        return vec.hash();
    }
};

template <typename T>
struct tuple_size<umath::Vector2<T>> : std::integral_constant<std::size_t, 2>
{
};

template <std::size_t I, typename T>
struct tuple_element<I, umath::Vector2<T>>
{
    static_assert(I < 2, "Index out of bounds for Vector2");
    using type = T;
};

template <std::size_t I, typename T>
constexpr T& get(umath::Vector2<T>& vec) noexcept
{
    static_assert(I < 2, "Index out of bounds for Vector2");
    if constexpr (I == 0)
        return vec.x;
    else
        return vec.y;
}

template <std::size_t I, typename T>
constexpr const T& get(const umath::Vector2<T>& vec) noexcept
{
    static_assert(I < 2, "Index out of bounds for Vector2");
    if constexpr (I == 0)
        return vec.x;
    else
        return vec.y;
}

template <std::size_t I, typename T>
constexpr T&& get(umath::Vector2<T>&& vec) noexcept
{
    static_assert(I < 2, "Index out of bounds for Vector2");
    if constexpr (I == 0)
        return std::move(vec.x);
    else
        return std::move(vec.y);
}
}  // namespace std


#endif  // End of include guard: LIB_UMATH_VEC2_HPP_a5enmn