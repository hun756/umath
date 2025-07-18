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
              >
    requires Arithmetic<ValueType> && std::convertible_to<ValueType, T>
#else
              ,
              typename = std::enable_if_t<detail::numeric_traits<ValueType>::is_valid
                                          && std::is_convertible_v<ValueType, T>>>
#endif
    class ImmutableVector2
    {
    private:
        const ValueType x_, y_;

    public:
        using value_type = ValueType;
        using const_reference = const ValueType&;

        template <typename X,
                  typename Y
#ifndef MATH_CONCEPTS_ENABLED
                  ,
                  typename = std::enable_if_t<
                      detail::numeric_traits<X>::is_valid && detail::numeric_traits<Y>::is_valid
                      && std::is_convertible_v<X, ValueType> && std::is_convertible_v<Y, ValueType>>
#endif
                  >
#ifdef MATH_CONCEPTS_ENABLED
        requires Arithmetic<X> && Arithmetic<Y> && std::convertible_to<X, ValueType>
                     && std::convertible_to<Y, ValueType>
#endif
        constexpr ImmutableVector2(X&& x_val, Y&& y_val) noexcept
            : x_{static_cast<ValueType>(std::forward<X>(x_val))},
              y_{static_cast<ValueType>(std::forward<Y>(y_val))}
        {
        }

        template <typename U
#ifdef MATH_CONCEPTS_ENABLED
                  >
        requires Arithmetic<U>
                     && std::convertible_to<U, ValueType>
#else
                  ,
                  typename = std::enable_if_t<detail::numeric_traits<U>::is_valid
                                              && std::is_convertible_v<U, ValueType>>
#endif
                            >
                 constexpr explicit ImmutableVector2(const Vector2<U>& other) noexcept
            : x_{static_cast<ValueType>(other.x)},
        y_{static_cast<ValueType>(other.y)}
        {
        }

        template <typename U
#ifndef MATH_CONCEPTS_ENABLED
                  ,
                  typename = std::enable_if_t<detail::numeric_traits<U>::is_valid
                                              && std::is_convertible_v<U, ValueType>>
#endif
                  >
#ifdef MATH_CONCEPTS_ENABLED
        requires Arithmetic<U> && std::convertible_to<U, ValueType>
#endif
        constexpr explicit ImmutableVector2(const ImmutableVector2<U>& other) noexcept
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

        template <typename U = T
#ifndef MATH_CONCEPTS_ENABLED
                  ,
                  typename = std::enable_if_t<detail::numeric_traits<U>::is_valid
                                              && std::is_convertible_v<ValueType, U>>
#endif
                  >
#ifdef MATH_CONCEPTS_ENABLED
        requires Arithmetic<U> && std::convertible_to<ValueType, U>
#endif
        [[nodiscard]] constexpr operator Vector2<U>() const noexcept
        {
            return {static_cast<U>(x_), static_cast<U>(y_)};
        }

        template <typename U
#ifndef MATH_CONCEPTS_ENABLED
                  ,
                  typename = std::enable_if_t<detail::numeric_traits<U>::is_valid>
#endif
                  >
#ifdef MATH_CONCEPTS_ENABLED
        requires Arithmetic<U>
#endif
        [[nodiscard]] constexpr auto as() const noexcept -> ImmutableVector2<U>
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

#ifdef MATH_CONCEPTS_ENABLED
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
            static const ImmutableVector2<U> zero_typed{U{}, U{}};
            return zero_typed;
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
            static const ImmutableVector2<U> one_typed{U{1}, U{1}};
            return one_typed;
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
            static const ImmutableVector2<U> neg_one_typed{U{-1}, U{-1}};
            return neg_one_typed;
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
            static const ImmutableVector2<U> unit_x_typed{U{1}, U{}};
            return unit_x_typed;
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
            static const ImmutableVector2<U> unit_y_typed{U{}, U{1}};
            return unit_y_typed;
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
            static const ImmutableVector2<U> down_typed{U{}, U{-1}};
            return down_typed;
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
            static const ImmutableVector2<U> left_typed{U{-1}, U{}};
            return left_typed;
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
            static const ImmutableVector2<U> right_typed{U{1}, U{}};
            return right_typed;
        }
    }

    template <typename U = T
#ifndef MATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<std::is_floating_point_v<U>>
#endif
              >
#ifdef MATH_CONCEPTS_ENABLED
    requires FloatingPoint<U>
#endif
    [[nodiscard]] static constexpr auto infinity_immutable() noexcept -> const ImmutableVector2<U>&
    {
        static const ImmutableVector2<U> infinity_typed{std::numeric_limits<U>::infinity(),
                                                        std::numeric_limits<U>::infinity()};
        return infinity_typed;
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

    template <typename OutputVector = Vector2
#ifndef MATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef MATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static constexpr OutputVector add(const Vector2& a,
                                      const Vector2& b,
                                      OutputVector* out = nullptr) noexcept
    {
        const auto result = OutputVector{a.x + b.x, a.y + b.y};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2
#ifndef MATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef MATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static constexpr OutputVector add_scalar(const Vector2& a,
                                             T scalar,
                                             OutputVector* out = nullptr) noexcept
    {
        const auto result = OutputVector{a.x + scalar, a.y + scalar};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2
#ifndef MATH_CONCEPTS_ENABLED
              ,
              std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>* = nullptr
#endif
              >
#ifdef MATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static constexpr OutputVector subtract(const Vector2& a,
                                           const Vector2& b,
                                           OutputVector* out = nullptr) noexcept
    {
        const auto result = OutputVector{a.x - b.x, a.y - b.y};
        if (out)
            *out = result;
        return result;
    }

    template <typename OutputVector = Vector2
#ifndef MATH_CONCEPTS_ENABLED
              ,
              std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>* = nullptr
#endif
              >
#ifdef MATH_CONCEPTS_ENABLED
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
#ifndef MATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef MATH_CONCEPTS_ENABLED
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
#ifndef MATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef MATH_CONCEPTS_ENABLED
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
#ifndef MATH_CONCEPTS_ENABLED
              ,
              std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>* = nullptr
#endif
              >
#ifdef MATH_CONCEPTS_ENABLED
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
#ifndef MATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef MATH_CONCEPTS_ENABLED
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
#ifndef MATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef MATH_CONCEPTS_ENABLED
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
#ifndef MATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef MATH_CONCEPTS_ENABLED
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
#ifndef MATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef MATH_CONCEPTS_ENABLED
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
#ifndef MATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef MATH_CONCEPTS_ENABLED
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
#ifndef MATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef MATH_CONCEPTS_ENABLED
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

    [[nodiscard]] constexpr precision_type fast_length() const noexcept
    {
        const auto min_comp = std::min(std::abs(x), std::abs(y));
        const auto max_comp = std::max(std::abs(x), std::abs(y));
        return static_cast<precision_type>(max_comp + precision_type{0.3} * min_comp);
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

    static constexpr precision_type fast_length(const Vector2& a) noexcept
    {
        return a.fast_length();
    }

    template <typename OutputVector = Vector2
#ifndef MATH_CONCEPTS_ENABLED
              ,
              std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>* = nullptr
#endif
              >
#ifdef MATH_CONCEPTS_ENABLED
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
#ifndef MATH_CONCEPTS_ENABLED
              ,
              typename = std::enable_if_t<detail::is_vector2_like_v<OutputVector, T>>
#endif
              >
#ifdef MATH_CONCEPTS_ENABLED
    requires Vector2Like<OutputVector, T>
#endif
    static constexpr OutputVector normalize_fast(const Vector2& a, OutputVector* out = nullptr)
    {
        if constexpr (std::is_floating_point_v<T>)
        {
            const auto len_sq = a.length_squared();
            if (len_sq < epsilon * epsilon)
            {
                throw std::runtime_error("Cannot normalize zero-length vector");
            }
            const auto inv_len = detail::fast_inverse_sqrt(len_sq);
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

    static constexpr precision_type distance_fast(const Vector2& a, const Vector2& b) noexcept
    {
        const auto dx = std::abs(a.x - b.x);
        const auto dy = std::abs(a.y - b.y);
        const auto min_d = std::min(dx, dy);
        const auto max_d = std::max(dx, dy);
        return static_cast<precision_type>(max_d + precision_type{0.3} * min_d);
    }

    static constexpr T manhattan_distance(const Vector2& a, const Vector2& b) noexcept
    {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }

    static constexpr T chebyshev_distance(const Vector2& a, const Vector2& b) noexcept
    {
        return std::max(std::abs(a.x - b.x), std::abs(a.y - b.y));
    }

    // --
    T x, y;
};

}  // namespace umath

#endif  // End of include guard: LIB_UMATH_VEC2_HPP_a5enmn