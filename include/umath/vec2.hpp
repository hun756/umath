#ifndef LIB_UMATH_VEC2_HPP_a5enmn
#define LIB_UMATH_VEC2_HPP_a5enmn

#include <cmath>
#include <type_traits>
#include <umath/umath.hpp>

namespace umath
{
namespace detail
{

template <typename T>
struct numeric_traits
{
    static constexpr bool is_valid = std::is_arithmetic_v<T> &&
                                     !std::is_same_v<T, bool> &&
                                     !std::is_same_v<T, char>;
    using precision_type =
        std::conditional_t<std::is_floating_point_v<T>, T, double>;
    static constexpr T epsilon =
        std::is_floating_point_v<T> ? std::numeric_limits<T>::epsilon() * T{100}
                                    : T{};
};

template <typename T, typename ValueType = void>
struct is_vector2_like_impl
{
    template <typename U>
    static auto test(U* p)
        -> decltype(p->x, p->y,
                    std::is_assignable_v<decltype(p->x)&, ValueType>,
                    std::is_assignable_v<decltype(p->y)&, ValueType>,
                    std::true_type{});
    static std::false_type test(...);

    static constexpr bool value =
        decltype(test(static_cast<T*>(nullptr)))::value;
};

template <typename T>
struct is_vector2_like_impl<T, void>
{
    template <typename U>
    static auto test(U* p) -> decltype(p->x, p->y, std::true_type{});
    static std::false_type test(...);

    static constexpr bool value =
        decltype(test(static_cast<T*>(nullptr)))::value;
};

template <typename T, typename ValueType = void>
inline constexpr bool is_vector2_like_v =
    is_vector2_like_impl<T, ValueType>::value;

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
            conv.i = 0x5f3759df - (conv.i >> 1);
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

} // namespace detail
} // namespace umath

#endif // End of include guard: LIB_UMATH_VEC2_HPP_a5enmn