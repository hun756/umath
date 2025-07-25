#include <umath/umath.hpp>

#include <iostream>
#include <limits>


using namespace umath;

int main()
{
    std::cout << "=== Special Value Handling ===" << std::endl;

    std::cout << "NaN handling:" << std::endl;
    double nan_val = std::numeric_limits<double>::quiet_NaN();
    std::cout << "sqrt(-1) is NaN: " << std::isnan(Math<double>::sqrt(-1.0)) << std::endl;
    std::cout << "atan(NaN) is NaN: " << std::isnan(Math<double>::atan(nan_val)) << std::endl;

    std::cout << "\nInfinity handling:" << std::endl;
    double inf = std::numeric_limits<double>::infinity();
    std::cout << "exp(inf) is inf: " << std::isinf(Math<double>::exp(inf)) << std::endl;
    std::cout << "atan(inf) = " << Math<double>::atan(inf) << std::endl;

    std::cout << "\nZero handling:" << std::endl;
    std::cout << "log10(0) is -inf: " << std::isinf(Math<double>::log10(0.0)) << std::endl;
    std::cout << "pow(0, 0) = " << Math<double>::pow(0.0, 0.0) << std::endl;

    return 0;
}
