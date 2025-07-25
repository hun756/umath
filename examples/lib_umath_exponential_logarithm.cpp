#include <umath/umath.hpp>

#include <iostream>


using namespace umath;

int main()
{
    std::cout << "=== Exponential and Logarithmic Functions ===" << std::endl;

    std::cout << "Exponential functions:" << std::endl;
    std::cout << "exp(1.0) = " << Math<double>::exp(1.0) << std::endl;
    std::cout << "exp(2.0) = " << Math<double>::exp(2.0) << std::endl;
    std::cout << "expm1(0.1) = " << Math<double>::expm1(0.1) << std::endl;

    std::cout << "\nLogarithmic functions:" << std::endl;
    std::cout << "log10(100) = " << Math<double>::log10(100.0) << std::endl;
    std::cout << "log10(1000) = " << Math<double>::log10(1000.0) << std::endl;
    std::cout << "log1p(0.1) = " << Math<double>::log1p(0.1) << std::endl;

    std::cout << "\nPower functions:" << std::endl;
    std::cout << "pow(2, 3) = " << Math<double>::pow(2.0, 3.0) << std::endl;
    std::cout << "pow(4, 0.5) = " << Math<double>::pow(4.0, 0.5) << std::endl;
    std::cout << "cbrt(27) = " << Math<double>::cbrt(27.0) << std::endl;

    std::cout << "\nSquare roots:" << std::endl;
    std::cout << "sqrt(16) = " << Math<float>::sqrt(16.0f) << std::endl;
    std::cout << "rsqrt(4) = " << Math<float>::rsqrt(4.0f) << std::endl;
    std::cout << "hypot(3, 4) = " << Math<float>::hypot(3.0f, 4.0f) << std::endl;

    return 0;
}
