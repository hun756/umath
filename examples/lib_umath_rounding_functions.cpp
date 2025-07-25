#include <umath/umath.hpp>

#include <iostream>


using namespace umath;

int main()
{
    std::cout << "=== Rounding Functions ===" << std::endl;

    double value = 3.7;
    std::cout << "Original value: " << value << std::endl;

    std::cout << "\nRounding operations:" << std::endl;
    std::cout << "ceil(3.7) = " << Math<double>::ceil(value) << std::endl;
    std::cout << "floor(3.7) = " << Math<double>::floor(value) << std::endl;
    std::cout << "round(3.7) = " << Math<double>::round(value) << std::endl;

    double negative = -2.3;
    std::cout << "\nNegative value: " << negative << std::endl;
    std::cout << "ceil(-2.3) = " << Math<double>::ceil(negative) << std::endl;
    std::cout << "floor(-2.3) = " << Math<double>::floor(negative) << std::endl;
    std::cout << "round(-2.3) = " << Math<double>::round(negative) << std::endl;

    std::cout << "\nSign function:" << std::endl;
    std::cout << "signum(5.0) = " << Math<double>::signum(5.0) << std::endl;
    std::cout << "signum(-3.0) = " << Math<double>::signum(-3.0) << std::endl;
    std::cout << "signum(0.0) = " << Math<double>::signum(0.0) << std::endl;

    return 0;
}
