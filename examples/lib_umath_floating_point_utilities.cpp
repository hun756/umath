#include <umath/umath.hpp>

#include <iostream>

using namespace umath;

int main()
{
    std::cout << "=== Floating Point Utilities ===" << std::endl;

    std::cout << "Exponent extraction:" << std::endl;
    std::cout << "getExponent(8.0) = " << Math<double>::getExponent(8.0) << std::endl;
    std::cout << "getExponent(0.5) = " << Math<double>::getExponent(0.5) << std::endl;
    std::cout << "getExponent(1.0) = " << Math<double>::getExponent(1.0) << std::endl;

    std::cout << "\nIEEE remainder:" << std::endl;
    std::cout << "IEEEremainder(7.0, 3.0) = " << Math<double>::IEEEremainder(7.0, 3.0) << std::endl;
    std::cout << "IEEEremainder(-7.0, 3.0) = " << Math<double>::IEEEremainder(-7.0, 3.0)
              << std::endl;

    std::cout << "\nScaling operations:" << std::endl;
    std::cout << "scalb(2.0, 3) = " << Math<double>::scalb(2.0, 3) << std::endl;
    std::cout << "scalb(1.5, -2) = " << Math<double>::scalb(1.5, -2) << std::endl;

    std::cout << "\nNext representable value:" << std::endl;
    double val = 1.0;
    double next = Math<double>::nextAfter(val, 2.0);
    std::cout << "nextAfter(1.0, 2.0) = " << next << std::endl;
    std::cout << "Difference: " << (next - val) << std::endl;

    return 0;
}
