#include <umath/umath.hpp>

#include <iostream>


using namespace umath;

int main()
{
    std::cout << "=== Basic Arithmetic with Overflow Protection ===" << std::endl;

    try
    {
        int result = Math<int>::addExact(2'147'483'647, 1);
        std::cout << "Result: " << result << std::endl;
    }
    catch (const arithmetic_overflow& e)
    {
        std::cout << "Overflow detected: " << e.what() << std::endl;
    }

    std::cout << "Safe operations:" << std::endl;
    std::cout << "100 + 200 = " << Math<int>::addExact(100, 200) << std::endl;
    std::cout << "500 - 300 = " << Math<int>::subtractExact(500, 300) << std::endl;
    std::cout << "25 * 4 = " << Math<int>::multiplyExact(25, 4) << std::endl;

    std::cout << "\nAbsolute values:" << std::endl;
    std::cout << "abs(-42) = " << Math<int>::abs(-42) << std::endl;
    std::cout << "abs(-3.14) = " << Math<float>::abs(-3.14f) << std::endl;

    std::cout << "\nMin/Max operations:" << std::endl;
    std::cout << "max(15, 23) = " << Math<int>::max(15, 23) << std::endl;
    std::cout << "min(7.5, 4.2) = " << Math<double>::min(7.5, 4.2) << std::endl;

    return 0;
}
