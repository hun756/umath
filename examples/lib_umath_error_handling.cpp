#include <umath/umath.hpp>

#include <iostream>
#include <stdexcept>


using namespace umath;

int main()
{
    std::cout << "=== Error Handling ===" << std::endl;

    std::cout << "Arithmetic overflow protection:" << std::endl;
    try
    {
        int max_int = std::numeric_limits<int>::max();
        Math<int>::addExact(max_int, 1);
    }
    catch (const arithmetic_overflow& e)
    {
        std::cout << "Caught overflow: " << e.what() << std::endl;
    }

    std::cout << "\nDivision by zero protection:" << std::endl;
    try
    {
        Math<int>::floorDiv(10, 0);
    }
    catch (const std::domain_error& e)
    {
        std::cout << "Caught division by zero: " << e.what() << std::endl;
    }

    std::cout << "\nInteger overflow in division:" << std::endl;
    try
    {
        int min_int = std::numeric_limits<int>::min();
        Math<int>::floorDiv(min_int, -1);
    }
    catch (const std::domain_error& e)
    {
        std::cout << "Caught integer overflow: " << e.what() << std::endl;
    }

    std::cout << "\nSafe operations continue normally:" << std::endl;
    std::cout << "addExact(100, 200) = " << Math<int>::addExact(100, 200) << std::endl;
    std::cout << "floorDiv(15, 4) = " << Math<int>::floorDiv(15, 4) << std::endl;

    return 0;
}
