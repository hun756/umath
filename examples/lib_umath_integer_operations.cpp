#include <umath/umath.hpp>

#include <iostream>


using namespace umath;

int main()
{
    std::cout << "=== Integer Operations ===" << std::endl;

    std::cout << "Floor division and modulo:" << std::endl;
    std::cout << "floorDiv(7, 3) = " << Math<int>::floorDiv(7, 3) << std::endl;
    std::cout << "floorMod(7, 3) = " << Math<int>::floorMod(7, 3) << std::endl;

    std::cout << "\nNegative numbers:" << std::endl;
    std::cout << "floorDiv(-7, 3) = " << Math<int>::floorDiv(-7, 3) << std::endl;
    std::cout << "floorMod(-7, 3) = " << Math<int>::floorMod(-7, 3) << std::endl;

    std::cout << "\nMixed signs:" << std::endl;
    std::cout << "floorDiv(7, -3) = " << Math<int>::floorDiv(7, -3) << std::endl;
    std::cout << "floorMod(7, -3) = " << Math<int>::floorMod(7, -3) << std::endl;

    try
    {
        Math<int>::floorDiv(10, 0);
    }
    catch (const std::domain_error& e)
    {
        std::cout << "\nDivision by zero caught: " << e.what() << std::endl;
    }

    return 0;
}
