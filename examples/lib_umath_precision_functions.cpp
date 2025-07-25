#include <umath/umath.hpp>

#include <iostream>


using namespace umath;

int main()
{
    std::cout << "=== High Precision Functions ===" << std::endl;

    std::cout << "Small value exponentials:" << std::endl;
    double small = 0.001;
    std::cout << "exp(" << small << ") - 1 = " << Math<double>::exp(small) - 1.0 << std::endl;
    std::cout << "expm1(" << small << ") = " << Math<double>::expm1(small) << std::endl;

    std::cout << "\nSmall value logarithms:" << std::endl;
    std::cout << "log(1 + " << small << ") = " << std::log(1.0 + small) << std::endl;
    std::cout << "log1p(" << small << ") = " << Math<double>::log1p(small) << std::endl;

    std::cout << "\nCube root precision:" << std::endl;
    std::cout << "cbrt(8) = " << Math<double>::cbrt(8.0) << std::endl;
    std::cout << "cbrt(-27) = " << Math<double>::cbrt(-27.0) << std::endl;
    std::cout << "cbrt(0.125) = " << Math<double>::cbrt(0.125) << std::endl;

    return 0;
}
