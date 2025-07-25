#include <umath/umath.hpp>

#include <chrono>
#include <iostream>


using namespace umath;

int main()
{
    std::cout << "=== Fast Math Operations ===" << std::endl;

    std::cout << "Fast square root:" << std::endl;
    std::cout << "sqrt(16) = " << Math<float>::sqrt(16.0f) << std::endl;
    std::cout << "rsqrt(4) = " << Math<float>::rsqrt(4.0f) << std::endl;

    std::cout << "\nOptimized power functions:" << std::endl;
    std::cout << "pow(2, 2) = " << Math<float>::pow(2.0f, 2.0f) << std::endl;
    std::cout << "pow(8, 0.5) = " << Math<float>::pow(8.0f, 0.5f) << std::endl;
    std::cout << "pow(27, 3) = " << Math<float>::pow(27.0f, 3.0f) << std::endl;

    std::cout << "\nSmall angle approximations:" << std::endl;
    float small_angle = 0.01f;
    std::cout << "sin(" << small_angle << ") = " << Math<float>::sin(small_angle) << std::endl;
    std::cout << "cos(" << small_angle << ") = " << Math<float>::cos(small_angle) << std::endl;
    std::cout << "tan(" << small_angle << ") = " << Math<float>::tan(small_angle) << std::endl;

    std::cout << "\nFast exponential for small values:" << std::endl;
    std::cout << "exp(" << small_angle << ") = " << Math<float>::exp(small_angle) << std::endl;

    return 0;
}
