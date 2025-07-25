#include <umath/umath.hpp>

#include <iostream>


using namespace umath;

int main()
{
    std::cout << "=== Trigonometric Functions ===" << std::endl;

    float angle_deg = 45.0f;
    float angle_rad = Math<float>::toRadians(angle_deg);

    std::cout << "Angle: " << angle_deg << "° = " << angle_rad << " rad" << std::endl;

    std::cout << "\nBasic trig functions:" << std::endl;
    std::cout << "sin(45°) = " << Math<float>::sin(angle_rad) << std::endl;
    std::cout << "cos(45°) = " << Math<float>::cos(angle_rad) << std::endl;
    std::cout << "tan(45°) = " << Math<float>::tan(angle_rad) << std::endl;

    std::cout << "\nHyperbolic functions:" << std::endl;
    std::cout << "sinh(1.0) = " << Math<double>::sinh(1.0) << std::endl;
    std::cout << "cosh(1.0) = " << Math<double>::cosh(1.0) << std::endl;
    std::cout << "tanh(1.0) = " << Math<double>::tanh(1.0) << std::endl;

    std::cout << "\nInverse trig:" << std::endl;
    std::cout << "atan(1.0) = " << Math<double>::atan(1.0) << " rad" << std::endl;
    std::cout << "atan(1.0) = " << Math<double>::toDegrees(Math<double>::atan(1.0)) << "°"
              << std::endl;

    return 0;
}
