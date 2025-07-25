#include <umath/umath.hpp>

#include <iostream>


using namespace umath;

int main()
{
    std::cout << "=== Angle Conversions ===" << std::endl;

    std::cout << "Degrees to radians:" << std::endl;
    std::cout << "0° = " << Math<double>::toRadians(0.0) << " rad" << std::endl;
    std::cout << "90° = " << Math<double>::toRadians(90.0) << " rad" << std::endl;
    std::cout << "180° = " << Math<double>::toRadians(180.0) << " rad" << std::endl;
    std::cout << "360° = " << Math<double>::toRadians(360.0) << " rad" << std::endl;

    std::cout << "\nRadians to degrees:" << std::endl;
    std::cout << "π/6 rad = " << Math<double>::toDegrees(3.14159 / 6) << "°" << std::endl;
    std::cout << "π/4 rad = " << Math<double>::toDegrees(3.14159 / 4) << "°" << std::endl;
    std::cout << "π/2 rad = " << Math<double>::toDegrees(3.14159 / 2) << "°" << std::endl;
    std::cout << "π rad = " << Math<double>::toDegrees(3.14159) << "°" << std::endl;

    std::cout << "\nCommon angles in both units:" << std::endl;
    double angles_deg[] = {30, 45, 60, 90, 120, 135, 150, 180};
    for (double deg : angles_deg)
    {
        double rad = Math<double>::toRadians(deg);
        std::cout << deg << "° = " << rad << " rad" << std::endl;
    }

    return 0;
}
