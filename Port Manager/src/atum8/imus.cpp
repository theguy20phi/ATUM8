#include "imus.hpp"

namespace atum8
{
    Imus::Imus(const std::vector<int> &ports)
    {
        for (int port : ports)
            imus.push_back(pros::Imu(port));
    }

    double Imus::get_rotation()
    {
        double sum{0};
        for (pros::Imu imu : imus)
            sum += imu.get_rotation();
        return sum / imus.size();
    }

    void Imus::reset()
    {
        for (pros::Imu imu : imus)
            imu.reset();
        while (imus.back().is_calibrating())
            pros::delay(stdDelay);
    }

    void Imus::tare_rotation()
    {
        for (pros::Imu imu : imus)
            imu.tare_rotation();
    }
}