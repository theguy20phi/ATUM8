#include "potentiometer.hpp"

namespace atum8
{
    Potentiometer::Potentiometer(uint8_t smartPort,
                                 uint8_t adiPort,
                                 const std::vector<double> &iPositionMap,
                                 int medFilterSize) : potentiometer{{smartPort, adiPort}},
                                                      positionMap{iPositionMap},
                                                      filter{medFilterSize}
    {
    }

    Potentiometer::Potentiometer(uint8_t port,
                                 const std::vector<double> &iPositionMap,
                                 int medFilterSize) : potentiometer{port},
                                                      positionMap{iPositionMap},
                                                      filter{medFilterSize}
    {
    }

    double Potentiometer::getPosition()
    {
        return filter.get(potentiometer.get_value());
    }

    int Potentiometer::getMappedPosition()
    {
        std::vector<double> distances = positionMap;
        const double currentPos{getPosition()};
        std::transform(distances.begin(), distances.end(), distances.begin(), [currentPos](double setPos)
                       { return std::abs(setPos - currentPos); });
        auto smallestDistance = std::min_element(distances.begin(), distances.end());
        return std::distance(distances.begin(), smallestDistance);
    }
}