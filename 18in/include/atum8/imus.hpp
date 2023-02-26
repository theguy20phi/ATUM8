/**
 * @file imus.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief Provides a simple interface to combine several imus into
 * one.
 * @version 0.1
 * @date 2023-02-25
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include "constants.hpp"

namespace atum8
{
    /**
     * @brief Provides a simple interface to combine several imus into
     * one.
     *
     */
    class Imus
    {
    public:
        /**
         * @brief Constructs a new Imus object.
         *
         * @param ports
         */
        Imus(const std::vector<int> &ports);

        /**
         * @brief Gets the average rotation of all the imus.
         *
         * @return double
         */
        double get_rotation();

        /**
         * @brief Calibrates the imus and blocks.
         *
         */
        void reset();

        /**
         * @brief Sets the rotation of each imu to zero.
         *
         */
        void tare_rotation();

    private:
        std::vector<pros::Imu> imus;
    };

    using UPImus = std::unique_ptr<Imus>;
    using SPImus = std::shared_ptr<Imus>;
}