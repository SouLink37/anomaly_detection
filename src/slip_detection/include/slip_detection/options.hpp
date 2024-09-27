#ifndef SLIP_DETECTION__OPTIONS_HPP
#define SLIP_DETECTION__OPTIONS_HPP

#include <cstdint>

#include "slip_detection/pose.hpp"

namespace anomaly_detection
{
    struct Options
    {
        uint16_t period{1000};          //ms
        PoseDifference accepted_diff{3.0, 3.0};     //m, rad
        bool print_timestamps{false};
        bool use_lidar_odom{false};
        bool show_diff{false};
    };
} // namespace slip_detection


#endif // SLIP_DETECTION__OPTIONS_HPP
