#ifndef camera_davis_TYPES_HPP
#define camera_davis_TYPES_HPP


#include <base/Time.hpp>

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace camera_davis
{
    struct Event
    {
        unsigned int x;
        unsigned int y;
        ::base::Time ts;
        bool polarity;
    };

    struct EventArray
    {
        ::base::Time ts;
        unsigned int height;
        unsigned int width;

        std::vector<Event> events;
    };


    struct DriverConfig
    {
        int streaming_rate;
        int max_events;
        bool aps_enabled;
        bool dvs_enabled;
        bool imu_enabled;
        bool autoexposure_enabled;
        int autoexposure_gain;
        int autoexposure_desired_intensity;
        int exposure;
        int frame_mode;
        int frame_interval;
        int imu_acc_scale;
        int imu_gyro_scale;
        int imu_low_pass_filter;

        bool state;
        std::string name;
    };
}

#endif

