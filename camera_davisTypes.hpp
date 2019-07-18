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
        ::base::Time time;
        unsigned int height;
        unsigned int width;

        std::vector<Event> events;
    };


    struct DeviceConfig
    {
        int streaming_rate;
        unsigned int max_events;
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

    struct HardwareFilters
    {
        bool background_activity_filter_enabled;
        int background_activity_filter_time;
        bool refractory_period_enabled;
        int refractory_period_time;
        bool skip_enabled;
        int skip_every;
        bool polarity_suppress;
        bool polarity_suppress_type;
        bool polarity_flatten;

        bool state;
        std::string name;
    };

    struct DVSRegionOfInterest
    {
        int roi_start_column;
        int roi_start_row;
        int roi_end_column;
        int roi_end_row;

        bool state;
        std::string name;
    };

    struct APSRegionOfInterest
    {
        int aps_roi_start_column;
        int aps_roi_start_row;
        int aps_roi_end_column;
        int aps_roi_end_row;

        bool state;
        std::string name;
    };

    struct PixelFilter
    {
        int pixel_0_row;
        int pixel_0_column;
        int pixel_1_row;
        int pixel_1_column;
        int pixel_2_row;
        int pixel_2_column;
        int pixel_3_row;
        int pixel_3_column;
        int pixel_4_row;
        int pixel_4_column;
        int pixel_5_row;
        int pixel_5_column;
        int pixel_6_row;
        int pixel_6_column;
        int pixel_7_row;
        int pixel_7_column;
        bool pixel_auto_train;

        bool state;
        std::string name;
    };

    struct DavisBiasesStage1
    {
        int PrBp_coarse;
        int PrBp_fine;
        int PrSFBp_coarse;
        int PrSFBp_fine;

        bool state;
        std::string name;
    };

    struct DavisBiasesStage2
    {

        int DiffBn_coarse;
        int DiffBn_fine;
        int ONBn_coarse;
        int ONBn_fine;
        int OFFBn_coarse;
        int OFFBn_fine;
        int RefrBp_coarse;
        int RefrBp_fine;

        bool state;
        std::string name;
    };

    struct DavisBiasesAPS
    {
        int ADC_RefHigh_volt;
        int ADC_RefHigh_curr;
        int ADC_RefLow_volt;
        int ADC_RefLow_curr;

        bool state;
        std::string name;
    };

}

#endif

