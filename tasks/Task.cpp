/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace camera_davis;

//DAVIS Bias types
#define CF_N_TYPE(COARSE, FINE) (struct caer_bias_coarsefine) \
{ .coarseValue = (uint8_t)(COARSE), .fineValue = (uint8_t)(FINE), \
    .enabled = true, .sexN = true, \
    .typeNormal = true, .currentLevelNormal = true }

#define CF_P_TYPE(COARSE, FINE) (struct caer_bias_coarsefine) \
{ .coarseValue = (uint8_t)(COARSE), .fineValue = (uint8_t)(FINE), \
    .enabled = true, .sexN = false, \
    .typeNormal = true, .currentLevelNormal = true }

#define SHIFTSOURCE(REF, REG, OPMODE) (struct caer_bias_shiftedsource) \
{ .refValue = (uint8_t)(REF), .regValue = (uint8_t)(REG), \
    .operatingMode = (caer_bias_shiftedsource_operating_mode)(OPMODE), .voltageLevel = (caer_bias_shiftedsource_voltage_level)(SPLIT_GATE) }

#define VDAC(VOLT, CURR) (struct caer_bias_vdac) \
{ .voltageValue = (uint8_t)(VOLT), .currentValue = (uint8_t)(CURR) }



Task::Task(std::string const& name)
    : TaskBase(name)
{
    this->device_is_running = false;
    this->imu_calibration_running = false;
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    /** Get the configuration values **/
    this->device_config = _device_config.value();
    this->hardware_filters = _hardware_filters.value();
    this->dvs_region_interest = _dvs_region_interest.value();
    this->aps_region_interest = _aps_region_interest.value();
    this->pixel_filter = _pixel_filter.value();
    this->davis_biases_1 = _davis_biases_1.value();
    this->davis_biases_2 = _davis_biases_2.value();
    this->davis_biases_aps = _davis_biases_aps.value();

    /** We need to connect to the camera first **/
    bool is_connected =  this->connect();

    this->delta = boost::posix_time::microseconds(1e6/this->device_config.streaming_rate);


    /** Set the period to the TaskContext **/
    TaskContext::setPeriod(1.0/this->device_config.streaming_rate);
    RTT::log(RTT::Info)<<"Connection result: "<<is_connected<<" - Set TaskContext period to: "<<1.0/this->device_config.streaming_rate<<"[seconds]"<<RTT::endlog();

    /** configure we set the configuration (after we connect) **/
    this->configureDevice();

    this->imu_calibration_running = true;
    this->imu_calibration_samples.clear();

    return is_connected;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    std::cout<<"*********START UPDATE HOOK ***********\n";
    this->readout();
    std::cout<<"*********END UPDATE HOOK ***********\n";
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    caerLog(CAER_LOG_INFO, "Exiting from DAVIS camera driver",  "executing data stop");
    RTT::log(RTT::Info)<<"Exiting from DAVIS camera driver, executing data stop"<<RTT::endlog();
    caerLog(CAER_LOG_ERROR, "stopHook",  "data stop now");
    caerDeviceDataStop(this->davis_handle);
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    caerLog(CAER_LOG_INFO, "Exiting from DAVIS camera driver",  "executing driver cleanup(close)");
    RTT::log(RTT::Info)<<"Exiting from DAVIS camera driver, executing driver cleanup(close)"<<RTT::endlog();
    caerDeviceClose(&(this->davis_handle));
    this->device_is_running = false;
    TaskBase::cleanupHook();
}

bool Task::connect()
{
    this->device_is_running = false;

    /** Seria number as a pointer to char **/
    const char* serial_number_restrict = (_serial_number.value() == "") ? NULL : _serial_number.value().c_str();

    if(serial_number_restrict)
    {
      RTT::log(RTT::Info)<<"Requested serial number: "<< _serial_number.value()<<RTT::endlog();
    }

    this->davis_handle = caerDeviceOpen(1, CAER_DEVICE_DAVIS, 0, 0, serial_number_restrict);

    // was opening successful?
    this->device_is_running = !(this->davis_handle == NULL);

    if (!this->device_is_running)
    {
        RTT::log(RTT::Warning)<<"Could not find DAVIS. Will retry every second."<<RTT::endlog();
        return false;
    }

    this->davis_info = caerDavisInfoGet(this->davis_handle);
    this->device_id = "DAVIS-" + std::string(this->davis_info.deviceString).substr(14, 8);

    RTT::log(RTT::Info)<<this->davis_info.deviceString<<" --- ID:"<<this->davis_info.deviceID<<
        ", Master:"<<this->davis_info.deviceIsMaster <<", DVS X:"<<this->davis_info.dvsSizeX <<", DVS Y:"<<this->davis_info.dvsSizeY<<
        ", Logic: "<< this->davis_info.logicVersion<<RTT::endlog();


    // Send the default configuration before using the device.
    // No configuration is sent automatically!
    caerDeviceSendDefaultConfig(this->davis_handle);

    // In case autoexposure is enabled, initialize the exposure time with the exposure value
    // from the parameter server
    if(this->device_config.autoexposure_enabled)
    {
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, this->device_config.exposure);
    }

    /** wait for driver to be ready **/
    sleep(0.6);

    this->resetTimestamps();

    return true;
}

void Task::resetTimestamps()
{
    caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RESET, 1);
    this->reset_time = base::Time::now();

    RTT::log(RTT::Info)<<"Reset timestamps on "<< this->device_id.c_str()<<" to "<< reset_time.toString()<<RTT::endlog();

    // if master, publish reset time to slaves
    if (_master.value())
    {
        //std_msgs::Time reset_msg;
        //reset_msg.data = reset_time_;
        //reset_pub_.publish(reset_msg);
    }
    return;
}

void Task::configureDevice()
{
    if(!this->device_config.autoexposure_enabled)
    {
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, this->device_config.exposure);
    }

    caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_MODE, this->device_config.frame_mode);
    caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_INTERVAL, this->device_config.frame_interval);

    caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, this->device_config.aps_enabled);
    caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN, this->device_config.dvs_enabled);
    caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_ACCELEROMETER, this->device_config.imu_enabled);
    caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_GYROSCOPE, this->device_config.imu_enabled);
    caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_TEMPERATURE, this->device_config.imu_enabled);

    if (this->device_config.imu_gyro_scale >= 0 && this->device_config.imu_gyro_scale <= 3)
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_GYRO_FULL_SCALE, this->device_config.imu_gyro_scale);

    if (this->device_config.imu_acc_scale >= 0 && this->device_config.imu_acc_scale <= 3)
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ACCEL_FULL_SCALE, this->device_config.imu_acc_scale);

    if (this->device_config.imu_low_pass_filter >= 0 && this->device_config.imu_low_pass_filter <= 6)
    {
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_DIGITAL_LOW_PASS_FILTER, this->device_config.imu_low_pass_filter);

            if(this->device_config.imu_low_pass_filter == 0)
            {
              // When the low pass filter is disabled, the output frequency of IMU events
              // is raised to 8KHz. To keep it to 1 kHz, we use the sample rate divider
              // (setting its value to 7 divides the frequency by 8).
              caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER, 7);
            }
            else
            {
              // When the low pass filter is enabled, the gyroscope output rate is set to 1 kHz,
              // so we should not use the sample rate divider, in order to keep the IMU output rate to 1 kHz.
              caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER, 0);
            }
    }
    if (this->davis_info.chipID == DAVIS_CHIP_DAVIS346B)
    {
        // VDAC
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_APSOVERFLOWLEVEL,
                            caerBiasVDACGenerate(VDAC(27,6)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_APSCAS,
                            caerBiasVDACGenerate(VDAC(21,6)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ADCREFHIGH,
                            caerBiasVDACGenerate(VDAC(this->davis_biases_aps.ADC_RefHigh_volt, this->davis_biases_aps.ADC_RefHigh_curr)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ADCREFLOW,
                            caerBiasVDACGenerate(VDAC(this->davis_biases_aps.ADC_RefLow_volt, this->davis_biases_aps.ADC_RefLow_curr)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ADCTESTVOLTAGE,
                            caerBiasVDACGenerate(VDAC(21,7)));
        // CF Biases
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_LOCALBUFBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(5, 164)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_PADFOLLBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(7, 215)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_DIFFBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(this->davis_biases_2.DiffBn_coarse, this->davis_biases_2.DiffBn_fine)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ONBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(this->davis_biases_2.ONBn_coarse, this->davis_biases_2.ONBn_fine)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_OFFBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(this->davis_biases_2.OFFBn_coarse, this->davis_biases_2.OFFBn_fine)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_PIXINVBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(5, 129)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_PRBP,
                            caerBiasCoarseFineGenerate(CF_P_TYPE(this->davis_biases_1.PrBp_coarse, this->davis_biases_1.PrBp_fine)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_PRSFBP,
                            caerBiasCoarseFineGenerate(CF_P_TYPE(this->davis_biases_1.PrSFBp_coarse, this->davis_biases_1.PrSFBp_fine)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_REFRBP,
                            caerBiasCoarseFineGenerate(CF_P_TYPE(this->davis_biases_2.RefrBp_coarse, this->davis_biases_2.RefrBp_fine)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_READOUTBUFBP,
                            caerBiasCoarseFineGenerate(CF_P_TYPE(6, 20)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_APSROSFBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(6, 219)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ADCCOMPBP,
                            caerBiasCoarseFineGenerate(CF_P_TYPE(5, 20)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_COLSELLOWBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(0, 1)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_DACBUFBP,
                            caerBiasCoarseFineGenerate(CF_P_TYPE(6, 60)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_LCOLTIMEOUTBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(5, 49)));;
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_AEPDBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(6, 91)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_AEPUXBP,
                            caerBiasCoarseFineGenerate(CF_P_TYPE(4, 80)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_AEPUYBP,
                            caerBiasCoarseFineGenerate(CF_P_TYPE(7, 152)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_IFREFRBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_IFTHRBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_BIASBUFFER,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(5, 254)));
        // Special Biases
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_SSP,
                            caerBiasShiftedSourceGenerate(SHIFTSOURCE(1,33,SHIFTED_SOURCE)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_SSN,
                            caerBiasShiftedSourceGenerate(SHIFTSOURCE(1,33,SHIFTED_SOURCE)));

        // Hardware filters
        if (this->davis_info.dvsHasPixelFilter)
        {
          caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_AUTO_TRAIN, this->pixel_filter.pixel_auto_train);

          // if using auto train, update the configuration with hardware values
          if (this->pixel_filter.pixel_auto_train)
          {
            caerDeviceConfigGet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_ROW, (uint32_t*)&(this->pixel_filter).pixel_0_row);
            caerDeviceConfigGet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_COLUMN, (uint32_t*)&(this->pixel_filter).pixel_0_column);
            caerDeviceConfigGet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_ROW, (uint32_t*)&(this->pixel_filter).pixel_1_row);
            caerDeviceConfigGet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_COLUMN, (uint32_t*)&(this->pixel_filter).pixel_1_column);
            caerDeviceConfigGet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_ROW, (uint32_t*)&(this->pixel_filter).pixel_2_row);
            caerDeviceConfigGet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_COLUMN, (uint32_t*)&(this->pixel_filter).pixel_2_column);
            caerDeviceConfigGet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_ROW, (uint32_t*)&(this->pixel_filter).pixel_3_row);
            caerDeviceConfigGet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_COLUMN, (uint32_t*)&(this->pixel_filter).pixel_3_column);
            caerDeviceConfigGet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_ROW, (uint32_t*)&(this->pixel_filter).pixel_4_row);
            caerDeviceConfigGet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_COLUMN, (uint32_t*)&(this->pixel_filter).pixel_4_column);
            caerDeviceConfigGet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_ROW, (uint32_t*)&(this->pixel_filter).pixel_5_row);
            caerDeviceConfigGet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_COLUMN, (uint32_t*)&(this->pixel_filter).pixel_5_column);
            caerDeviceConfigGet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_ROW, (uint32_t*)&(this->pixel_filter).pixel_6_row);
            caerDeviceConfigGet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_COLUMN, (uint32_t*)&(this->pixel_filter).pixel_6_column);
            caerDeviceConfigGet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_ROW, (uint32_t*)&(this->pixel_filter).pixel_7_row);
            caerDeviceConfigGet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_COLUMN, (uint32_t*)&(this->pixel_filter).pixel_7_column);
          }
          else // apply current configuration to hardware
          {
            caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_ROW, this->pixel_filter.pixel_0_row);
            caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_COLUMN, this->pixel_filter.pixel_0_column);
            caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_ROW, this->pixel_filter.pixel_1_row);
            caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_COLUMN, this->pixel_filter.pixel_1_column);
            caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_ROW, this->pixel_filter.pixel_2_row);
            caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_COLUMN, this->pixel_filter.pixel_2_column);
            caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_ROW, this->pixel_filter.pixel_3_row);
            caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_COLUMN, this->pixel_filter.pixel_3_column);
            caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_ROW, this->pixel_filter.pixel_4_row);
            caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_COLUMN, this->pixel_filter.pixel_4_column);
            caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_ROW, this->pixel_filter.pixel_5_row);
            caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_COLUMN, this->pixel_filter.pixel_5_column);
            caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_ROW, this->pixel_filter.pixel_6_row);
            caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_COLUMN, this->pixel_filter.pixel_6_column);
            caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_ROW, this->pixel_filter.pixel_7_row);
            caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_COLUMN, this->pixel_filter.pixel_7_column);
          }
        }

        if (this->davis_info.dvsHasBackgroundActivityFilter)
        {
          caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY, this->hardware_filters.background_activity_filter_enabled);
          caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY_TIME, this->hardware_filters.background_activity_filter_time);

          caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD, this->hardware_filters.refractory_period_enabled);
          caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD_TIME, this->hardware_filters.refractory_period_time);
        }

        if (this->davis_info.dvsHasROIFilter)
        {
          caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_START_COLUMN, this->dvs_region_interest.roi_start_column);
          caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_START_ROW, this->dvs_region_interest.roi_start_row);
          caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_END_COLUMN, this->dvs_region_interest.roi_end_column);
          caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_END_ROW, this->dvs_region_interest.roi_end_row);
        }

        if (this->davis_info.dvsHasSkipFilter)
        {
          caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_SKIP_EVENTS, this->hardware_filters.skip_enabled);
          caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_SKIP_EVENTS_EVERY, this->hardware_filters.skip_every);
        }

        if (this->davis_info.dvsHasPolarityFilter)
        {
          caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_POLARITY_FLATTEN, this->hardware_filters.polarity_flatten);
          caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_POLARITY_SUPPRESS, this->hardware_filters.polarity_suppress);
          caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_POLARITY_SUPPRESS_TYPE, this->hardware_filters.polarity_suppress_type);
        }

        // APS region of interest
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_0, this->aps_region_interest.aps_roi_start_column);
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_0, this->aps_region_interest.aps_roi_start_row);
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_0, this->aps_region_interest.aps_roi_end_column);
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_0, this->aps_region_interest.aps_roi_end_row);
    }
    else
    {
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP,
                            caerBiasCoarseFineGenerate(CF_P_TYPE(this->davis_biases_1.PrBp_coarse, this->davis_biases_1.PrBp_fine)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP,
                            caerBiasCoarseFineGenerate(CF_P_TYPE(this->davis_biases_1.PrSFBp_coarse, this->davis_biases_1.PrSFBp_fine)));

        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_DIFFBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(this->davis_biases_2.DiffBn_coarse, this->davis_biases_2.DiffBn_fine)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_ONBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(this->davis_biases_2.ONBn_coarse, this->davis_biases_2.ONBn_fine)));
        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_OFFBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(this->davis_biases_2.OFFBn_coarse, this->davis_biases_2.OFFBn_fine)));

        caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_REFRBP,
                            caerBiasCoarseFineGenerate(CF_P_TYPE(this->davis_biases_2.RefrBp_coarse, this->davis_biases_2.RefrBp_fine)));

    }
}

void Task::updateIMUBias()
{
    this->bias.acc.setZero();
    this->bias.gyro.setZero();

    for (auto m : this->imu_calibration_samples)
    {
        this->bias.acc += m.acc;
        this->bias.gyro += m.gyro;
    }

    bias.acc /= (double) this->imu_calibration_samples.size();
    bias.acc[2] -= STANDARD_GRAVITY * sgn(bias.acc[2]);

    bias.gyro /= (double) this->imu_calibration_samples.size();

    RTT::log(RTT::Info)<<"IMU calibration done."<<RTT::endlog();
    RTT::log(RTT::Info)<<"Acceleration biases: "<< this->bias.acc[0] <<", "<< this->bias.acc[1] <<", "<< this->bias.acc[2] <<" [m/s^2]"<<RTT::endlog();
    RTT::log(RTT::Info)<<"Gyroscope biases: "<< this->bias.gyro[0] <<", "<< this->bias.gyro[1] <<", "<< this->bias.gyro[2] <<" [rad/s]"<<RTT::endlog();
}

void Task::readout()
{
    caerDeviceConfigSet(this->davis_handle, CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);
    boost::posix_time::ptime next_send_time = boost::posix_time::microsec_clock::local_time();

    caerEventPacketContainer packetContainer = caerDeviceDataGet(this->davis_handle);

    int32_t packetNum = caerEventPacketContainerGetEventPacketsNumber(packetContainer);

    for (int32_t i = 0; i < packetNum; i++)
    {
        caerEventPacketHeader packetHeader = caerEventPacketContainerGetEventPacket(packetContainer, i);
        if (packetHeader == NULL)
        {
            continue; // Skip if nothing there.
        }

        const int type = caerEventPacketHeaderGetEventType(packetHeader);

        // Packet 0 is always the special events packet for DVS128, while packet is the polarity events packet.
        if (type == POLARITY_EVENT)
        {
            this->event_array_msg.height = this->davis_info.dvsSizeY;
            this->event_array_msg.width = this->davis_info.dvsSizeX;

            caerPolarityEventPacket polarity = (caerPolarityEventPacket) packetHeader;

            const int numEvents = caerEventPacketHeaderGetEventNumber(packetHeader);
            for (int j = 0; j < numEvents; j++)
            {
                // Get full timestamp and addresses of first event.
                caerPolarityEvent event = caerPolarityEventPacketGetEvent(polarity, j);

                camera_davis::Event e;
                e.x = caerPolarityEventGetX(event);
                e.y = caerPolarityEventGetY(event);
                e.ts = this->reset_time +
                    ::base::Time::fromMicroseconds (caerPolarityEventGetTimestamp64(event, polarity));
                e.polarity = caerPolarityEventGetPolarity(event);

                if(j == 0)
                {
                    this->event_array_msg.time = e.ts;
                }

                this->event_array_msg.events.push_back(e);
            }

            // throttle event messages
            if (boost::posix_time::microsec_clock::local_time() > next_send_time ||
                    this->device_config.streaming_rate == 0 ||
                    (this->device_config.max_events != 0 && this->event_array_msg.events.size() > this->device_config.max_events)
                    )
            {
                _events.write(this->event_array_msg);

                if (this->device_config.streaming_rate > 0)
                {
                    next_send_time += this->delta;
                }
                if (this->device_config.max_events != 0 && this->event_array_msg.events.size() > this->device_config.max_events)
                {
                    next_send_time = boost::posix_time::microsec_clock::local_time() + this->delta;
                }

                this->event_array_msg.events.clear();
            }
        }
        else if (type == IMU6_EVENT)
        {
            caerIMU6EventPacket imu = (caerIMU6EventPacket) packetHeader;

            const int numEvents = caerEventPacketHeaderGetEventNumber(packetHeader);

            for (int j = 0; j < numEvents; j++)
            {
                caerIMU6Event event = caerIMU6EventPacketGetEvent(imu, j);

                // convert from g's to m/s^2 and align axes with camera frame
                this->imu_msg.acc[0] = -caerIMU6EventGetAccelX(event) * STANDARD_GRAVITY;
                this->imu_msg.acc[1] = caerIMU6EventGetAccelY(event) * STANDARD_GRAVITY;
                this->imu_msg.acc[2] = -caerIMU6EventGetAccelZ(event) * STANDARD_GRAVITY;
                // convert from deg/s to rad/s and align axes with camera frame
                this->imu_msg.gyro[0] = -caerIMU6EventGetGyroX(event) / 180.0 * M_PI;
                this->imu_msg.gyro[1] = caerIMU6EventGetGyroY(event) / 180.0 * M_PI;
                this->imu_msg.gyro[2] = -caerIMU6EventGetGyroZ(event) / 180.0 * M_PI;

                // time
                this->imu_msg.time = this->reset_time +
                        ::base::Time::fromMicroseconds(caerIMU6EventGetTimestamp64(event, imu));

                // IMU calibration
                if (this->imu_calibration_running)
                {
                    if (this->imu_calibration_samples.size() < _imu_calibration_sample_size.value())
                    {
                        this->imu_calibration_samples.push_back(this->imu_msg);
                    }
                    else
                    {
                        this->imu_calibration_running = false;
                        this->updateIMUBias();
                    }
                }

                // bias correction
                this->imu_msg.acc -= this->bias.acc;
                this->imu_msg.gyro -= this->bias.gyro;

                _imu.write(this->imu_msg);
            }
        }
        else if (type == FRAME_EVENT)
        {

        }
    }

    caerEventPacketContainerFree(packetContainer);

}
