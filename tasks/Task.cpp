/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <atomic>
#include <csignal>

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


static std::atomic_bool globalShutdown(false);

static void hdl(int signal, siginfo_t *siginfo, void *context)
{
    // Simply set the running flag to false on SIGTERM and SIGINT (CTRL+C) for global shutdown.
    if (signal == SIGTERM || signal == SIGINT)
    {
        reinterpret_cast<Task*>(context)->stopHook();
        reinterpret_cast<Task*>(context)->cleanupHook();
        globalShutdown.store(true);
    }
}

static void usbShutdownHandler(void *ptr)
{
    (void) (ptr); // UNUSED.

    globalShutdown.store(true);
}

Task::Task(std::string const& name)
    : TaskBase(name)
{
    this->device_is_running = false;
    this->imu_calibration_running = false;
    this->davis_handle = nullptr;
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
    RTT::log(RTT::Info)<<"Connection result: "<<is_connected<<" - Set TaskContext period to: "<<TaskContext::getPeriod()<<"[seconds]"<<RTT::endlog();

    /** configure we set the configuration (after we connect) **/
    this->configureDevice();

    /** imu calibration control variables **/
    this->imu_calibration_running = true;
    this->imu_calibration_samples.clear();

    /** calibration parameters in frame helper **/
    this->task_frame_helper.setCalibrationParameter(_camera_parameters.value());

    /** set the frame member **/
    ::base::samples::frame::Frame *frame = new ::base::samples::frame::Frame();
    this->frame_msg.reset(frame);
    frame = nullptr;

    /** catch the signals **/
    struct sigaction action;
    action.sa_sigaction = &hdl;
    action.sa_flags   = SA_SIGINFO;
    sigemptyset(&action.sa_mask);
    sigaddset(&action.sa_mask, SIGTERM);
    sigaddset(&action.sa_mask, SIGINT);

    return is_connected;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    /** Start device **/
    this->davis_handle->configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);
    this->davis_handle->dataStart(nullptr, nullptr, nullptr, &usbShutdownHandler, nullptr);
    this->next_send_time = boost::posix_time::microsec_clock::local_time();

    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    while (true)
    {
        std::cout<<"****\n";
        this->readout();
    }
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
    this->davis_handle->dataStop();
    this->device_is_running = false;
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    caerLog(CAER_LOG_INFO, "Exiting from DAVIS camera driver",  "executing driver cleanup(close)");
    RTT::log(RTT::Info)<<"Exiting from DAVIS camera driver, executing driver cleanup(close)"<<RTT::endlog();
	// Close automatically done by destructor.
    TaskBase::cleanupHook();

}

bool Task::connect()
{
    this->device_is_running = false;

    /** Serial number as a pointer to char **/
    const char* serial_number_restrict = (_serial_number.value() == "") ? NULL : _serial_number.value().c_str();

    if(serial_number_restrict)
    {
      RTT::log(RTT::Info)<<"Requested serial number: "<< _serial_number.value()<<RTT::endlog();
    }

    try
    {
        this->davis_handle = std::unique_ptr<libcaer::devices::davis>(new libcaer::devices::davis(1, 0, 0, serial_number_restrict));
    }
    catch(std::runtime_error& e)
    {
        RTT::log(RTT::Warning)<<"Could not find DAVIS camera. ERROR: "<<e.what()<<RTT::endlog();
        this->device_is_running = false;
        return false;
    }

    // was opening successful
    this->device_is_running = true;

    this->davis_info = this->davis_handle->infoGet();
    this->device_id = "DAVIS-" + std::string(this->davis_info.deviceString).substr(14, 8);

    RTT::log(RTT::Info)<<this->davis_info.deviceString<<" --- ID:"<<this->davis_info.deviceID<<
        ", Master:"<<this->davis_info.deviceIsMaster <<", DVS X:"<<this->davis_info.dvsSizeX <<", DVS Y:"<<this->davis_info.dvsSizeY<<
        ", Logic: "<< this->davis_info.logicVersion<<RTT::endlog();


    // Send the default configuration before using the device.
    // No configuration is sent automatically!
    this->davis_handle->sendDefaultConfig();

    // In case autoexposure is enabled, initialize the exposure time with the exposure value
    // from the parameter server
    if(this->device_config.autoexposure_enabled)
    {
        this->davis_handle->configSet (DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, this->device_config.exposure);
    }

    this->resetTimestamps();

    return this->device_is_running;
}

void Task::resetTimestamps()
{
    this->davis_handle->configSet(DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RESET, 1);
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
        this->davis_handle->configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, this->device_config.exposure);
    }

    this->davis_handle->configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_MODE, this->device_config.frame_mode);
    this->davis_handle->configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_INTERVAL, this->device_config.frame_interval);

    this->davis_handle->configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, this->device_config.aps_enabled);
    this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN, this->device_config.dvs_enabled);
    this->davis_handle->configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_ACCELEROMETER, this->device_config.imu_enabled);
    this->davis_handle->configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_GYROSCOPE, this->device_config.imu_enabled);
    this->davis_handle->configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_TEMPERATURE, this->device_config.imu_enabled);

    if (this->device_config.imu_gyro_scale >= 0 && this->device_config.imu_gyro_scale <= 3)
        this->davis_handle->configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_GYRO_FULL_SCALE, this->device_config.imu_gyro_scale);

    if (this->device_config.imu_acc_scale >= 0 && this->device_config.imu_acc_scale <= 3)
        this->davis_handle->configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ACCEL_FULL_SCALE, this->device_config.imu_acc_scale);

    if (this->device_config.imu_low_pass_filter >= 0 && this->device_config.imu_low_pass_filter <= 6)
    {
        this->davis_handle->configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_DIGITAL_LOW_PASS_FILTER, this->device_config.imu_low_pass_filter);

            if(this->device_config.imu_low_pass_filter == 0)
            {
              // When the low pass filter is disabled, the output frequency of IMU events
              // is raised to 8KHz. To keep it to 1 kHz, we use the sample rate divider
              // (setting its value to 7 divides the frequency by 8).
              this->davis_handle->configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER, 7);
            }
            else
            {
              // When the low pass filter is enabled, the gyroscope output rate is set to 1 kHz,
              // so we should not use the sample rate divider, in order to keep the IMU output rate to 1 kHz.
              this->davis_handle->configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER, 0);
            }
    }
    if (this->davis_info.chipID == DAVIS_CHIP_DAVIS346B)
    {
        // VDAC
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_APSOVERFLOWLEVEL,
                            caerBiasVDACGenerate(VDAC(27,6)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_APSCAS,
                            caerBiasVDACGenerate(VDAC(21,6)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ADCREFHIGH,
                            caerBiasVDACGenerate(VDAC(this->davis_biases_aps.ADC_RefHigh_volt, this->davis_biases_aps.ADC_RefHigh_curr)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ADCREFLOW,
                            caerBiasVDACGenerate(VDAC(this->davis_biases_aps.ADC_RefLow_volt, this->davis_biases_aps.ADC_RefLow_curr)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ADCTESTVOLTAGE,
                            caerBiasVDACGenerate(VDAC(21,7)));
        // CF Biases
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_LOCALBUFBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(5, 164)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_PADFOLLBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(7, 215)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_DIFFBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(this->davis_biases_2.DiffBn_coarse, this->davis_biases_2.DiffBn_fine)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ONBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(this->davis_biases_2.ONBn_coarse, this->davis_biases_2.ONBn_fine)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_OFFBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(this->davis_biases_2.OFFBn_coarse, this->davis_biases_2.OFFBn_fine)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_PIXINVBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(5, 129)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_PRBP,
                            caerBiasCoarseFineGenerate(CF_P_TYPE(this->davis_biases_1.PrBp_coarse, this->davis_biases_1.PrBp_fine)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_PRSFBP,
                            caerBiasCoarseFineGenerate(CF_P_TYPE(this->davis_biases_1.PrSFBp_coarse, this->davis_biases_1.PrSFBp_fine)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_REFRBP,
                            caerBiasCoarseFineGenerate(CF_P_TYPE(this->davis_biases_2.RefrBp_coarse, this->davis_biases_2.RefrBp_fine)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_READOUTBUFBP,
                            caerBiasCoarseFineGenerate(CF_P_TYPE(6, 20)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_APSROSFBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(6, 219)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ADCCOMPBP,
                            caerBiasCoarseFineGenerate(CF_P_TYPE(5, 20)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_COLSELLOWBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(0, 1)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_DACBUFBP,
                            caerBiasCoarseFineGenerate(CF_P_TYPE(6, 60)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_LCOLTIMEOUTBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(5, 49)));;
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_AEPDBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(6, 91)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_AEPUXBP,
                            caerBiasCoarseFineGenerate(CF_P_TYPE(4, 80)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_AEPUYBP,
                            caerBiasCoarseFineGenerate(CF_P_TYPE(7, 152)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_IFREFRBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_IFTHRBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_BIASBUFFER,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(5, 254)));
        // Special Biases
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_SSP,
                            caerBiasShiftedSourceGenerate(SHIFTSOURCE(1,33,SHIFTED_SOURCE)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_SSN,
                            caerBiasShiftedSourceGenerate(SHIFTSOURCE(1,33,SHIFTED_SOURCE)));

        // Hardware filters
        if (this->davis_info.dvsHasPixelFilter)
        {
          this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_AUTO_TRAIN, this->pixel_filter.pixel_auto_train);

          // if using auto train, update the configuration with hardware values
          if (this->pixel_filter.pixel_auto_train)
          {
            this->davis_handle->configGet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_ROW, (uint32_t*)&(this->pixel_filter).pixel_0_row);
            this->davis_handle->configGet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_COLUMN, (uint32_t*)&(this->pixel_filter).pixel_0_column);
            this->davis_handle->configGet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_ROW, (uint32_t*)&(this->pixel_filter).pixel_1_row);
            this->davis_handle->configGet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_COLUMN, (uint32_t*)&(this->pixel_filter).pixel_1_column);
            this->davis_handle->configGet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_ROW, (uint32_t*)&(this->pixel_filter).pixel_2_row);
            this->davis_handle->configGet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_COLUMN, (uint32_t*)&(this->pixel_filter).pixel_2_column);
            this->davis_handle->configGet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_ROW, (uint32_t*)&(this->pixel_filter).pixel_3_row);
            this->davis_handle->configGet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_COLUMN, (uint32_t*)&(this->pixel_filter).pixel_3_column);
            this->davis_handle->configGet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_ROW, (uint32_t*)&(this->pixel_filter).pixel_4_row);
            this->davis_handle->configGet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_COLUMN, (uint32_t*)&(this->pixel_filter).pixel_4_column);
            this->davis_handle->configGet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_ROW, (uint32_t*)&(this->pixel_filter).pixel_5_row);
            this->davis_handle->configGet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_COLUMN, (uint32_t*)&(this->pixel_filter).pixel_5_column);
            this->davis_handle->configGet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_ROW, (uint32_t*)&(this->pixel_filter).pixel_6_row);
            this->davis_handle->configGet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_COLUMN, (uint32_t*)&(this->pixel_filter).pixel_6_column);
            this->davis_handle->configGet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_ROW, (uint32_t*)&(this->pixel_filter).pixel_7_row);
            this->davis_handle->configGet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_COLUMN, (uint32_t*)&(this->pixel_filter).pixel_7_column);
          }
          else // apply current configuration to hardware
          {
            this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_ROW, this->pixel_filter.pixel_0_row);
            this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_COLUMN, this->pixel_filter.pixel_0_column);
            this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_ROW, this->pixel_filter.pixel_1_row);
            this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_COLUMN, this->pixel_filter.pixel_1_column);
            this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_ROW, this->pixel_filter.pixel_2_row);
            this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_COLUMN, this->pixel_filter.pixel_2_column);
            this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_ROW, this->pixel_filter.pixel_3_row);
            this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_COLUMN, this->pixel_filter.pixel_3_column);
            this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_ROW, this->pixel_filter.pixel_4_row);
            this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_COLUMN, this->pixel_filter.pixel_4_column);
            this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_ROW, this->pixel_filter.pixel_5_row);
            this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_COLUMN, this->pixel_filter.pixel_5_column);
            this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_ROW, this->pixel_filter.pixel_6_row);
            this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_COLUMN, this->pixel_filter.pixel_6_column);
            this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_ROW, this->pixel_filter.pixel_7_row);
            this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_COLUMN, this->pixel_filter.pixel_7_column);
          }
        }

        if (this->davis_info.dvsHasBackgroundActivityFilter)
        {
          this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY, this->hardware_filters.background_activity_filter_enabled);
          this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY_TIME, this->hardware_filters.background_activity_filter_time);

          this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD, this->hardware_filters.refractory_period_enabled);
          this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD_TIME, this->hardware_filters.refractory_period_time);
        }

        if (this->davis_info.dvsHasROIFilter)
        {
          this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_START_COLUMN, this->dvs_region_interest.roi_start_column);
          this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_START_ROW, this->dvs_region_interest.roi_start_row);
          this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_END_COLUMN, this->dvs_region_interest.roi_end_column);
          this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_END_ROW, this->dvs_region_interest.roi_end_row);
        }

        if (this->davis_info.dvsHasSkipFilter)
        {
          this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_SKIP_EVENTS, this->hardware_filters.skip_enabled);
          this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_SKIP_EVENTS_EVERY, this->hardware_filters.skip_every);
        }

        if (this->davis_info.dvsHasPolarityFilter)
        {
          this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_POLARITY_FLATTEN, this->hardware_filters.polarity_flatten);
          this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_POLARITY_SUPPRESS, this->hardware_filters.polarity_suppress);
          this->davis_handle->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_POLARITY_SUPPRESS_TYPE, this->hardware_filters.polarity_suppress_type);
        }

        // APS region of interest
        this->davis_handle->configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_0, this->aps_region_interest.aps_roi_start_column);
        this->davis_handle->configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_0, this->aps_region_interest.aps_roi_start_row);
        this->davis_handle->configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_0, this->aps_region_interest.aps_roi_end_column);
        this->davis_handle->configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_0, this->aps_region_interest.aps_roi_end_row);
    }
    else
    {
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP,
                            caerBiasCoarseFineGenerate(CF_P_TYPE(this->davis_biases_1.PrBp_coarse, this->davis_biases_1.PrBp_fine)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP,
                            caerBiasCoarseFineGenerate(CF_P_TYPE(this->davis_biases_1.PrSFBp_coarse, this->davis_biases_1.PrSFBp_fine)));

        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_DIFFBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(this->davis_biases_2.DiffBn_coarse, this->davis_biases_2.DiffBn_fine)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_ONBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(this->davis_biases_2.ONBn_coarse, this->davis_biases_2.ONBn_fine)));
        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_OFFBN,
                            caerBiasCoarseFineGenerate(CF_N_TYPE(this->davis_biases_2.OFFBn_coarse, this->davis_biases_2.OFFBn_fine)));

        this->davis_handle->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_REFRBP,
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

    std::unique_ptr<libcaer::events::EventPacketContainer> packet_container = this->davis_handle->dataGet();

    /** Skip if nothing there **/
    if (packet_container == nullptr)
    {
        std::cout<<"Container empty\n";
        return;
    }

    printf("\nGot event container with %d packets (allocated).\n", packet_container->size());

    for (auto &packet : *packet_container)
    {
        if (packet == nullptr)
        {
            //printf("Packet is empty (not present).\n");
            continue; // Skip if nothing there.
        }

        // Packet 0 is always the special events packet for DVS128, while packet is the polarity events packet.
        if (packet->getEventType() == POLARITY_EVENT)
        {
            this->event_array_msg.time = this->reset_time + ::base::Time::fromMicroseconds(packet_container->getLowestEventTimestamp());
            this->event_array_msg.height = this->davis_info.dvsSizeY;
            this->event_array_msg.width = this->davis_info.dvsSizeX;

            std::shared_ptr<const libcaer::events::PolarityEventPacket> polarity
                = std::static_pointer_cast<libcaer::events::PolarityEventPacket>(packet);


            for (const auto &evt : *polarity)
            {
                camera_davis::Event e;
                e.ts = this->reset_time + ::base::Time::fromMicroseconds(evt.getTimestamp64(*polarity));
                e.x = evt.getX();
                e.y = evt.getY();
                e.polarity   = evt.getPolarity();
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
        else if (packet->getEventType() == IMU6_EVENT)
        {

            std::shared_ptr<const libcaer::events::IMU6EventPacket> imu
                = std::static_pointer_cast<libcaer::events::IMU6EventPacket>(packet);

            // Print out timestamps and data.
            for (const auto &evt : *imu)
            {
                this->imu_msg.time = this->reset_time +
                    ::base::Time::fromMicroseconds(evt.getTimestamp64(*imu));

                // convert from g's to m/s^2 and align axes with camera frame
                this->imu_msg.acc[0]  = -evt.getAccelX() * STANDARD_GRAVITY;
                this->imu_msg.acc[1]  = evt.getAccelY() * STANDARD_GRAVITY;
                this->imu_msg.acc[2] = -evt.getAccelZ() * STANDARD_GRAVITY;
                // convert from deg/s to rad/s and align axes with camera frame
                this->imu_msg.gyro[0] = -evt.getGyroX() / 180.0 * M_PI;
                this->imu_msg.gyro[1] = evt.getGyroY() / 180.0 * M_PI;
                this->imu_msg.gyro[2] = -evt.getGyroZ() / 180.0 * M_PI;

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
        else if (packet->getEventType() == FRAME_EVENT)
        {
            std::shared_ptr<const libcaer::events::FrameEventPacket> frame
                = std::static_pointer_cast<libcaer::events::FrameEventPacket>(packet);

            for (const auto &f : *frame)
            {
                if (f.getROIIdentifier() != 0)
                {
                    continue;
                }

                /*cv::Mat cv_frame = f.getOpenCVMat(false);
                std::string ty =  type2str( cv_frame.type() );
                printf("Matrix: %s %dx%d \n", ty.c_str(), cv_frame.cols, cv_frame.rows );
                std::cout<<"Number channels: "<<channel2str(f.getChannelNumber())<<"\n";*/

                /** Get image frame metadata **/
                ::base::samples::frame::Frame tmp_frame;
                tmp_frame.init(f.getLengthX(), f.getLengthY(), static_cast<uint8_t>(8), ::base::samples::frame::MODE_GRAYSCALE, f.getPixelsSize());

                /** get the image data **/
                 tmp_frame.image.clear();
                for (int img_y=0; img_y<f.getLengthY(); img_y++)
                {
                    for (int img_x=0; img_x<f.getLengthX(); img_x++)
                    {
                        const uint16_t value = f.getPixel(img_x, img_y);
                        tmp_frame.image.push_back(value >> 8);
                    }
                }

                /** Undistort the image **/
                ::base::samples::frame::Frame *frame_msg_ptr = this->frame_msg.write_access();
                frame_msg_ptr->init(tmp_frame.size.width, tmp_frame.size.height, tmp_frame.getDataDepth(), tmp_frame.getFrameMode());
                frame_msg_ptr->image.clear();
                this->task_frame_helper.convert(tmp_frame, *frame_msg_ptr);

                /** Write in the class member **/
                frame_msg_ptr->time = (this->reset_time + ::base::Time::fromMicroseconds(f.getTimestamp()));
                frame_msg_ptr->received_time = ::base::Time::fromMicroseconds(f.getTimestamp());
                this->frame_msg.reset(frame_msg_ptr);

                /** Write the camera frame into the port **/
                _frame.write(this->frame_msg);
            }
        }
    }
}

