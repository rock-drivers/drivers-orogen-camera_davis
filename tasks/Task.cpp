/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace camera_davis;

Task::Task(std::string const& name)
    : TaskBase(name)
{  caerDeviceHandle davis_handle_;
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

    /** At configure we set the configuration and the dynamic reconfiguration
     * **/

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    /** At start we connect to the camera **/

    this->device_is_running = false;

    /** Seria number as a pointer to char **/
    const char* serial_number_restrict = (_serial_number.value() == "") ? NULL : _serial_number.value().c_str();

    if(serial_number_restrict)
    {
      RTT::log(RTT::Info)<<"Requested serial number: %s"<< _serial_number.value().c_str()<<RTT::endlog();
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
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    caerLog(CAER_LOG_INFO, "Exiting from DAVIS camera driver",  "executing data stop");
    RTT::log(RTT::Info)<<"Exiting from DAVIS camera driver, executing data stop"<<RTT::endlog();
    caerDeviceDataStop(this->davis_handle);
    caerDeviceClose(&(this->davis_handle));
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    RTT::log(RTT::Info)<<"Exiting from DAVIS camera driver, executing driver cleanup"<<RTT::endlog();
    caerLog(CAER_LOG_ERROR, "destructor",  "data stop now");
    caerDeviceClose(&(this->davis_handle));
    TaskBase::cleanupHook();
}

void Task::resetTimestamps()
{
    caerDeviceConfigSet(this->davis_handle, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RESET, 1);
    base::Time reset_time = base::Time::now();

    RTT::log(RTT::Info)<<"Reset timestamps on "<< this->device_id.c_str()<<" to "<< reset_time.toSeconds()<<RTT::endlog();

    // if master, publish reset time to slaves
    //if (master_)
    //{
        //std_msgs::Time reset_msg;
        //reset_msg.data = reset_time_;
        //reset_pub_.publish(reset_msg);
    //}
    return;
}
