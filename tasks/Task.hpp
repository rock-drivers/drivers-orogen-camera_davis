/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CAMERA_DAVIS_TASK_TASK_HPP
#define CAMERA_DAVIS_TASK_TASK_HPP

#pragma once

// DAVIS driver
#include <libcaercpp/devices/davis.hpp>

/** boost **/
#include <boost/date_time/posix_time/posix_time.hpp>

/** Base types **/
#include <base/samples/IMUSensors.hpp>
#include <base/samples/Frame.hpp>

#include "camera_davis/TaskBase.hpp"

namespace camera_davis{

    /*! \class Task
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Declare a new task context (i.e., a component)

The corresponding C++ class can be edited in tasks/Task.hpp and
tasks/Task.cpp, and will be put in the camera_davis namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','camera_davis::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class Task : public TaskBase
    {

    friend class TaskBase;
    protected:

        static constexpr double STANDARD_GRAVITY = 9.81;

        /** control boolean flag **/
        bool device_is_running;
        bool imu_calibration_running;

        /** Davis camera handler **/
        std::unique_ptr<libcaer::devices::davis> davis_handle;

        /** configuration variables **/
        struct caer_davis_info davis_info;
        std::string device_id;
        camera_davis::DeviceConfig device_config;
        camera_davis::HardwareFilters hardware_filters;
        camera_davis::DVSRegionOfInterest dvs_region_interest;
        camera_davis::APSRegionOfInterest aps_region_interest;
        camera_davis::PixelFilter pixel_filter;
        camera_davis::DavisBiasesStage1 davis_biases_1;
        camera_davis::DavisBiasesStage2 davis_biases_2;
        camera_davis::DavisBiasesAPS davis_biases_aps;
        boost::posix_time::time_duration delta;

        /** Calibration variables **/
        ::base::samples::IMUSensors bias;
        std::vector< ::base::samples::IMUSensors > imu_calibration_samples;

        base::Time reset_time;

        /** output ports variables **/
        ::base::samples::IMUSensors imu_msg;
        camera_davis::EventArray event_array_msg;
        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> frame_msg;

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "camera_davis::Task");

        /** Default deconstructor of Task
         */
        ~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_cbase::Time reset_time onfiguration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        /** @brief connect to the device
         * **/
        bool connect();

        /** @brief configure the device
         * **/
        void configureDevice();

        /** @brief reset the timestamp
         * */
        void resetTimestamps();

        /** @brief simply accumulate data to
         * compute imu bias
         * **/
        void updateIMUBias();

        /** @brief readout data samples from device
         * **/
        void readout();


        template <typename T> int sgn(T val)
        {
              return (T(0) < val) - (val < T(0));
        }

    };
}

#endif

