/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef IMU_MYAHRS_PLUS_TASK_TASK_HPP
#define IMU_MYAHRS_PLUS_TASK_TASK_HPP

#include "imu_myahrs_plus/TaskBase.hpp"

/** Rock data types **/
#include <base/Time.hpp>
#include <base/samples/IMUSensors.hpp>
#include <base/samples/RigidBodyState.hpp>

/** myAHRS+ library **/
#include <imu_myahrs_plus/myahrs_plus.hpp>

#include <map>
#include <string>

namespace imu_myahrs_plus
{

    /** MYAHRS+ constant values **/
    static const int MYAHRS_PLUS_MAX_RATE = 100; /** Maximum sampling rate in Hz **/

    /** WGS-84 ellipsoid constants (Nominal Gravity Model and Earth angular velocity) **/
    static const int Re = 6378137; /** Equatorial radius in meters **/
    static const int Rp = 6378137; /** Polar radius in meters **/
    static const double ECC = 0.0818191908426; /** First eccentricity **/
    static const double GRAVITY = 9.79766542; /** Mean value of gravity value in m/s^2 **/
    static const double GWGS0 = 9.7803267714; /** Gravity value at the equator in m/s^2 **/
    static const double GWGS1 = 0.00193185138639; /** Gravity formula constant **/
    static const double EARTHW = 7.292115e-05; /** Earth angular velocity in rad/s **/

    /*! \class Task
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Declare a new task context (i.e., a component)

     * The corresponding C++ class can be edited in tasks/Task.hpp and
     * tasks/Task.cpp, and will be put in the imu_myahrs_plus namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','imu_myahrs_plus::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:

        /**************************/
        /*** Internal Variables ***/
        /**************************/
        base::Time prev_ts;

        WithRobot::MyAhrsPlus sensor;
        WithRobot::SensorData sensor_data;

        /***************************/
        /** Output port variables **/
        /***************************/
        base::samples::RigidBodyState orientation_out; /** the output orientation **/


    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "imu_myahrs_plus::Task");

        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices.
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

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
           needs_configuration
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
    };
}

#endif

