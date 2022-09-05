/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

//#define DEBUG_PRINTS 1

using namespace imu_myahrs_plus;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    prev_ts = base::Time::fromSeconds(0.00);
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
    prev_ts = base::Time::fromSeconds(0.00);
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

     /*
     * 	start communication with the myAHRS+.
     */
    if(sensor.start(_port.value(), _baudrate.value()) == false)
    {
        RTT::log(RTT::Error)<<"[IMU_MYAHRS_PLUS] ERROR opening serial port!!"<<RTT::endlog();
        return false;
    }

    /*
     *  set binary output format
     *   - select Quaternion and IMU data
     */
    if(sensor.cmd_binary_data_format("QUATERNION, IMU") == false)
    {
        RTT::log(RTT::Error)<<"[IMU_MYAHRS_PLUS] ERROR setting the binary output format!!"<<RTT::endlog();
        return false;
    }

    /** Calculate the divider based on the task period **/
    double sampling_frequency = 1.0/base::Time::fromSeconds(this->getPeriod()).toSeconds();
    if (sampling_frequency > MYAHRS_PLUS_MAX_RATE)
    {
        sampling_frequency = MYAHRS_PLUS_MAX_RATE;
    }
    std::string divider = std::to_string(MYAHRS_PLUS_MAX_RATE/sampling_frequency);

    /*
     *  set divider
     *   - output rate(Hz) = max_rate/divider
     */
    if(sensor.cmd_divider(divider.c_str()) ==false)
    {
        RTT::log(RTT::Error)<<"[IMU_MYAHRS_PLUS] ERROR setting the sampling frequency (myAHRS+ divider)!!"<<RTT::endlog();
        return false;
    }

    /*
     *  set transfer mode
     *   - BC : Binary Message & Continuous mode
     */
    if(sensor.cmd_mode("BC") ==false)
    {
        RTT::log(RTT::Error)<<"[IMU_MYAHRS_PLUS] ERROR setting the transfer mode!!"<<RTT::endlog();
        return false;
    }

    /** Frame names for the output port transformation **/
    this->orientation_out.sourceFrame = _source_frame.value();
    this->orientation_out.targetFrame = _target_frame.value();

   // std::vector<std::string> attributes = sensor.get_attribute_list();
   // for(std::vector<std::string>::iterator it=attributes.begin(); it!=attributes.end(); it++)
   // {
   //     std::cout<<(*it)<<"\n";
   // }
   // std::string offset_orientation;
   // sensor.get_attribute(attributes[9].c_str(), offset_orientation);
   // std::cout<<attributes[9]<<" is "<<offset_orientation<<"\n";

   // std::string command = std::string("@set_offset,") + offset_orientation;

    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    /** Output variable **/
    this->orientation_out.invalidate();
    this->orientation_out.orientation.setIdentity();

    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    if(sensor.wait_data() == true) // waiting for new data
    {
        /** Time is current time minus the latency **/
        base::Time recvts = base::Time::now();

        #ifdef DEBUG_PRINTS
        base::Time diffTime = recvts - prev_ts;
        std::cout<<"Delta time[s]: "<<diffTime.toSeconds()<<"\n";
        prev_ts = recvts;
        #endif

        /*
        * copy sensor data
        */
        sensor.get_data(sensor_data);

        /*
         * print quaternion & imu data
         */
        WithRobot::Quaternion& q = sensor_data.quaternion;
        WithRobot::ImuData<float>& imu = sensor_data.imu;
        //printf("Quaternion(xyzw)=%.4f,%.4f,%.4f,%.4f, Accel(xyz)=%.4f,%.4f,%.4f, Gyro(xyz)=%.4f,%.4f,%.4f, Magnet(xyz)=%.2f,%.2f,%.2f\n",
        //        q.x, q.y, q.z, q.w,
        //        imu.ax, imu.ay, imu.az,
        //        imu.gx, imu.gy, imu.gz,
        //        imu.mx, imu.my, imu.mz);

        /** Convert to Rock IMU sensor data **/
        base::samples::IMUSensors imusamples;
        imusamples.time = recvts;
        imusamples.acc = base::Vector3d(GRAVITY*imu.ax, GRAVITY*imu.ay, GRAVITY*imu.az);
        imusamples.gyro = base::Vector3d(D2R*imu.gx, D2R*imu.gy, D2R*imu.gz);
        imusamples.mag = base::Vector3d(imu.mx, imu.my, imu.mz);
        _inertial_sensors_out.write(imusamples);

        /** Convert to Rock RBS **/
	/** Have a look at the IMU documentation to see the frame location **/
        orientation_out.time = recvts;
        orientation_out.orientation = base::Orientation(q.w, q.x, q.y, q.z);
        orientation_out.orientation.normalize();
        _orientation_samples_out.write(orientation_out);
    }
}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();

    /*
     * 	stop communication
     */
    sensor.stop();
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    /*
     * 	stop communication
     */
    sensor.stop();

}
