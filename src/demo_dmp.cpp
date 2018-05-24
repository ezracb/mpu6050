#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float last_ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
sensor_msgs::Imu imu;
ros::Publisher imu_pub;
boost::array<double, 9> linear_acceleration_cov;
boost::array<double, 9> angular_velocity_cov;
boost::array<double, 9> orientation_cov;
boost::array<double, 9> unk_orientation_cov;
boost::array<double, 9> magnetic_cov;
ros::Time last_update;


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // initialize device
    ROS_INFO("Initializing I2C devices...\n");
    mpu.initialize();

    // verify connection
    ROS_INFO("Testing device connections...\n");
    ROS_INFO(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

    // load and configure the DMP
    ROS_INFO("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        ROS_INFO("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        ROS_INFO("DMP ready!\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        ROS_ERROR_STREAM("DMP Initialization failed (code " << devStatus <<")\n");
    }
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        ROS_WARN("FIFO overflow!\n");

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (fifoCount >= 42) {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        ros::Time now = ros::Time::now();
        imu.header.stamp = now;
        ros::Duration dt_r = now - last_update;
        last_update = now;

        // display quaternion values in easy matrix form: w x y z
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        imu.orientation.x = q.x;
        imu.orientation.y = q.y;
        imu.orientation.z = q.z;
        imu.orientation.w = q.w;

        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        float yaw_ang_vel = ypr[0] - last_ypr[0] / dt_r.toSec();
        float pitch_ang_vel = ypr[1] - last_ypr[1] / dt_r.toSec();
        float roll_ang_vel = ypr[2] - last_ypr[2] / dt_r.toSec();

        imu.angular_velocity.x = roll_ang_vel;
        imu.angular_velocity.y = pitch_ang_vel;
        imu.angular_velocity.z = yaw_ang_vel;

        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        //printf("aworld %6d %6d %6d    ", aaWorld.x, aaWorld.y, aaWorld.z);
        imu.linear_acceleration.x = aaWorld.x;
        imu.linear_acceleration.y = aaWorld.y;
        imu.linear_acceleration.z = aaWorld.z;
    }
}

void setup_covariance(boost::array<double, 9> &cov, double stdev) {
    std::fill(cov.begin(), cov.end(), 0.0);
    if (stdev == 0.0)
        cov[0] = -1.0;
    else {
        cov[0 + 0] = cov[3 + 1] = cov[6 + 2] = std::pow(stdev, 2);
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "mpu6050");
    ros::NodeHandle nh;
    double linear_stdev, angular_stdev, orientation_stdev;
    int update_rate;
    nh.param("linear_acceleration_stdev", linear_stdev, 0.0003);
    nh.param("angular_velocity_stdev", angular_stdev, 0.02 * (M_PI / 180.0));
    nh.param("orientation_stdev", orientation_stdev, 1.0);
    nh.param("update_rate", update_rate, 100);

    setup_covariance(linear_acceleration_cov, linear_stdev);
    setup_covariance(angular_velocity_cov, angular_stdev);
    setup_covariance(orientation_cov, orientation_stdev);

    imu.header.frame_id = "imu_link";
    imu.orientation_covariance = orientation_cov;
    imu.angular_velocity_covariance = angular_velocity_cov;
    imu.linear_acceleration_covariance = linear_acceleration_cov;
    imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 5);

    setup();
    ros::Duration d(2.0);
    d.sleep();

    ros::Rate r(update_rate);

    while(ros::ok()){
        ros::spinOnce();
        loop();
        imu_pub.publish(imu);
        r.sleep();
    }
    return 0;
}
