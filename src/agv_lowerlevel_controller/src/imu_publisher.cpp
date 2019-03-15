#include <ros/time.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "brick_imu_v2.h"
#include "ip_connection.h"
#include <string.h>

#define PORT 4223
#define UID "6DcMEq"

#define HOST "localhost"

IMUV2 imu;
IPConnection ipcon;

ros::Publisher imu_pub;
sensor_msgs::Imu imu_msg;

const double deg2rad = M_PI/180;

void publishImuMsg(void)
{
  imu_msg.header.stamp    = ros::Time::now();
  imu_msg.header.frame_id = "imu_link";
  int16_t w,x,y,z;

  if(imu_v2_get_angular_velocity(&imu, &x, &y, &z) < 0) {
              ROS_INFO("Could not get quaternion, probably timeout");

          }

  imu_msg.angular_velocity.x = (float)x/16.0*deg2rad;
  imu_msg.angular_velocity.y = (float)x/16.0*deg2rad;
  imu_msg.angular_velocity.z = (float)x/16.0*deg2rad;
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0.02;

  if(imu_v2_get_linear_acceleration(&imu, &x, &y, &z) < 0) {
                ROS_INFO("Could not get quaternion, probably timeout");
            }

  imu_msg.linear_acceleration.x = (float)x/100.0;
  imu_msg.linear_acceleration.y = (float)x/100.0;
  imu_msg.linear_acceleration.z = (float)x/100.0;
  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;
  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[5] = 0;
  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

  if(imu_v2_get_quaternion(&imu, &w, &x, &y, &z) < 0) {
            ROS_INFO("Could not get quaternion, probably timeout");
        }
  imu_msg.orientation.w = (float)w/16383.0;
  imu_msg.orientation.x = (float)x/16383.0;
  imu_msg.orientation.y = (float)y/16383.0;
  imu_msg.orientation.z = (float)z/16383.0;

  imu_msg.orientation_covariance[0] = 0.0025;
  imu_msg.orientation_covariance[1] = 0;
  imu_msg.orientation_covariance[2] = 0;
  imu_msg.orientation_covariance[3] = 0;
  imu_msg.orientation_covariance[4] = 0.0025;
  imu_msg.orientation_covariance[5] = 0;
  imu_msg.orientation_covariance[6] = 0;
  imu_msg.orientation_covariance[7] = 0;
  imu_msg.orientation_covariance[8] = 0.0025;

  imu_pub.publish(imu_msg);

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_pub");
    ros::NodeHandle nh;
    ipcon_create(&ipcon);
    imu_v2_create(&imu, UID, &ipcon);
    if(ipcon_connect(&ipcon, HOST, PORT) < 0) {
                   ROS_ERROR("Could not connect imu device \n");
                   return 1;
                    }
 if(imu_v2_set_sensor_fusion_mode(&imu,2) < 0){
      ROS_ERROR("fusion mode unable to set");
    }
    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data_brick",1);
    uint8_t mode = 3;
   // imu_v2_set_sensor_fusion_mode(&imu,mode);

/*    if(imu_v2_set_sensor_fusion_mode(&imu,mode) < 0){
      ROS_ERROR("Could not remove magnetometer \n");
      return 1;
 }*/
     
    ros::Rate r(32);
    while (nh.ok()){

    publishImuMsg();
    ros::spinOnce();
    r.sleep();
}
   imu_v2_destroy(&imu);
   ipcon_destroy(&ipcon);
   return 0;
}
