#include "tinkerforge/AgvLowerlevelController.h"
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <boost/thread.hpp>

#define CONTROL_MOTOR_SPEED_PERIOD       30   //hz
#define IMU_PUBLISH_PERIOD               15.0  //hz
#define DRIVE_INFORMATION_PUBLISH_PERIOD 30   //hz
#define DEBUG_INFORMATION_PUBLISH_PERIOD 15.0



int main(int argc, char** argv)

{
  ros::init(argc, argv, "agv_lowerlevel_controller");
  ros::NodeHandle nodeHandle("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  tf::TransformBroadcaster tfbroadcaster;
  uint64_t tTime[4];
  agv_lowerlevel_controller::AgvLowerlevelController agvLowerlevelControllerObj(nodeHandle);

  while(ros::ok()){
        uint64_t t = agvLowerlevelControllerObj.millis();
        if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_PERIOD)){
                //ROS_INFO("loop 1 %ld",(t-tTime[0]) );
                agvLowerlevelControllerObj.motorWrite();
                tTime[0] = t;
              }

        if ((t-tTime[1]) >= (1000 / DEBUG_INFORMATION_PUBLISH_PERIOD)){
                //ROS_INFO("loop 2 %ld",(t-tTime[1]) );

                agvLowerlevelControllerObj.publish_register_data();
                agvLowerlevelControllerObj.buzerState();
                agvLowerlevelControllerObj.publishDebug();
                //agvLowerlevelControllerObj.publishSonar();
                tTime[1] = t;
                    }

        if ((t-tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_PERIOD)){
               // ROS_INFO("loop 3 %ld",(t-tTime[2]) );
                agvLowerlevelControllerObj.publishSensorStateMsg();
                agvLowerlevelControllerObj.publishDriveInformation(tfbroadcaster);
                tTime[2] = t;
              }
/*
        if ((t-tTime[3]) >= (1000 / IMU_PUBLISH_PERIOD)){
                      agvLowerlevelControllerObj.publishImuMsg();
                      tTime[3] = t;
                    }
*/
        //ROS_INFO("time %ld",t);

        //ros::spinOnce();

      }
      spinner.stop();
      return 0;
}
