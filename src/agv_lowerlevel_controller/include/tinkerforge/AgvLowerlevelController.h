#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <modbus/modbus.h>
#include <chrono>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <agv_msgs/SensorState.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <sensor_msgs/Range.h>
//#include "brick_imu_v2.h"
//#include "ip_connection.h"
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int16.h>
#define CW   1
#define CCW  2

#define LEFT                             0
#define RIGHT                            1
#define TICK2RAD                         0.00418879



namespace agv_lowerlevel_controller{

class AgvLowerlevelController{

 public:
  AgvLowerlevelController(ros::NodeHandle& nodeHandle);

  virtual ~AgvLowerlevelController();

  ros::NodeHandle& nh;
 // IMUV2 imu;
  //IPConnection ipcon;

  int LMotor = 0;
  int RMotor = 1;

  int left_count = 0;
  int right_count = 0;

  int LEFT_DIR;
  int RIGHT_DIR;
  int DIR[2] = {LEFT_DIR,RIGHT_DIR};



  float WheelSeparation = 0.35;//0.532;
  float WheelDiameter = 0.10;  //0.20
  int TPR = 1500; //Encoder ticks per rotation
  double WCS[2] = {0.0,0.0};
  float LIMIT_X_VEL = 3.14;  //0.835

  int MotorNum[2] = {LMotor, RMotor};
  long EncoderVal[2] = {0,0};
  double DDis[2] = {0.0,0.0};
  uint64_t Time[2] = {0,0};

  double Vels[2] = {0.0,0.0};
  int CVEL[2]= {0,0};
  int Mspeeds[2] = {0,0};
  double MWS[2]= {0.0,0.0};

  bool init_encoder_[2]  = {false, false};
  int32_t last_diff_tick_[2];
  int32_t last_tick_[2];
  double last_rad_[2];
  double last_velocity_[2];
  double goal_linear_velocity  = 0.0;
  double goal_angular_velocity = 0.0;


  float params[3]={75,10,0.5}; //kp,ki,kd
  float param1[3] = {3000,0.09,0.01};
  float param2[3] = {850,0.4,0.0};

  float err_old[2]={0,0};
  float err_int[2] = {0,0};
  uint64_t DTime[2] = {0,0};

  int OdomWait = 3;
  int OdomCount = 0;
  double pid_data = 0.0;
  int imu_init = 0;
  bool break_flag=true;



  ros::Publisher regs_read;
 //ros::Publisher imu_pub;
  ros::Subscriber buzer_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber sub;
  ros::Subscriber sub_pid;
  ros::Publisher Debug;
  ros::Publisher sensor_state_pub;
  ros::Publisher joint_states_pub;
  ros::Publisher odom_pub;
  ros::Publisher right_sonar_pub;
  ros::Publisher left_sonar_pub;
  ros::Publisher raw_vel;

  

  sensor_msgs::JointState joint_states;
  sensor_msgs::Range right_sonar;
  sensor_msgs::Range left_sonar;
  geometry_msgs::Twist raw_msgs;
  //geometry_msgs::Twist pid_msgs;



  //sensor_msgs::Imu imu_msg;
  geometry_msgs::Twist debug_msg;
  std_msgs::UInt16MultiArray regs_val;
  agv_msgs::SensorState sensor_state_msg;
  nav_msgs::Odometry odom;
  geometry_msgs::TransformStamped tfs_msg;
  geometry_msgs::TransformStamped odom_tf;



  std::vector<std::string> name = {"left_wheel_joint","right_wheel_joint"};
  uint16_t write_reg_data[6] = {0,0,0,0,0,0};
  int write_test = 0;

  modbus_t *plc;
  std::string ip_address;
  int port;
  unsigned short int reg_data[6];
  int reg_data_test;
 // int emergency = 99;
  double last_theta=0.0;
  uint64_t prev_update_time;
  double odom_pose[3];
  double quat[4];
  float joint_states_pos[2] = {0.0, 0.0};
  float joint_states_vel[2] = {0.0, 0.0};
  float joint_states_eff[2] = {0.0, 0.0};
  bool modbus_write_flag = false;
  void callback_pid( const geometry_msgs::Twist& pid_msg);
  void callback_cmd( const geometry_msgs::Twist& CVel);
  void imu_callback(const sensor_msgs::Imu& imu);
  void buzer_callback(const std_msgs::Int16& buzer);
  uint64_t millis();
  uint64_t micros();
  uint64_t nanos();
  void read_register();
  void write_register(int reg_add,int value);
  void setup();
  uint32_t readEncoder(int encoder_num );
  double TicksToMeters(int Ticks);
  double pid_out(double err, int M, double setspeed);
  double CorrectedSpeed(int M, double CVel);
  void motorGo(uint8_t motor, uint8_t direct, uint16_t pwm);
  void publishDebug();
  void buzerState();
  void publish_register_data();
  void publishSensorStateMsg(void);
  void motorWrite();
  void publishSonar();
 // void publishImuMsg(void);
  int write_registers(int addr, int nb, uint16_t data[]);
  void updateJoint(void);
  void updateTF(geometry_msgs::TransformStamped& odom_tf);
  void publishDriveInformation(tf::TransformBroadcaster tfbroadcaster);
  bool updateOdometry(double diff_time);

};
}
