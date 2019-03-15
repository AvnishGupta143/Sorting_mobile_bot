#include "tinkerforge/AgvLowerlevelController.h"
//#define HOST "localhost"
//#define PORT 4223
//#define UID "6DcMEq"

double a,b,c,d;

namespace agv_lowerlevel_controller {

AgvLowerlevelController::AgvLowerlevelController(ros::NodeHandle& nodeHandle)
    : nh(nodeHandle)
{
      nh.param<std::string>("plc_modbus_node/ip", ip_address, "192.168.0.1");
      nh.param("plc_modbus_node/port", port, 502);
      setup();
/*
      ipcon_create(&ipcon);_
         ROS_ERROR("Could not connect imu device \n");
    
     }
*/

      buzer_sub = nh.subscribe("/tilt",1,&AgvLowerlevelController::buzer_callback, this);
      imu_sub = nh.subscribe("/imu/data",1,&AgvLowerlevelController::imu_callback, this);
      sub = nh.subscribe("/cmd_vel", 1 ,&AgvLowerlevelController::callback_cmd, this);
      sub_pid = nh.subscribe("/pid", 1 ,&AgvLowerlevelController::callback_pid, this);
      right_sonar_pub = nh.advertise<sensor_msgs::Range>("/right_sonar", 1);
      left_sonar_pub = nh.advertise<sensor_msgs::Range>("/left_sonar", 1);
      regs_read = nh.advertise<std_msgs::UInt16MultiArray>("/regs_read", 5);
      Debug = nh.advertise<geometry_msgs::Twist>("/debug",1);
     // sensor_state_pub = nh.advertise<agv_msgs::SensorState>("/sensor_state",5);
    //  imu_pub = nh.advertise<sensor_msgs::Imu>("/imu",5);
      joint_states_pub = nh.advertise<sensor_msgs::JointState>("/joint_states",1);
      odom_pub = nh.advertise<nav_msgs::Odometry>("/odom",1);
     // raw_vel = nh.advertise<geometry_msgs::Twist>("/raw_vel",1);
      for (int i = 0; i<2 ; i++){
            joint_states.name.push_back(name[i]);
          }
      right_sonar.header.frame_id = "right_sonar_link";
      right_sonar.min_range = 0.05;
      right_sonar.max_range = 0.50;
      right_sonar.field_of_view = 0.25;

      left_sonar.header.frame_id = "left_sonar_link";
      left_sonar.min_range = 0.05;
      left_sonar.max_range = 0.50;
      left_sonar.field_of_view = 0.25;
      
     
}


AgvLowerlevelController::~AgvLowerlevelController(){
  modbus_close(plc);
  modbus_free(plc);
 // imu_v2_destroy(&imu);
 // ipcon_destroy(&ipcon);
}

void AgvLowerlevelController::publishSonar(){
  read_register();
  right_sonar.range = ((reg_data[6]*0.0186519865) + 20)/1000;//*0.000319;
  if((right_sonar.range < 0.05))
  {
    right_sonar.range =0.05;
  }
  else if (right_sonar.range > 0.50) 
  {
    right_sonar.range =0.5;
  } 


  left_sonar.range = ((reg_data[5]*0.0186519865) + 20)/1000;//*0.000319;
  if((left_sonar.range < 0.05))
  {
    left_sonar.range =0.05;
  }
  else if (left_sonar.range > 0.50) 
  {
    left_sonar.range =0.5;
  } 

  right_sonar_pub.publish(right_sonar);
  left_sonar_pub.publish(left_sonar);
}


void AgvLowerlevelController::setup(){
  prev_update_time = millis();
  ROS_INFO("Connecting to modbus device on %s/%d", ip_address.c_str(), port);
  plc = modbus_new_tcp(ip_address.c_str(), port);

  if (plc == NULL) {
          ROS_FATAL("Unable to allocate libmodbus context\n");
          return;
      }

  if (modbus_connect(plc) == -1) {
          ROS_FATAL("Failed to connect to modbus device!!!");
          ROS_FATAL("%s", modbus_strerror(errno));
          modbus_free(plc);
          return;
      }
  else {
         ROS_INFO("Connection to modbus device established");
     }

}

uint64_t AgvLowerlevelController::millis()
{
    uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::
                  now().time_since_epoch()).count();
    return ms;
}

uint64_t AgvLowerlevelController::micros()
{
    uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::
                  now().time_since_epoch()).count();
    return us;
}

uint64_t AgvLowerlevelController:: nanos()
{
    uint64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::
                  now().time_since_epoch()).count();
    return ns;
}

void AgvLowerlevelController::read_register(){
  try{
    reg_data_test = modbus_read_registers(plc,0,6,reg_data);
    if (reg_data_test == -1){
        ROS_ERROR("Unable to read reg addr:");
        throw -1;
      }
  }
  catch(int e){
    ROS_ERROR("modbus_error %d",reg_data_test);

  }

}

void AgvLowerlevelController::write_register(int reg_add,int value){
  try{
    reg_data_test = modbus_write_register(plc,reg_add,value);
    if (reg_data_test == -1){
            ROS_ERROR("Unable to write  reg addr %d:",reg_add);
            throw value;
          }

  }
  catch(int e){
    ROS_ERROR("modbus error value %d: out of range",e);

  }

}

int AgvLowerlevelController::write_registers(int addr, int nb,uint16_t data[]){  
    ROS_INFO("write_registers");
	reg_data_test = modbus_write_registers(plc,addr,nb,data);
    if (reg_data_test == -1){
            ROS_ERROR("Unable to write  registers addr ");

          }
    return reg_data_test;


}

uint32_t AgvLowerlevelController::readEncoder(int encoder_num ){
  read_register();
  uint32_t tick = (reg_data[encoder_num]*65536) + reg_data[encoder_num+1];
  return tick;

}

double AgvLowerlevelController::TicksToMeters(int Ticks){
  return (Ticks*3.14*WheelDiameter)/TPR;
}


double AgvLowerlevelController::pid_out(double err, int M, double setspeed)
{
  err_int[M] +=err;
  if (setspeed == 0 && err == 0){
      err_int[M] = 0;
   } 
  pid_data = params[0]*err + params[1]* err_int[M] + params[2]*(err-err_old[M]);
  err_old[M]=err;
//  debug_msg.angular.y = pid_out;
  //ROS_INFO("pid ouput %f , err : %f motor no.: %d",pid_data,err,M);
  return pid_data;
}

double AgvLowerlevelController::CorrectedSpeed(int M, double CVel){

  if(Time[0]==0 && Time[1] == 0)
  {
Time[0] = millis();
Time[1] = millis();
return 0;
}

uint64_t T = millis();
DTime[M] = T-Time[M];
Time[M] = T;

Vels[M] = TicksToMeters(last_diff_tick_[M])/(DTime[M]*0.001);
double diff =  fabs(CVel) - fabs(Vels[M]);
//     if(fabs(diff)>=0.05)
//     {
params[0]=param1[0];
params[1]=param1[1];
params[2] = param1[2];
 // ROS_INFO("1");
//}
//     else{
//params[0]=param2[0];//630
//params[2]=param2[1];//0.4
 // ROS_INFO("2");

//}




if(MWS[M]< 13000 && MWS[M] >=0){MWS[M]=MWS[M]+pid_out(diff,M,CVel);}//(dif*AccParam);}
if(MWS[M] > 13000){MWS[M]=13000;}
if(MWS[M]<0){MWS[M]=0;}

if(CVel == 0){MWS[M] = 0;}

//DEBUG
CVEL[M] = MWS[M];
return MWS[M];

}

void AgvLowerlevelController::motorGo(uint8_t motor, uint8_t direct, uint16_t pwm)
{
if(break_flag==true)
{//
  write_register(15,1);
}
else
{
  write_register(15,0);


if (motor == 0){
      if (direct ==CW){

        write_register(14,0);


        }
      else if(direct == CCW){

        write_register(14,1);

          }
      //write_reg_data[8] = pwm;
      write_register(16,pwm);
      //write_register(10,1);
  }
if(motor == 1){
      if (direct ==CW){

        write_register(17,1);

      }
      else if(direct == CCW){

        write_register(17,0);

         }
      //write_reg_data[9] = pwm;
      write_register(19,pwm);
    } 
}
}
void AgvLowerlevelController::callback_pid( const geometry_msgs::Twist& pid_msg){
   param1[0] = pid_msg.linear.x;
   param1[1] = pid_msg.linear.y;
   param1[2] = pid_msg.linear.z;
   param2[0] = pid_msg.angular.x;
   param2[1] = pid_msg.angular.y;
   param2[2] = pid_msg.angular.z;
   ROS_INFO("pid update %f %f %f %f %f %f",param1[0],param1[1],param1[2],param2[0],param2[1],param2[2]);
}

void AgvLowerlevelController::buzer_callback(const std_msgs::Int16& buzer){
  uint16_t current_condition = buzer.data;
  //read_register();
  //uint16_t last_condition = reg_data[10];
  //if (current_condition != last_condition){
  if (current_condition == 1){
    //write_register(10,1);
    modbus_write_flag = true;
    
   }
  /*else{
    //write_register(10,0);
    modbus_write_flag = false; 
 }
*/
}

void AgvLowerlevelController::buzerState(){
  if (modbus_write_flag == true){
    write_register(20,1);
    modbus_write_flag = false;
}
  /*else{
   write_register(20,0);
}
*/
}



void AgvLowerlevelController::callback_cmd( const geometry_msgs::Twist& CVel){
  //geometry_msgs::Twist twist = twist_msg;
   // ROS_INFO("CMD");
    double vel_x = CVel.linear.x;
    double vel_th = CVel.angular.z;
    double right_vel = 0.0;
    double left_vel = 0.0;

    if((vel_x == 0.0) && (vel_th == 0.0))
    {
     break_flag = true;
     
    }
    else
    {
     break_flag= false;
   
    }

    left_vel = vel_x - vel_th * WheelSeparation / 2.0;
    right_vel = vel_x + vel_th * WheelSeparation / 2.0;

    if(left_vel > LIMIT_X_VEL){
      left_vel = LIMIT_X_VEL;
    }
    else if(left_vel < -LIMIT_X_VEL){
      left_vel = -LIMIT_X_VEL;
    }

    if(right_vel > LIMIT_X_VEL){
      left_vel = LIMIT_X_VEL;
    }
    else if(right_vel < -LIMIT_X_VEL){
      left_vel = -LIMIT_X_VEL;
    }


     WCS[0] = left_vel;
     WCS[1] = right_vel;



}


void AgvLowerlevelController::imu_callback(const sensor_msgs::Imu& imu){

   // ROS_INFO("In call back \n");
    
    quat[0] = imu.orientation.w;
    quat[1] = imu.orientation.x;
    quat[2] = imu.orientation.y;
    quat[3] = imu.orientation.z;
    //ROS_INFO("quant");
    
    a=quat[0];
    b=quat[1];
    c=quat[2];
    d=quat[3];
     
}


void AgvLowerlevelController::publishDebug(){
  
  debug_msg.linear.x = WCS[0];
  debug_msg.linear.y = Vels[0];
  debug_msg.linear.z = Mspeeds[0];
  debug_msg.angular.x = WCS[1];
  debug_msg.angular.y = Vels[1];
  debug_msg.angular.z= Mspeeds[1];
/*
  sensor_state_msg.header.stamp = ros::Time::now();
  sensor_state_msg.header.frame_id = "encoder";
  sensor_state_msg.left_encoder = readEncoder(0);
  sensor_state_msg.right_encoder = readEncoder(2);
  sensor_state_pub.publish(sensor_state_msg);*/
  Debug.publish(debug_msg);
}

void AgvLowerlevelController::publish_register_data(){
  regs_val.data.clear();
  read_register();
  for (int i = 0; i < 6; i++) {
    regs_val.data.push_back(reg_data[i]);
            }
  if (regs_val.data.size() > 0) {
              regs_read.publish(regs_val);
          }

}

void AgvLowerlevelController::publishSensorStateMsg(void)
{
  int32_t current_tick;


  current_tick = readEncoder(0);//sensor_state_msg.left_encoder;
  if (!init_encoder_[LEFT])
  {
    last_tick_[LEFT] = current_tick;
    init_encoder_[LEFT] = true;
  }
  last_diff_tick_[LEFT] = current_tick - last_tick_[LEFT];
  last_tick_[LEFT] = current_tick;
  last_rad_[LEFT] += TICK2RAD * (double)last_diff_tick_[LEFT];



  current_tick =readEncoder(2) ;//sensor_state_msg.right_encoder;

  if (!init_encoder_[RIGHT])
  {
    last_tick_[RIGHT] = current_tick;
    init_encoder_[RIGHT] = true;
  }

  last_diff_tick_[RIGHT] = current_tick - last_tick_[RIGHT];
  last_tick_[RIGHT] = current_tick;
  last_rad_[RIGHT] += TICK2RAD * (double)last_diff_tick_[RIGHT];

}

void AgvLowerlevelController::motorWrite(){
  for(int i =0; i<2; i++){
    if(WCS[i]>0){
      DIR[i] = CW;
    }
    else if(WCS[i] <0){
      DIR[i] = CCW;
    }
  }
    Mspeeds[0] = CorrectedSpeed(0, WCS[0]);
    Mspeeds[1] = CorrectedSpeed(1, WCS[1]);

    motorGo(MotorNum[0], DIR[0], Mspeeds[0]);
    motorGo(MotorNum[1], DIR[1], Mspeeds[1]);
   // write_test = write_registers(4,6,write_reg_data);
    //ROS_DEBUG("write condition : %d ",write_test);
  }

/*
void AgvLowerlevelController::publishImuMsg(void)
{
  imu_msg.header.stamp    = ros::Time::now();
  imu_msg.header.frame_id = "imu_link";
  int16_t w,x,y,z;

  if(imu_v2_get_angular_velocity(&imu, &x, &y, &z) < 0) {
              ROS_INFO("Could not get quaternion, probably timeout");

          }

  imu_msg.angular_velocity.x = (float)x/16.0;
  imu_msg.angular_velocity.y = (float)x/16.0;
  imu_msg.angular_velocity.z = (float)x/16.0;
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
buzerState
  imu_pub.publish(imu_msg);

}
*/
bool AgvLowerlevelController::updateOdometry(double diff_time)
{
  double odom_vel[3];

  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, delta_theta, current_theta;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  if (std::isnan(wheel_l))
      wheel_l = 0.0;

  if (std::isnan(wheel_r))
      wheel_r = 0.0;

  step_time = diff_time;

  if (step_time == 0)
      return false;

  wheel_l= TicksToMeters((double)last_diff_tick_[LEFT]);
  wheel_r= TicksToMeters((double)last_diff_tick_[RIGHT]);

  if (std::isnan(wheel_l))
      wheel_l = 0.0;

  if (std::isnan(wheel_r))
        wheel_r = 0.0;

  Vels[0]=wheel_l/step_time;
  Vels[1]=wheel_r/step_time;


/*
  int16_t wa,x,y,z;

  if(imu_v2_get_quaternion(&imu, &wa, &x, &y, &z) < 0) {
              ROS_INFO("Could not get quaternion, probably timeout");
          }

  double quat[4] = {(double)wa/16383.0,(double)x/16383.0,(double)y/16383.0,(double)z/16383.0};
*/
 // current_theta= atan2(quat[1]*quat[2] + quat[0]*quat[3],
   //                      0.5f - quat[2]*quat[2] - quat[3]*quat[3]);


 // current_theta= atan2(b*c + a*d,
     //                    0.5f - c*c - d*d);
  delta_theta = (wheel_r - wheel_l) / WheelSeparation;
  delta_s     = (wheel_r + wheel_l) / 2.0;
//make delta theta zero at first condition if imu not publishing zero at initial condition
 /* if (imu_init == 0 && current_theta != 0){
   last_theta = current_theta;
   imu_init=imu_init+1;
   ROS_INFO(" imu_init");
 }
*/
  //delta_theta = current_theta - last_theta;
  raw_msgs.linear.x = delta_s;
  raw_msgs.angular.z = delta_theta;
  v = delta_s / step_time;
  w = delta_theta / step_time;
  last_velocity_[LEFT]  = last_rad_[LEFT]/step_time;
  last_velocity_[RIGHT] = last_rad_[RIGHT]/step_time;
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation =tf::createQuaternionMsgFromYaw(odom_pose[2]);
  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];

  last_theta = current_theta;
  raw_vel.publish(raw_msgs);
  return true;

}

void AgvLowerlevelController::updateJoint(void)
{
  joint_states_pos[LEFT]  = last_rad_[LEFT];
  joint_states_pos[RIGHT] = last_rad_[RIGHT];
//  ROS_INFO("joint states %ld",joint_states_pos[LEFT]);

  joint_states_vel[LEFT]  = last_velocity_[LEFT];
  joint_states_vel[RIGHT] = last_velocity_[RIGHT];
  joint_states.position.clear();
  joint_states.velocity.clear();

  for (int i = 0; i < 2 ; i++){
    joint_states.position.push_back(joint_states_pos[i]);
    joint_states.velocity.push_back(joint_states_vel[i]);
  }

}

void AgvLowerlevelController::updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom.header.frame_id = "odom";
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = "base_footprint";
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation = odom.pose.pose.orientation;
}

void AgvLowerlevelController::publishDriveInformation(tf::TransformBroadcaster tfbroadcaster)
{
  uint64_t time_now = millis();
  uint64_t step_time = time_now - prev_update_time;
  prev_update_time = time_now;

  ros::Time stamp_now = ros::Time::now();
  updateOdometry(double(step_time*0.001));
  odom.header.stamp = stamp_now;
  odom.child_frame_id = "base_footprint";
  odom_pub.publish(odom);

  // joint_states
  updateJoint();
  joint_states.header.stamp = stamp_now;
  joint_states.header.frame_id = "base_link";
  joint_states_pub.publish(joint_states);

  updateTF(odom_tf);
  tfbroadcaster.sendTransform(odom_tf);
}
}
