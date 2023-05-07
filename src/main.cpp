#include <Arduino.h>
#include <SimpleFOC.h>
#include <arduino-timer.h>
#include "SimpleFOCDrivers.h"
#include "encoders/mt6835/MagneticSensorMT6835.h"

#define MORNITORING true
#define TurnOffMotor_1 true
bool check_waveform = true;

#define SENSOR1_CS 5 // some digital pin that you're using as the nCS pin
MagneticSensorMT6835 sensor_0(SENSOR1_CS);

// 在线电流检测实例
//InlineCurrentSense current_sense_0 = InlineCurrentSense(0.01, 50.0, 39, 36);
InlineCurrentSense current_sense_0 = InlineCurrentSense(0.01f, 50.0f, 39, 36);
//InlineCurrentSense current_sense_0 = InlineCurrentSense(50.0, 39, 36);

//电机参数
BLDCMotor motor_0 = BLDCMotor(11);
BLDCDriver3PWM driver_0 = BLDCDriver3PWM(32, 33, 25, 22);

// include commander interface
Commander command = Commander(Serial);
void doMotor_0(char* cmd)
{
  command.motor(&motor_0, cmd);
}

float target_0 = 3.14; //0.2; //6.28318531; //3.14159265;
float target_1 = 0;
void doTarget_0(char* cmd) { command.scalar(&target_0, cmd); }
//void doTarget_1(char* cmd) { command.scalar(&target_1, cmd); }
float tar_motor_repeat_time = 2; //2s
float old_tar_motor_repeat_time = 0; //2s
void doRepeatTimer(char* cmd) { command.scalar(&tar_motor_repeat_time, cmd); }
float next_motor_mode = 0;
void doMotorMode(char* cmd) { command.motion(&motor_0, cmd);} // change control mode
void doTopSpeed(char* cmd) { command.scalar(&motor_0.velocity_limit, cmd);} // change control mode

auto timer = timer_create_default();  // create a timer with default settings
auto timer_motor_loop = timer_create_default();  // create a timer with default settings
bool motor_dir = false;

unsigned int n = 0;
bool timer_callback(void*)
{
  Serial.print(F("Motor 0 angle, vel and curr: "));
  Serial.print(sensor_0.getAngle());
  Serial.print(" / ");
  Serial.print(sensor_0.getVelocity());
  Serial.print(" / ");
  Serial.print(current_sense_0.getDCCurrent());
  Serial.print("\t");
  Serial.println(rand_r(&n));  // counter

  // Serial.println(F("Motor 1 angle: "));
  // Serial.println(sensor1.getAngle());
  return true;
}
bool timer_motor_loop_callback(void*)
{
  motor_dir = !motor_dir;
  return true;
}


unsigned int loop_time_cmd_val_to_local(float f)
{
  unsigned int t = (unsigned int)abs(f)*1000;
  if (t < 500)
  {
    t = 500;
  }
  return t;
}
void check_and_apply_motor_loop_time()
{
  if (old_tar_motor_repeat_time == tar_motor_repeat_time) return;
  timer_motor_loop.cancel();  // clear all timer
  timer_motor_loop.every(loop_time_cmd_val_to_local(old_tar_motor_repeat_time = tar_motor_repeat_time), timer_motor_loop_callback); //repeat every 2s, no less than 500ms
}

void setup()
{
  Serial.begin(921600);
  //Serial.begin(115200);
  Serial.println("");
  
  sensor_0.init();
  Serial.println("sensor ok");
  //连接motor对象与传感器对象
  motor_0.linkSensor(&sensor_0);
  Serial.println("sensor linked to motor");
  //*
  //供电电压设置 [V]
  driver_0.voltage_power_supply = 24;
  driver_0.init();
  Serial.println("driver ok");

  //连接电机和driver对象
  motor_0.linkDriver(&driver_0);
  Serial.println("motor linked to driver");

    // 电流检测
  current_sense_0.init();
  current_sense_0.gain_b *= -1;
  current_sense_0.skip_align = true;
  motor_0.linkCurrentSense(&current_sense_0);

  // 控制环
  // 其他模式 TorqueControlType::voltage TorqueControlType::dc_current
  // max current senser range <3.3a
  motor_0.torque_controller = TorqueControlType::voltage;
  //motor_0.torque_controller = TorqueControlType::dc_current;

  //FOC模型选择
  motor_0.foc_modulation = FOCModulationType::SpaceVectorPWM;

  //运动控制模式设置
  motor_0.controller = MotionControlType::angle;
  //motor_0.controller = MotionControlType::torque;


  //motor_0.voltage_sensor_align = 6;

  // 电流限制
  motor_0.current_limit = 4.4; //current senser max 3.3, driver can hold max 4.4
  // 电压限制
  motor_0.voltage_limit = 24; //24
  // rad/s
  // 6.28 for no gearbox
  motor_0.velocity_limit = 0.5; //30; //0.5;//6.28; //= 115; //max 115

  //target_0 = 3.09795942; //177.5
  //target_0 = 0.13469389; //177.5 / 23 = 7.717391304
  //target_0 = 71.2530667; //177.5 * 23 = 4082.5
  target_0 = 0;

  tar_motor_repeat_time = 15; //motor repeat timing gap, 2 for 2s
  check_waveform = false;

  //23.04.10
  //ok for 115, no obvious overshoot, torque using voltage
  //
  /*
  motor_0.P_angle.P = 20;
  motor_0.P_angle.I = 0;
  motor_0.PID_velocity.P = 0.5;          //0.5
  motor_0.PID_velocity.I = 15;             //10
  motor_0.PID_velocity.output_ramp = 1000; //1000
  motor_0.LPF_velocity.Tf = 0.001;
  motor_0.PID_current_q.P = 3;
  motor_0.PID_current_q.I = 300;
  motor_0.LPF_current_q.Tf = 0.001;
  //*/

  //23.04.13
  //use to mesure torque, ok with no gearbox, torque using current
  //2.4~2.47a at 300g
  //*
  motor_0.P_angle.P = 10;
  motor_0.P_angle.I = 0;
  motor_0.PID_velocity.P = 0.5;          //0.5
  motor_0.PID_velocity.I = 20;             //10
  motor_0.PID_velocity.output_ramp = 1000; //1000
  motor_0.LPF_velocity.Tf = 0.001;
  motor_0.PID_current_q.P = 3;
  motor_0.PID_current_q.I = 300;
  motor_0.LPF_current_q.Tf = 0.001;
  //*/

  #if MORNITORING
  motor_0.useMonitoring(Serial);

  motor_0.monitor_downsample = 0;  // disable monitor at first - optional
  #endif

  //初始化电机
  motor_0.init();
  Serial.println("motor ok");

  //初始化 FOC
  motor_0.initFOC();
  Serial.println("foc ok");

  // add the motor to the commander interface
  // The letter (here 'M') you will provide to the SimpleFOCStudio

  command.add('M', doMotor_0); //change motor parameter, motor 0

  command.add('q', doTarget_0); // change target value, motor 0

  //timer.every(250, timer_callback);  // set up timer
  check_and_apply_motor_loop_time();
  command.add('t', doRepeatTimer);  //change loop repeat time

  command.add('a', doMotorMode);  //change motor mode
  command.add('w', doTopSpeed);  //change motor top speed
  //*/
}
void loop()
{
  motor_0.loopFOC();
  //Serial.println(motor_0.shaft_angle);
  //motor_1.loopFOC();
  //float voltage_control = pid_stb(0 - motor_0.shaft_angle);
  //Serial.println(voltage_control);
  //Serial.println(motor_0.shaft_velocity);
  //Serial.println(motor_0.shaft_velocity_sp);
  //Serial.println(motor_0.shaft_angle_sp);
  //Serial.println("---------------------------------");
  
  if (check_waveform)
  {
    if (motor_dir)
    {
      motor_0.move(target_0);
    }
    else
    {
      motor_0.move(-target_0);
    }
  }
  else
  {
    motor_0.move(target_0);
  }

  //*/
  #if MORNITORING
  // real-time monitoring calls
  motor_0.monitor();
  // real-time commander calls
  command.run();
  #endif

  ///*
  //timer.tick(); // tick the timer
  timer_motor_loop.tick();
  check_and_apply_motor_loop_time();
  //*/
}
