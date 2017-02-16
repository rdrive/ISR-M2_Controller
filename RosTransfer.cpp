#include "Controller.h"
#include "RosTransfer.h"
#include <isr_m2_driver/RobotStatus.h>
#include <isr_m2_driver/RobotCommand.h>
#include <isr_m2_driver/EncoderValue.h>
#include <isr_m2_driver/WheelVelocity.h>
#include <isr_m2_driver/SetWheelVelocity.h>

void onRobotStatus(const isr_m2_driver::RobotStatus::Request& req, isr_m2_driver::RobotStatus::Response res)
{
  res.motor_enabled = (controller.IsMotorEnabled() ? 1 : 0);
  res.motor_stopped = (controller.IsMotorStopped() ? 1 : 0);
  res.estop_pressed = (controller.IsEStopPressed() ? 1 : 0);
}

void onEncoderValue(const isr_m2_driver::EncoderValue::Request& req, isr_m2_driver::EncoderValue::Response& res)
{
  unsigned long leftEncoder, rightEncoder;
  controller.ReadEncoder(leftEncoder, rightEncoder);
  res.l_pulse_count = leftEncoder;
  res.r_pulse_count = rightEncoder;
}

void onWheelVelocity(const isr_m2_driver::WheelVelocity::Request& req, isr_m2_driver::WheelVelocity::Response& res)
{
  int actual_leftMotorRPM, actual_rightMotorRPM;
  controller.ReadActualMotorSpeed(actual_leftMotorRPM, actual_rightMotorRPM);
  res.l_motor_dir = controller.IsLeftMotorDirForward();
  res.r_motor_dir = controller.IsRightMotorDirForward();
  res.l_motor_vel_rpm = actual_leftMotorRPM;
  res.r_motor_vel_rpm = actual_rightMotorRPM;
}

void onSetWheelVelocity(const isr_m2_driver::SetWheelVelocity::Request& req, isr_m2_driver::SetWheelVelocity::Response& res)
{
  int leftMotorRPM = req.l_motor_vel_rpm;
  int rightMotorRPM = req.r_motor_vel_rpm;
  controller.MotorSpeed(leftMotorRPM, rightMotorRPM);
}

void onRobotCommand(const isr_m2_driver::RobotCommand::Request& req, isr_m2_driver::RobotCommand::Response& res)
{
  uint8_t command = req.command;
  uint8_t arg1    = req.arg1;

  switch (command) 
  {
    case isr_m2_driver::RobotCommandRequest::COMMAND_INITIALIZE:
      controller.Initialize();
      break;

    case isr_m2_driver::RobotCommandRequest::COMMAND_ENABLE_MOTOR:
      controller.MotorEnable(arg1 == 0 ? false : true);
      break;

    case isr_m2_driver::RobotCommandRequest::COMMAND_STOP_MOTOR:
      controller.MotorStop(arg1 == 0 ? false : true);
      break;

    case isr_m2_driver::RobotCommandRequest::COMMAND_RESET_ENCODER:
      controller.ResetEncoder();
      break;
  }
}

ros::ServiceServer<isr_m2_driver::RobotStatus::Request, isr_m2_driver::RobotStatus::Response> robot_status_srv("robot_status", onRobotStatus);
ros::ServiceServer<isr_m2_driver::RobotCommand::Request, isr_m2_driver::RobotCommand::Response> robot_command_srv("robot_command", onRobotCommand);
ros::ServiceServer<isr_m2_driver::EncoderValue::Request, isr_m2_driver::EncoderValue::Response> encoder_value_srv("encoder_value", onEncoderValue);
ros::ServiceServer<isr_m2_driver::WheelVelocity::Request, isr_m2_driver::WheelVelocity::Response> wheel_velocity_srv("wheel_velocity", onWheelVelocity);
ros::ServiceServer<isr_m2_driver::SetWheelVelocity::Request, isr_m2_driver::SetWheelVelocity::Response> set_wheel_velocity_srv("set_wheel_velocity", onSetWheelVelocity);

void RosTransfer::init()
{
  nh.initNode();
  nh.advertiseService(robot_status_srv);
  nh.advertiseService(robot_command_srv);
  nh.advertiseService(encoder_value_srv);
  nh.advertiseService(wheel_velocity_srv);
  nh.advertiseService(set_wheel_velocity_srv);
}

void RosTransfer::spinOnce()
{
  nh.spinOnce();
}


