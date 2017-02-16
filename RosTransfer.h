#ifndef __RosTransfer_H__
#define __RosTransfer_H__

#include <ArduinoHardware.h>
#include <ros.h>
  
class RosTransfer {
public:
  void init();
  void spinOnce();

private:
  ros::NodeHandle nh;
};

#endif

