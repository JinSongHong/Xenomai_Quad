#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <mscl/mscl.h>
#include <thread>
#include <iostream>
#include <chrono>
#include <data_mutex.h>

class sensor_data
{
private:

  std::unique_ptr<mscl::Connection> connection;
  std::unique_ptr<mscl::InertialNode> node;
  
  bool IMU_on = false;
  bool get_IMU_on = false;
  double Trunk_data[6] {0,};
  Vector3d Trunk_ori {Vector3d::Zero(),}; // alpha, beta, gamma
  Vector3d Trunk_linear_vel {Vector3d::Zero(),}; 

public:

  sensor_data();
  
  void Init_IMU();
  void get_IMU_data();
  void exchange_mutex();
};

#endif // SENSOR_DATA_H
