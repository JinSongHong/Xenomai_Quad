#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <data_mutex.h>
#include <controller.h>

class trajectory
{
private:
  //// For Homming ////

  Controller joint_c[NUMOFSLAVES]; // joint controller
  bool Homming_checked = false;
  bool Homming_clicked = false;
  bool Homming_HAA_W_flag = false;
  int Homming_t = 0;
  int HAA_W_Homming_t = 0;
  
  
  double Homming_duration = 2.5;
  double Homming_gain = 120;
  double Motor_Home_pos[14];
  double Motor_pos_init[14];
  double Motor_pos[14];
  double Homming_input[14];
  double joint_kp[NUMOFSLAVES]= {200,600,600,  200, 200, 150, 150,200,200,  200,200,150,   0, 200};
  double joint_kd[NUMOFSLAVES]= {2,5,5,      2,2,2,       2,2,2,        2,2,2,      0,1};
  double joint_ki[NUMOFSLAVES]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  double Homming_traj[NUMOFSLAVES];
  
  //// For Leg Trajectory  ////
    
  int Leg_num;
  Vector4d ref_r_vel;
  Vector4d ref_th_vel;
  Vector4d r_velRW;
  Vector4d th_velRW;
  Vector4d ori_dhat_LPF {Vector4d::Zero()};
  Vector4d ori_DOB_delta {Vector4d::Zero()};
  
  //// Trunk ////
  double T_W = 0.25;
  double T_L = 0.66;
  Vector2d ref_ori {Vector2d::Zero()}; // Alpha, Beta
  
  //// Orientation DOB ////
  MatrixXd Trunk_ori_jacb {4,2};
  
  //// Flag ////
  int Trunk_Leg_CtrlMode = 100;
  bool OriDOB_on = false;
  
public:
  trajectory();
  double* homming();
  void Exchagne_mutex();
  void exchange_mutex(int Leg_num);
  void leg_vel_traj(double t, int Leg_num);
  
  //// Trunk ////
  void Trunk_vel_traj(double t);
  
  bool safety_flag = true;
  

};

#endif // TRAJECTORY_H
// motor[0] = 60
// motor[1] = 180
// motor[2] = 190
// motor[3] = 
