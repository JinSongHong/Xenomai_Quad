#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <data_mutex.h>
#include <actuator.h>
#include <filter.h>


class Controller
{
private:
  filter Filter;
  
  double Ts;

  //// PID ////
  Vector4d RW_r_posPgain; // FL FR RL RR leg
  Vector4d RW_r_posIgain;
  Vector4d RW_r_posDgain;
  Vector4d RW_r_posD_cutoff;

  Vector4d RW_th_posPgain; // FL FR RL RR
  Vector4d RW_th_posIgain;
  Vector4d RW_th_posDgain;
  Vector4d RW_th_posD_cutoff;
  
  Vector4d RW_r_velPgain; // FL FR RL RR leg
  Vector4d RW_r_velIgain;
  Vector4d RW_r_velDgain;
  Vector4d RW_r_velD_cutoff;

  Vector4d RW_th_velPgain; // FL FR RL RR
  Vector4d RW_th_velIgain;
  Vector4d RW_th_velDgain;
  Vector4d RW_th_velD_cutoff;

  Vector2d tau_mutex_dhat[4];
  Vector2d Extforce_mutex[4];

  Vector2d PID_output;
  double cutoff_freq_pos = 50; 
  double cutoff_freq_vel = 50;
  double cutoff_freq = 50; 
  
  
  //// RWDOB ////
  double cutoff_RWDOB;
  Vector4d RWDOB_cutoff = Vector4d{4,4,4,4};
  Vector2d thddot;
  Vector2d lhs_dob;
  Vector2d rhs_dob;
  Vector2d lhs_dob_old;
  Vector2d rhs_dob_old;
  Vector2d lhs_dob_LPF;
  Vector2d rhs_dob_LPF;
  Vector2d lhs_dob_LPF_old;
  Vector2d rhs_dob_LPF_old;
  Vector2d tau_dhat;
  bool RWDOB_on = false; 
  bool Ctrl_on = false;
  
  //// RWFOB ////
  double cutoff_RWFOB;
  Vector4d RWFOB_cutoff = Vector4d{10,10,10,10};
  Vector2d T_fob[2];
  Vector2d lhs_fob = Vector2d{0,0};
  Vector2d rhs_fob = Vector2d{0,0};
  Vector2d tauExt_hat[2]; //[old][th_m,th_b] (2,2)
  Vector2d forceExt_hat[3]; //[old][r,theta] (3,2)
  
  //// Admittance parameter ////
  Vector2d deltapos[3]; // [old][r,theta] (3,2)
  
  //// Using in Function ////
  double P_term[2][2]; // first column is about r, second column is about theta
  double I_term[2][2];
  double D_term[2][2];
  double kp; double ki; double kd;
  double kp_pos; double ki_pos; double kd_pos;
  double kp_vel; double ki_vel; double kd_vel;
  Vector2d error;
  Vector2d error_old;
  
  //// joint PID ////
  double j_Kp_, j_Kd_, j_Ki_; 
  double j_I_term_[2] = {0};
  double j_D_term_[2] = {0};
  double j_P_term_ = 0;
  double j_imax_;
  double j_err[2] = {0};
  
  //// Model Parameters ////
  double L;
  double d_thigh;
  double d_shank;
  double m_thigh;
  double m_shank;
  double m_leg;
  double m_trunk;
  double m_total;
  double Izz_thigh;
  double Izz_shank;
  double Jzz_thigh;
  double Jzz_shank;
  double th2;
  double M1;
  double M2;
  double M12;
  double JzzR_thigh;
  double JzzR_couple;
  double JzzR_shank;
  double g = 9.80665;
  Vector2d C;
  Matrix2d M_Joint;
  Matrix2d M_RW;
  Matrix2d M_Norminal;
  
  //// Trunk ////
  double T_W = 0.25;
  double T_L = 0.66;
  MatrixXd Trunk_ori_jacb {4,2};
  
  //// Orientation DOB ////
  Vector2d Trunk_ori_for_DOB {Vector2d::Zero(),};  
  Vector4d r_vel_ref {Vector4d::Zero(),};      // From Kinematics
  Vector4d r_vel_from_IMU {Vector4d::Zero(),}; // Kinematics
  Vector4d delta_vel_ref {Vector4d::Zero(),};
  Vector4d ori_dhat[2] {Vector4d::Zero(),};
  Vector4d ori_dhat_LPF[2] {Vector4d::Zero(),};
  Vector4d Ori_DOB_cutoff = Vector4d{10,10,10,10};
  bool OriDOB_on = false;
  
public:
  Controller();

  // Data set//
  void setDelayData();
  void exchange_mutex(int Leg_num);
  void Exchagne_mutex();
  void Cal_Parameter(double thm, double thb, double thmdot, double thbdot);
  
  // DOB
  Vector2d RWDOB(double thmddot, double thbddot, Vector2d tau, Matrix2d Jacobian,int Leg_num);
  void orientation_DOB();
  // FOB
  void RWFOB(Vector2d DOB_output, Matrix2d Jacobian_T, double thddot_m, double thddot_b,int Leg_num);
  
  //Admittance Control
  Vector2d admittance(double omega_n, double zeta, double k);
  
  
  // feedback control //;
  double pid(Vector2d posRW_err, Vector2d posRW_err_old, Vector2d velRW_err, Vector2d velRW_err_old, int r0th1, int Leg_num, int mode); // idx:  r(=0), th(=1)중 어떤 state의 PD control?     
  double jointpid(double joint_pos, double joint_pos_old);
  double tustin_derivative(double input, double input_old, double output_old, double cutoff_freq);
  
  // gain setting
  void j_set_gain(double Kp, double Ki, double Kd){j_Kp_ = Kp; j_Ki_ = Ki; j_Kd_ = Kd; }
  // reset when
  void j_reset();
  // constrain
  double j_constraint(double v, double v_min, double v_max){return (v> v_max)?v_max : (v<v_min)?v_min:v;}
  
  // PID operator
  double j_posPID(double target, double current_ang, double dt, double cutoff);
  void j_setDelayData() {j_D_term_[1] = j_D_term_[0]; j_I_term_[1] = j_I_term_[0];}
  double j_get_posPgain() {return j_Kp_;}
  double j_get_posIgain() {return j_Ki_;}
  double j_get_posDgain() {return j_Kd_;}

  double get_posPgain(int Leg_num, int r0th1);
  double get_posIgain(int Leg_num, int r0th1);
  double get_posDgain(int Leg_num, int r0th1);
  double get_posD_cutoff(int Leg_num, int r0th1);
  
  double get_velPgain(int Leg_num, int r0th1);
  double get_velIgain(int Leg_num, int r0th1);
  double get_velDgain(int Leg_num, int r0th1);
  double get_velD_cutoff(int Leg_num, int r0th1);
};

#endif // CONTROLLER_H
