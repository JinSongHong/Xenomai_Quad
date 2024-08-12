#include "controller.h"

Controller::Controller()
{ 
  for(int i = 0; i < 2; i ++)
  {
    forceExt_hat[0][i] = 0;
    forceExt_hat[1][i] = 0;
    forceExt_hat[2][i] = 0;
    
    deltapos[0][i] = 0.0;
    deltapos[1][i] = 0.0;
    deltapos[2][i] = 0.0;
    
    tau_dhat[i] = 0.0;
    rhs_dob_LPF[i] = 0.0;
    lhs_dob_LPF[i] = 0.0;
    rhs_dob[i] = 0.0;
    lhs_dob[i] = 0.0;
    
    
    j_I_term_[i] = 0;
    j_D_term_[i] = 0;
    j_P_term_ = 0;
    j_imax_ = 0;
    j_err[i] = 0;
  
  }
  
    error.setZero();
    error_old.setZero();
    tauExt_hat[0].setZero();
    tauExt_hat[1].setZero();
    tauExt_hat[2].setZero();
    
  T_fob[0].setZero();
  T_fob[1].setZero();
  
  Ts = 0.001;
  cutoff_RWFOB = 10;
  
  Trunk_ori_jacb << T_W/2, -T_L/2, -T_W/2, -T_L/2, T_W/2, T_L/2, -T_W/2, T_L/2;

}
//



double Controller :: tustin_derivative(double input, double input_old, double output_old, double cutoff_freq)
{
    double time_const = 1 / (2 * M_PI * cutoff_freq);
    double output = 0;

    output = (2 * (input - input_old) - (T - 2 * time_const) * output_old) / (T + 2 * time_const);

    return output;
}

// PID operator
double Controller ::j_posPID(double target, double current_ang, double dt, double cutoff)
{   
    
    double t_const= 1/(2*M_PI*cutoff); // time constant
    j_err[0] = target-current_ang;
    j_P_term_ = j_Kp_*j_err[0];
    j_D_term_[0] = (j_Kd_*2*(j_err[0]-j_err[1])-(dt-2*t_const)*j_D_term_[1])/(2*t_const+dt);
    j_I_term_[0] = j_I_term_[1] + j_Ki_*dt/2*(j_err[0] + j_err[1]);
    j_I_term_[0] = j_constraint(j_I_term_[0],-10,10);
    j_err[1] = j_err[0];
    return j_P_term_+ j_I_term_[0] + j_D_term_[0];
}

void Controller ::j_reset(){ //to eliminate residual value in pid term
  for(int i = 0 ; i < 2 ; i++ ){
    j_I_term_[i] = 0;
    j_D_term_[i] = 0;
    j_P_term_ = 0;
    j_imax_ = 0;
    j_err[i] = 0;
  }
}


double Controller::pid(Vector2d posRW_err, Vector2d posRW_err_old, Vector2d velRW_err, Vector2d velRW_err_old, int idx,int Leg_num, int mode)
                                       // posRW_err[0] : R direction
                                       // posRW_err[1] : th direction
                                       // posRW_err_old same
                                       // mode 0 : position control mode, mode 1 : velocity control, mode 2: Cascade?
{
    
    kp_pos = get_posPgain(Leg_num, idx);
    ki_pos = get_posIgain(Leg_num, idx);
    kd_pos = get_posDgain(Leg_num, idx);
    cutoff_freq_pos = get_posD_cutoff(Leg_num, idx);
    kp_vel = get_velPgain(Leg_num, idx);
    ki_vel = get_velIgain(Leg_num, idx);
    kd_vel = get_velDgain(Leg_num, idx);
    cutoff_freq_vel = get_velD_cutoff(Leg_num, idx);
    
    
  if(mode == 0) // mode = 0: position PID
  {
    kp = kp_pos;
    ki = ki_pos;
    kd = kd_pos;
    cutoff_freq = cutoff_freq_pos;
    for (int i = 0; i < 2; i++)
    {error[i] = posRW_err[i];
     error_old[i] =  posRW_err_old[i];}
  }
  else if(mode == 1) // mode = 1: velocity PID
  {
    kp = kp_vel;
    ki = ki_vel;
    kd = kd_vel;
    
    cutoff_freq = cutoff_freq_vel;
    for (int i = 0; i < 2; i++)
    {error[i] = velRW_err[i];
     error_old[i] =  velRW_err_old[i];}
  }
  else
  {
    kp = 0;
    ki = 0;
    kd = 0;
    cutoff_freq = 50;
  }
  double tau = 1 / (2 * PI * cutoff_freq);
  
  P_term[idx][0] = kp * error[idx];
  I_term[idx][0] = ki * T / 2 * (error[idx] + error_old[idx]) + I_term[idx][1];
  D_term[idx][0] = 2 * kd / (2 * tau + T) * (error[idx] - error_old[idx]) -
                       (T - 2 * tau) / (2 * tau + T) * D_term[idx][1]; // 이 함수 내에서 r_err는 주소값 but [0]와 같은 배열 위치로 원소를
                                                                           // 특정해주면 그 부분의 value가 된다.(이건 그냥 c++ 문법)
  PID_output[idx] = P_term[idx][0] + D_term[idx][0] + I_term[idx][0];

  
  return PID_output[idx];
}

Vector2d Controller::RWDOB(double thmddot, double thbddot, Vector2d tau, Matrix2d Jacobian, int Leg_num)
{
  
  if(Ctrl_on==true)
  {
//  tau_mutex_dhat(2,4);
   cutoff_RWDOB = RWDOB_cutoff[Leg_num];
   thddot[0] = thmddot;
   thddot[1] = thbddot;
   
   lhs_dob = tau;
   rhs_dob = Jacobian.transpose()*M_Norminal*Jacobian* thddot;  // Q is already applied

  
   if(RWDOB_on == true)
   {
    for(int i = 0; i < 2; i++)
    {
      lhs_dob_LPF[i] = Filter.lowpassfilter(lhs_dob[i], lhs_dob_old[i], lhs_dob_LPF_old[i], cutoff_RWDOB);
      rhs_dob_LPF[i] = Filter.lowpassfilter(rhs_dob[i], rhs_dob_old[i], rhs_dob_LPF_old[i], cutoff_RWDOB);
      
      tau_dhat[i] = lhs_dob_LPF[i] - rhs_dob_LPF[i];

    }
   }
   else
   {
   for(int i = 0; i < 2; i++) 
   {tau_dhat[i] = 0;}
   }
   
    tau_mutex_dhat[Leg_num] = tau_dhat;

//    cout << tau_mutex_dhat[0][0] << endl;
   return tau_dhat;
  }
  else return Vector2d::Zero() ;
}
void Controller::RWFOB(Vector2d tau, Matrix2d Jacobian_T, double thddot_m, double thddot_b, int Leg_num)
{
  if (Ctrl_on == true)
  {
  cutoff_RWFOB = RWFOB_cutoff[Leg_num];
  
  Vector2d result;
  Vector2d qddot;
  
  qddot[0] = thddot_m;
  qddot[1] = thddot_b;
  
  lhs_fob = tau; // desired - DOBoutput
  rhs_fob = M_Joint * qddot;
  
  // calculate FOB output
  for(int i=  0; i <2 ; i++) // 
  
  
  {
    T_fob[0][i] = lhs_fob[i] - rhs_fob[i];
  }
  
  

  double tau_2 = 1/(2*M_PI*cutoff_RWFOB);
  
  for(int i = 0 ; i< 2 ; i++)
  {
    tauExt_hat[0][i] = Filter.lowpassfilter(T_fob[0][i], T_fob[1][i], tauExt_hat[1][i], cutoff_RWFOB);
  }
    
  result = Jacobian_T.inverse() * tauExt_hat[0]; // 0th colum is current tauExt_hat
 
  // buble arrangement before 
  tauExt_hat[1] = tauExt_hat[0];  // This is problem 
  
  
  forceExt_hat[2] = forceExt_hat[1];
  forceExt_hat[1] = forceExt_hat[0];
  forceExt_hat[0] = result; // r direction
  
  Extforce_mutex[Leg_num] = result;
  }
}

Vector2d Controller::admittance(double omega_n, double zeta, double k)
{
  double ad_M = k/(pow(omega_n,2));
  double ad_B = 2*zeta*k/omega_n;
  double ad_K = k;
  
  double c1 = 4 * ad_M + 2 * ad_B * Ts + ad_K * pow(Ts, 2);
  double c2 = -8 * ad_M + 2 * ad_K * pow(Ts, 2);
  double c3 = 4 * ad_M - 2 * ad_B * Ts + ad_K * pow(Ts, 2);
  
//  cout << "c1" << c1 << endl;
//  cout << "c2" << c2 << endl;
//  cout << "c3" << c3 << endl;

//  cout << "in Admittance: " << forceExt_hat[0][0] << endl;
  
  deltapos[0][0] =
        (pow(Ts, 2) * forceExt_hat[0][0] + 2 * pow(Ts, 2) * forceExt_hat[1][0] +
            pow(Ts, 2) * forceExt_hat[2][0] - c2 * deltapos[1][0] - c3 * deltapos[2][0]) / c1;
  
  deltapos[0][1] = 0;
    
  return deltapos[0];
  
  
}


void Controller::Cal_Parameter(double thm, double thb, double thmdot, double thbdot)
{
  th2 = thb - thm; 
  
   /* Leg Parameters */
    L = 0.25;
    d_thigh = 0.11017; // local position of CoM of thigh
    d_shank = 0.12997; // local position of CoM of shank

    m_thigh = 1.017; // mass of thigh link
    m_shank = 0.143; // mass of shank link
    m_leg = m_thigh + m_shank;
    m_total = m_trunk + 4 * m_leg;

    Izz_thigh = 0.0057;     // MoI of thigh w.r.t. CoM
    Izz_shank = 8.0318e-04; // MoI of shank w.r.t. CoM

    Jzz_thigh =
        Izz_thigh + m_thigh * pow(d_thigh, 2); // MoI of thigh w.r.t. HFE
    Jzz_shank =
        Izz_shank + m_shank * pow(d_shank, 2); // MoI of thigh w.r.t. KFE

    M1 = Jzz_thigh + m_shank * pow(L, 2);
    M2 = m_shank * d_shank * L * cos(th2);
    M12 = Jzz_shank;
    
    
    
    JzzR_thigh  = Jzz_thigh + Jzz_shank + m_shank * pow(L, 2) - 2 * m_shank * d_shank * L * cos(th2);
    JzzR_couple = Jzz_thigh + m_shank * pow(L, 2) - Jzz_shank;
    JzzR_shank = Jzz_thigh + Jzz_shank+ m_shank * pow(L, 2) + 2 * m_shank * d_shank * L * cos(th2);
    
    M_Joint(0,0) = M1;   M_RW(0,0) = JzzR_thigh / (4 * pow(L, 2) * pow(sin(th2 / 2), 2));
    M_Joint(0,1) = M12;  M_RW(0,1) = JzzR_couple / (2 * pow(L, 2) * sin(th2));
    M_Joint(1,0) = M12;  M_RW(1,0) = JzzR_couple / (2 * pow(L, 2) * sin(th2));
    M_Joint(1,1) = M2;   M_RW(1,1) = JzzR_shank / (4 * pow(L, 2) * pow(cos(th2 / 2), 2));
    
    M_RW(0,0) = JzzR_thigh / (4 * pow(L, 2) * pow(sin(th2 / 2), 2));
    M_RW(0,1) = JzzR_couple / (2 * pow(L, 2) * sin(th2));
    M_RW(1,0) = JzzR_couple / (2 * pow(L, 2) * sin(th2));
    M_RW(1,1) = JzzR_shank / (4 * pow(L, 2) * pow(cos(th2 / 2), 2));
    
    M_Norminal(0,0) = M_RW(0,0);
    M_Norminal(0,1) = 0;
    M_Norminal(1,0) = 0;
    M_Norminal(1,1) = M_RW(1,1);
    
    
    
    C[0] = -m_shank * d_shank * L * sin(th2) * pow(thbdot, 2) - g * (m_thigh * d_thigh + m_shank * L) * cos(thm);
    C[1] = m_shank * d_shank * L * sin(th2) * pow(thmdot, 2)- g * m_shank * d_shank * cos(thb);

}

void Controller:: orientation_DOB()
{
  if(OriDOB_on == true)  
  {
    r_vel_from_IMU = Trunk_ori_jacb * Trunk_ori_for_DOB;
    
    ori_dhat[0] = r_vel_ref - r_vel_from_IMU; 
    
   
    for(int i = 0; i < 4; i++)
    ori_dhat_LPF[0][i] = Filter.lowpassfilter(ori_dhat[0][i], ori_dhat[1][i], ori_dhat_LPF[1][i], Ori_DOB_cutoff[i]);
    
  }  
    
}


void Controller::setDelayData() {
  for (int i = 0; i < 1; i++) //[i][0] = z^0, [i][1] = z^1 ->  delay data 만들어 주는 function
  {
    /****************** Delay Data ******************/
    // r
    D_term[0][i + 1] = D_term[0][i];
    I_term[0][i + 1] = I_term[0][i];
    P_term[0][i + 1] = P_term[0][i];

    // th
    D_term[1][i + 1] = D_term[1][i];
    I_term[1][i + 1] = I_term[1][i];
    P_term[1][i + 1] = P_term[1][i];
  }
  
    // RWDOB
  for (int i = 0; i < 2; i ++)
  {
    lhs_dob_old[i] = lhs_dob[i];
    rhs_dob_old[i] = rhs_dob[i];
    lhs_dob_LPF_old[i] = lhs_dob_LPF[i];
    rhs_dob_LPF_old[i] = rhs_dob_LPF[i];
    
    T_fob[1] = T_fob[0];
    
  }
  
    
}

void Controller::exchange_mutex(int Leg_num) // Mutex에서 데이터를 받아오는 function.  Ui gain setting -> Mutex -> 여기 순서대로 이동 된 값임
{
  for (int i = 0; i < 4; i++) 
  { // 
    RW_r_posPgain[i] = _M_RW_r_posPgain[i];
    RW_r_posIgain[i] = _M_RW_r_posIgain[i];
    RW_r_posDgain[i] = _M_RW_r_posDgain[i];
    RW_r_posD_cutoff[i] = _M_RW_r_posD_cutoff[i];
    RW_th_posPgain[i] = _M_RW_th_posPgain[i];
    RW_th_posIgain[i] = _M_RW_th_posIgain[i];
    RW_th_posDgain[i] = _M_RW_th_posDgain[i];
    RW_th_posD_cutoff[i] = _M_RW_th_posD_cutoff[i];
    
    RW_r_velPgain[i] = _M_RW_r_velPgain[i];
    RW_r_velIgain[i] = _M_RW_r_velIgain[i];
    RW_r_velDgain[i] = _M_RW_r_velDgain[i];
    RW_r_velD_cutoff[i] = _M_RW_r_velD_cutoff[i];
    RW_th_velPgain[i] = _M_RW_th_velPgain[i];
    RW_th_velIgain[i] = _M_RW_th_velIgain[i];
    RW_th_velDgain[i] = _M_RW_th_velDgain[i];
    RW_th_velD_cutoff[i] = _M_RW_th_velD_cutoff[i];
    RWDOB_cutoff[i] = _M_RW_DOB_cutoff[i];
    RWFOB_cutoff[i] = _M_RW_FOB_cutoff[i]; 
  }
  
  r_vel_ref = _M_ref_r_vel;
  Ori_DOB_cutoff = _M_Ori_DOB_cutoff;
  _M_ori_dhat_LPF = ori_dhat_LPF[0];
  _M_tau_dhat[Leg_num] = tau_mutex_dhat[Leg_num];
  _M_forceExt_hat[Leg_num] = Extforce_mutex[Leg_num];

  //// Flag //// 
  RWDOB_on = _M_RWDOB_on;
  Ctrl_on = _M_Ctrl_on;
  OriDOB_on = _M_OriDOB_on;
}

void Controller::Exchagne_mutex()
{
  //// Trunk ////
  Trunk_ori_for_DOB[0] = _M_Trunk_ori[0];
  Trunk_ori_for_DOB[1] = _M_Trunk_ori[1];
  Ori_DOB_cutoff = _M_Ori_DOB_cutoff;
  _M_ori_dhat_LPF = ori_dhat_LPF[0];
  
  //// Leg ////
  r_vel_ref = _M_ref_r_vel;
  
  //// Flag //// 
  RWDOB_on = _M_RWDOB_on;
  OriDOB_on = _M_OriDOB_on;
  Ctrl_on = _M_Ctrl_on;
}

double Controller::get_posPgain(int Leg_num, int r0th1) {
    if (r0th1 == 0)
      return RW_r_posPgain[Leg_num];
    else
      return RW_th_posPgain[Leg_num];
  };
double Controller:: get_posIgain(int Leg_num, int r0th1) {
    if (r0th1 == 0)
      return RW_r_posIgain[Leg_num];
    else
      return RW_th_posIgain[Leg_num];
  };
double Controller::get_posDgain(int Leg_num, int r0th1) {
    if (r0th1 == 0)
      return RW_r_posDgain[Leg_num];
    else
      return RW_th_posDgain[Leg_num];
  };
double Controller::get_posD_cutoff(int Leg_num, int r0th1) {
    if (r0th1 == 0)
      return RW_r_posD_cutoff[Leg_num];
    else
      return RW_th_posD_cutoff[Leg_num];
  }; 
// ==================================velocity gain callback=============================================
double Controller::get_velPgain(int Leg_num, int r0th1) {
    if (r0th1 == 0)
      return RW_r_velPgain[Leg_num];
    else
      return RW_th_velPgain[Leg_num];
};
double Controller:: get_velIgain(int Leg_num, int r0th1) {
  if (r0th1 == 0)
    return RW_r_velIgain[Leg_num];
  else
    return RW_th_velIgain[Leg_num];
};
double Controller::get_velDgain(int Leg_num, int r0th1) {
  if (r0th1 == 0)
    return RW_r_velDgain[Leg_num];
  else
    return RW_th_velDgain[Leg_num];
};
double Controller::get_velD_cutoff(int Leg_num, int r0th1) {
  if (r0th1 == 0)
    return RW_r_velD_cutoff[Leg_num];
  else
    return RW_th_velD_cutoff[Leg_num];
};
  
