#include "trajectory.h"

trajectory::trajectory()
{
for(int i = 0 ; i<NUMOFSLAVES  ; i++ )
  {
    joint_c[i].j_set_gain(joint_kp[i],joint_ki[i],joint_kd[i]);
//    cout << "kd_c =" <<joint_c[i].j_get_posDgain()<< "kp_c =" <<joint_c[i].j_get_posPgain()<< "kI_c =" <<joint_c[i].j_get_posIgain()<<endl; 

  Trunk_ori_jacb << T_W/2, -T_L/2, -T_W/2, -T_L/2, T_W/2, T_L/2, -T_W/2, T_L/2;

  }

  
for(int i = 0; i < NUMOFSLAVES; i ++)
  {Homming_input[i] = 0;}
  
    //*************** Home Position ***************//
    Motor_Home_pos[0] = 0;
    Motor_Home_pos[1] = 0.7854; //HIP
    
    Motor_Home_pos[2] = 2.3562; //KNEE
    Motor_Home_pos[3] = 2.3562; //KNEE
    Motor_Home_pos[4] = 0.7854; //HIP
    Motor_Home_pos[5] = 0;
    Motor_Home_pos[6] = 0;
    Motor_Home_pos[7] = 0.7854;
    Motor_Home_pos[8] = 2.3562;
    Motor_Home_pos[9] = 2.3562;
    Motor_Home_pos[10] = 0.7854;
    Motor_Home_pos[11] = 0;
    Motor_Home_pos[12] = 0;
    Motor_Home_pos[13] = 0;
}

double* trajectory::homming()
{  
for(int i = 0 ; i<NUMOFSLAVES  ; i++ )
  {
   joint_c[i].j_setDelayData();
  }
    if(Homming_checked == true)
      {
        Homming_t++;
        if(Homming_t*0.001 < Homming_duration)
        { 
          for(int i = 0 ; i<NUMOFSLAVES; i++ )
          {
          
            Homming_traj[i] =  0.5*(Motor_Home_pos[i]-Motor_pos_init[i])*(1-cos(PI/Homming_duration*(Homming_t*0.001)))+Motor_pos_init[i];
            Homming_input[i] = joint_c[i].j_posPID(Homming_traj[i],Motor_pos[i],T,75);
//            if(i == 4)cout << "error"<< Homming_traj[i]-Motor_pos[i] <<"  desired  = "<< Homming_traj[i]<<  " current pos = " << Motor_pos[i]<<endl;
          }
        }
        else
        {
          for (int i = 0; i < NUMOFSLAVES; i ++)
            Homming_input[i] = joint_c[i].j_posPID(Motor_Home_pos[i],Motor_pos[i],T,75);  
        }
      }
  else{
        Homming_t = 0;
        for (int i = 0; i < NUMOFSLAVES; i ++) // initialize joint PID data for tunning.
          joint_c[i].j_reset();
        }
  
  return Homming_input;
}


void trajectory::leg_vel_traj(double t, int Leg_num)
{
 if(Trunk_Leg_CtrlMode == 0)
 {
 
  double f = 0.5;
  
  //Leg_Num = FL(0), FR(1), RL(2), RR(3)
  switch(Leg_num)
  {
    case 0: 
    // FL position trajectory
      ref_r_vel[0] = -0.005;//0.01*sin(2*PI*f*t);
      ref_th_vel[0] = 0;
    break;
  
    case 1: 
    // FR position trajectory
      ref_r_vel[1] = 0.2*sin(2*PI*f*t);//0.05*sin(2*PI*f*t) + 0.25;
      ref_th_vel[1] = 0;
    break;
      
    case 2: 
    // RL position trajectory
      ref_r_vel[2] = 0; //0.05*sin(2*PI*f*t) + 0.25;
      ref_th_vel[2] = 0;
    break;
      
    case 3:
    // RR position trajectory
      ref_r_vel[3] = 0;//0.05*sin(2*PI*f*t) + 0.25;
      ref_th_vel[3] = 0;
    break;      
    }
  }
}

void trajectory::Trunk_vel_traj(double t)
{
  if (Trunk_Leg_CtrlMode == 1)
  {
  ref_ori[0] = 0; // Alpha 
  ref_ori[1] = 0; // Beta
   
  ref_r_vel = Trunk_ori_jacb * ref_ori;
  
  if(OriDOB_on == true){ori_DOB_delta = ori_dhat_LPF;}
  else {ori_DOB_delta = Vector4d::Zero();}
  

    // FL position trajectory
      ref_r_vel[0] = ref_r_vel[0] + ori_DOB_delta[0];//0.01*sin(2*PI*f*t);
      ref_th_vel[0] = 0;

  
    // FR position trajectory
      ref_r_vel[1] = ref_r_vel[1] + ori_DOB_delta[1];//0.05*sin(2*PI*f*t) + 0.25;
      ref_th_vel[1] = 0;
//      cout << "Trunk_vel_traj: " << ori_DOB_delta << endl;
      
    // RL position trajectory
      ref_r_vel[2] = ref_r_vel[2] + ori_DOB_delta[2]; //0.05*sin(2*PI*f*t) + 0.25;
      ref_th_vel[2] = 0;
      
    // RR position trajectory
      ref_r_vel[3] = ref_r_vel[3] + ori_DOB_delta[3];//0.05*sin(2*PI*f*t) + 0.25;
      ref_th_vel[3] = 0;    
  }
  
}

void trajectory::exchange_mutex(int Leg_num)
{


    _M_ref_r_vel[Leg_num] = ref_r_vel[Leg_num];
    _M_ref_th_vel[Leg_num] = ref_th_vel[Leg_num];
//    _M_ref_r_pos[Leg_num] = ref_r_pos[Leg_num]; 
//    _M_ref_th_pos[Leg_num] = ref_th_pos[Leg_num];
    ori_dhat_LPF = _M_ori_dhat_LPF;

    OriDOB_on = _M_OriDOB_on; 
    Trunk_Leg_CtrlMode = _M_Trunk_Leg_CtrlMode;
  
  
}


void trajectory::Exchagne_mutex()
{

  //// flag
  Homming_checked = _M_Homming_checked;
  Homming_clicked = _M_Homming_clicked;  
  Trunk_Leg_CtrlMode = _M_Trunk_Leg_CtrlMode;
  _M_ref_r_vel = ref_r_vel;
          
  for(int i = 0; i < NUMOFSLAVES; i ++)
  {  
    Motor_pos_init[i] = _M_Motor_pos_init[i];
    Motor_pos[i] = _M_motor_position[i];
  }        
} 

