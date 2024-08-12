#include "actuator.h"
#include <iostream>
#include <stdio.h>
using namespace std;

Actuator::Actuator(int Motor_num, double motor_jig_pos) {
  Motor_Num = Motor_num; // /////////////////////////////////////////////// ZZi reum
//  Motor_pos = motor_init_pos;
  
  Motor_jig_pos = motor_jig_pos;
  
  QString ReadPath = "/home/mcl/Documents/RT_ECAT_MASTER_ELMO_GOLD/initial_encoder/";
  ReadPath.append(QString("Motor %1.txt").arg(Motor_Num));
  
  os_loadinit.open(ReadPath.toStdString());
  os_loadinit >> Motor_pos_offset;
  
  os_loadinit.close();
  
}

//void Actuator::DATA_reset() //
//{
//  for (int i = 0; i < NUMOFSLAVES; i++) //[i][0] = z^0, [i][1] = z^1.  2x1으로 1x1은 현재 값, 2x1은 이전 값인데 그값 초기화
//  {
//    Motor_pos = 0;
//    Motor_vel = 0;
//    Motor_torque = 0;
//  }
//}

void Actuator::DATA_Receive(input_GTWI_t** in_twitter_GTWI) //Motor_num은 class를 선언할 때 Ethercat 통신 순서대로 이미 정해줌
{

    //11-1-3.Data received from slave
    //Copy the data from received PDO
#ifdef SLAVE_GTWI
  
    position_raw = in_twitter_GTWI[Motor_Num]->TXPDO_ACTUAL_POSITION_DATA;
    velocity_raw = in_twitter_GTWI[Motor_Num]->TXPDO_ACTUAL_VELOCITY_DATA;
    torque_raw = in_twitter_GTWI[Motor_Num]->TXPDO_ACTUAL_TORQUE_DATA;
    statusword = in_twitter_GTWI[Motor_Num]->TXPDO_STATUSWORD_DATA;
    modeofOP_disp = in_twitter_GTWI[Motor_Num]->TXPDO_MODE_OF_OPERATION_DISPLAY_DATA;
    
    
    DATA_unit_change();

#endif

#ifdef SLAVE_PTWI


#endif



}

void Actuator::DATA_Send(output_GTWI_t** out_twitter_GTWI) //
{

#ifdef SLAVE_GTWI
    // Have to check what the multipling factors are.
    out_twitter_GTWI[Motor_Num]->RXPDO_TARGET_POSITION_DATA = (int32)(target_position * (Enc_resolution / (2 * M_PI)));
    out_twitter_GTWI[Motor_Num]->RXPDO_TARGET_POSITION_DATA_0 = (int32)(target_position * (Enc_resolution / (2 * M_PI)));
    out_twitter_GTWI[Motor_Num]->RXPDO_TARGET_VELOCITY_DATA = (int32)(target_speed * (Enc_resolution / (2 * M_PI)));
    if(safety_flag == true)
    {
      out_twitter_GTWI[Motor_Num]->RXPDO_TARGET_TORQUE_DATA = (int16)(target_torque * 1000000.0 / (45000 * Torque_constant * Gear_ratio)); // DS402 Maxon
      out_twitter_GTWI[Motor_Num]->RXPDO_CONTROLWORD_DATA = controlword;
      out_twitter_GTWI[Motor_Num]->RXPDO_CONTROLWORD_DATA_0 = controlword;
    }
    else
    {
      out_twitter_GTWI[Motor_Num]->RXPDO_TARGET_TORQUE_DATA = 0; // DS402 Maxon
      out_twitter_GTWI[Motor_Num]->RXPDO_CONTROLWORD_DATA = 0;
      out_twitter_GTWI[Motor_Num]->RXPDO_CONTROLWORD_DATA_0 = 0;
    }
    out_twitter_GTWI[Motor_Num]->RXPDO_DIGITAL_OUTPUTS_DATA = digital_output;
    out_twitter_GTWI[Motor_Num]->RXPDO_MAXIMAL_TORQUE = 1000; //?  
    out_twitter_GTWI[Motor_Num]->RXPDO_MODE_OF_OPERATION_DATA = modeOP;
    
//    if (Motor_Num == 0)
//    cout << (int16)(target_torque * 1000000.0 / (45000 * Torque_constant * Gear_ratio)) << endl;
#endif
}
// input current = 1000000/(45000*gear_ratio*torque_constant)


void Actuator::DATA_unit_change() {
  if(Motor_Num == 0 || Motor_Num == 3||Motor_Num == 4||Motor_Num == 6||Motor_Num == 8)
  {
    if(Motor_Num == 0||Motor_Num == 6)
    Motor_pos = -(double)(position_raw / Enc_resolution * 2 * M_PI) /50;
    else 
    Motor_pos = -(double)(position_raw / Enc_resolution * 2 * M_PI) /100;
  }
  else
  {
    if(Motor_Num == 5 || Motor_Num == 11)
      Motor_pos = (double)(position_raw / Enc_resolution * 2 * M_PI) /50;
    else 
      Motor_pos = (double)(position_raw / Enc_resolution * 2 * M_PI) /100;
  } 
    if(Enc_init == true)
    {
      initial_enc_pos = Motor_pos;
      Motor_pos_offset = Motor_pos - Motor_jig_pos;
      QString LoggingPath = "/home/mcl/Documents/RT_ECAT_MASTER_ELMO_GOLD/initial_encoder/";
      LoggingPath.append(QString("Motor %1.txt").arg(Motor_Num));
      os_saveinit.open(LoggingPath.toStdString());
      os_saveinit<<std::left<<Motor_pos_offset;

      os_saveinit.close();
      
    } 
    
    Motor_pos = Motor_pos - Motor_pos_offset;
    if(Motor_Num == 0 || Motor_Num == 3||Motor_Num == 4||Motor_Num == 6||Motor_Num == 8)
    {
      if(Motor_Num == 0||Motor_Num == 6)
      {
        Motor_vel = -(double)(velocity_raw / Enc_resolution * 2 * M_PI) /50;
        Motor_acc = Filter.tustin_derivate(&Motor_vel, &Motor_acc, Derivative_cutoff); 
      }
      else
      { 
        Motor_vel = -(double)(velocity_raw / Enc_resolution * 2 * M_PI) /100;
        Motor_acc = Filter.tustin_derivate(&Motor_vel, &Motor_acc, Derivative_cutoff); 
      }
    }
    else
    {
      if(Motor_Num == 5 || Motor_Num == 11)
      {
        Motor_vel = (double)(velocity_raw / Enc_resolution * 2 * M_PI)/50;
        Motor_acc = Filter.tustin_derivate(&Motor_vel, &Motor_acc, Derivative_cutoff); 
      }
      else
      { 
        Motor_vel = (double)(velocity_raw / Enc_resolution * 2 * M_PI) /100;
        Motor_acc = Filter.tustin_derivate(&Motor_vel, &Motor_acc, Derivative_cutoff); 
      }
    } 
    
    Motor_torque = (double)((double)torque_raw) * 45000 / 1000000;  
    
}

void Actuator::get_Derivative_cutoff()
{
  // Derivative cut off //
  if(Motor_Num == 1 || Motor_Num == 2) // FL
    Derivative_cutoff = _M_RW_DOB_cutoff[0];
  else if(Motor_Num == 3 || Motor_Num == 4) // FR
    Derivative_cutoff = _M_RW_DOB_cutoff[1];
  else if(Motor_Num == 9 || Motor_Num == 10) // RL
    Derivative_cutoff = _M_RW_DOB_cutoff[2];
  else if(Motor_Num == 7|| Motor_Num == 8) // RR
    Derivative_cutoff = _M_RW_DOB_cutoff[3];
  else
    Derivative_cutoff = 5;
}

void Actuator::safety()
{
  if(safety_on == true)
  {
    //HAA
    if(Motor_Num == 0 || Motor_Num == 5 || Motor_Num == 6 || Motor_Num == 11)
    {
      if(Motor_pos >= 0.2 || Motor_pos <= -0.5) 
      {
        cout << " Motor[" << Motor_Num << "] reach ROM!" << endl;  
        safety_flag = false;
      }
      else if(abs(Motor_torque) >= current_limit)
      {
        cout << " Motor[" << Motor_Num << "]'s current become greater than " << current_limit << endl;  
        safety_flag = false;
      }
      else
      safety_flag = true;
    }
    else if(Motor_Num == 12 || Motor_Num == 13)
    {
      if(Motor_pos <= -0.1 || Motor_pos >= 0.1) 
      {
        cout << " Motor[" << Motor_Num << "] reach ROM!" << endl;
        safety_flag = false;
      }
      else if(abs(Motor_torque) >= current_limit)
      {
        cout << " Motor[" << Motor_Num << "]'s current become greater than " << current_limit << endl;  
        safety_flag = false;
      }
      else
      safety_flag = true;    
    }
    //HIP, KNEE
    else
    {  
      if(Motor_pos <= 0.1 || Motor_pos >= M_PI - 0.1) 
      {
        cout << " Motor[" << Motor_Num << "] reach ROM!" << endl;
        safety_flag = false;
      }
      else if(abs(Motor_torque) >= current_limit)
      {
        cout << " Motor[" << Motor_Num << "]'s current become greater than " << current_limit << endl;  
        safety_flag = false;
      }
      else
      safety_flag = true;
    }
   }
}

void Actuator::exchange_mutex() {

    controlword = _M_CONTROLWORD[Motor_Num];
    modeOP = _M_MODE_OF_OPERATION[Motor_Num];
    target_torque = _M_motor_torque[Motor_Num];
  
    _M_motor_position[Motor_Num] = Motor_pos;
    _M_actual_current[Motor_Num] = Motor_torque;
    _M_safety_flag = safety_flag;
    safety_on = _M_safety_on;
    safety_flag = _M_safety_flag;
    
    current_limit = _M_current_limit;
    
    get_Derivative_cutoff();
    
    //// Flag ////
    Enc_init = _M_Enc_init;

}
