#include<data_mutex.h>
#include<QTimer>

double _M_sampling_time_ms = 0; //sync(send) data from RT thread to GUI
int _M_overrun_cnt = 0;

uint16_t    _M_Ecat_states[NUMOFSLAVES+1+1];
int         _M_Ecat_WKC = 0;    //Actual Working Counter
int         _M_Ecat_expectedWKC = 0;    //Working Counter which is expected.

uint16_t    _M_STATUSWORD[NUMOFSLAVES];
int8_t      _M_MODE_OF_OPERATION_DISPLAY[NUMOFSLAVES];

int8_t     _M_MODE_OF_OPERATION[NUMOFSLAVES];
uint16_t    _M_CONTROLWORD[NUMOFSLAVES];

using std:: vector;

//// Parameter ////
double T = 0.001;


//// Motor parameter ////

double _M_motor_position[NUMOFSLAVES];
double _M_motor_torque[NUMOFSLAVES];
double _M_motor_velocity[NUMOFSLAVES];
double _M_ref_current[NUMOFSLAVES];

// actual current 
double _M_actual_current[NUMOFSLAVES];

//// Leg parameter (RW) ////
Vector4d _M_ref_r_pos;
Vector4d _M_ref_th_pos;
Vector4d _M_ref_r_vel;
Vector4d _M_ref_th_vel;

//// State in RW ////
Vector4d _M_RW_r_pos;
Vector4d _M_RW_th_pos;
Vector4d _M_RW_r_vel;
Vector4d _M_RW_th_vel;

//// RW error ////
Vector4d _M_r_pos_error;
Vector4d _M_th_pos_error;
Vector4d _M_r_vel_error;
Vector4d _M_th_vel_error;

//// Position PID Gain ////
Vector4d _M_RW_r_posIgain;
Vector4d _M_RW_r_posPgain;
Vector4d _M_RW_r_posDgain;
Vector4d _M_RW_r_posD_cutoff;

Vector4d _M_RW_th_posIgain;
Vector4d _M_RW_th_posPgain;
Vector4d _M_RW_th_posDgain;
Vector4d _M_RW_th_posD_cutoff;

//// Velocity PID Gain ////
Vector4d _M_RW_r_velIgain;
Vector4d _M_RW_r_velPgain;
Vector4d _M_RW_r_velDgain;
Vector4d _M_RW_r_velD_cutoff;

Vector4d _M_RW_th_velIgain;
Vector4d _M_RW_th_velPgain;
Vector4d _M_RW_th_velDgain;
Vector4d _M_RW_th_velD_cutoff;

//// DOB, FOB ////
Vector4d _M_RW_DOB_cutoff {6,6,6,6};
Vector4d _M_RW_FOB_cutoff {10,10,10,10};

Vector2d _M_tau_dhat[4];
Vector2d _M_forceExt_hat[4];

//// Orientation DOB ////
Vector4d _M_Ori_DOB_cutoff = Vector4d{10,10,10,10};
Vector4d _M_ori_dhat_LPF {Vector4d::Zero()};

//// IMU DATA ////
Vector3d _M_Trunk_ori {Vector3d::Zero(),}; 
Vector3d _M_Trunk_linear_vel {Vector3d::Zero(),}; 

double _M_Homming_input[NUMOFSLAVES];
double _M_Motor_pos_init[NUMOFSLAVES];



//// Mutex connect ////
timespec data_mut_lock_timeout;
int timeout_ns;
QTimer *tim;

//////////// Flag ////////////
int _M_Traj_ON;
bool _M_Ctrl_on;
int _M_ctrl_mode;
int _M_Trunk_Leg_CtrlMode;
bool _M_Enc_init;
bool _M_Homming_checked = false;
bool _M_Homming_clicked = false;
bool _M_DataLog_flag = false;
bool _M_stop = false;
bool _M_IMU_on = false;
bool _M_RWDOB_on = false;
bool _M_OriDOB_on = false;
bool _M_safety_on = false;
bool _M_safety_flag = true;

//// Safety ////
double _M_current_limit = 5;

