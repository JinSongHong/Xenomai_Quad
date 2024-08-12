////////////////////////////////////////////////
///Title: RT_Master_Bimanipulator_Platinum_v1 - main.cpp
///Functions: Multislave+Anybus
///Author: Copyright (C) 2022- Taehoon Kim
///Date: 2022.09.01
///Finished: 2022.09.02
////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////* INDEX *//////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////*

/// 1. INCLUDE
/// 2. MUTEX VARIABLE
/// 3. SETUP FOR RT THREAD
/// 4. DEFINITION FOR SOEM
/// 5. FUNCTION & VARIABLES DECLARATION
/// 6. MAIN FUNCTION
/// 7. FUNCTION DEFINITION
///     1) clean up
///     2) realtime_thread
///         (1) Parameter settingFound
///         (2) EherCAT MASTER
///         (3) Realtime loop
///             - Control loop




///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////* INCLUDE */////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///
#include "mainwindow.h"
#include "controlwindow.h"

#include <QApplication>

///* General includes */
#include <errno.h>    //Header for defining macros for reporting and retrieving error conditions using the symbol 'errno'
#include <error.h>    //
#include <fcntl.h>    //C POSIX lib header. Header for opening and locking files and processing other tasks.
#include <inttypes.h> //
#include <iostream>
#include <malloc.h> //Memory allocation
#include <math.h>
#include <pthread.h>  //Header for using Thread operation from xenomai pthread.h
#include <rtdm/ipc.h> //
#include <signal.h>   //Header for signal processing
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>    //
#include <sys/timerfd.h> //
#include <unistd.h>      //C POSIX lib header. Header for accessing to the POSIX OS API

///* Personal includes *///
//Have to write includes in CMakeLists.txt in order to fully
//include personal headers.

#include <data_logging.h>
#include <actuator.h>
#include <controller.h>
#include <data_exchange_mutex.h>
#include <data_mutex.h>
#include <ecat_func.h>
#include <ethercat.h>
#include <filter.h>
#include <kinematics.h>
#include <trajectory.h>
#include <sensor_data.h>
#include <qcustomplot.h>


using namespace std;


#ifndef PI
#define PI	(3.14159265359)
#define PI2	(6.28318530718)
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////* MUTEX VARIABLE *//////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

pthread_mutex_t data_mut = PTHREAD_MUTEX_INITIALIZER;


///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////* SETUP FOR RT THREAD *///////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

#define RT_PERIOD_MS 1 //1msec
//#define CPU_AFFINITY_NUM 0 //Up to {$ grep processor /proc/cpuinfo | wc -l} - 1, 0~3 for Mini PC
#define XDDP_PORT 0 //0~{CONFIG-XENO_OPT_PIPE_NRDEV-1} //XENO_OPT_PIPE_NRDEV: No. of pipe devices

pthread_t rt;
int sigRTthreadKill = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////* DEFINITION FOR SOEM *///////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

const char *IFNAME = "eno1"; //Checked from SOEM simpletest //char*->const char*: After C++11,
char IOmap[4096]; //
int usedmem; //

int expectedWKC; //Working Counter:
volatile int wkc; //Doesn't want to occupy memory location

//Gold Twitter
//Mensioned in ecat_func.h as extern
//{No. of entries to be mapped, Entry addr., Entry addr., ...}
uint16 RXPDO_ADDR_GTWI[3] = {2, 0x1600, 0x1605}; //
//uint16 TXPDO_ADDR_GTWI[5] = {4, 0x1A02, 0x1A03, 0x1A18, 0x1A1D};
//uint16 TXPDO_ADDR_GTWI[6] = {5, 0x1A02, 0x1A03, 0x1A18, 0x1A1D, 0x1A1E};
uint16 TXPDO_ADDR_GTWI[3] = {2, 0x1A02,0x1A11};

//Platinum Twitter
//uint16 RXPDO_ADDR_PTWI[2] = {1, 0x1600};
//uint16 TXPDO_ADDR_PTWI[2] = {1, 0x1A00};

//TS
uint16 TXPDO_ADDR_TS[4] = {3, 0x1A01, 0x1A02,0x1A03};

int a = 0 ;
double offset = 0;
double knee_pos = 0;
double knee_pos_error = 0; 
///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////* FUNCTION & VARIABLES DECLARATION */////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


static void cleanup(void); //Delete, release of all handle, memory
static void *realtime_thread(void *arg); //RT thread

///////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////* Class declaration *//////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
MainWindow* w;
Controlwindow* c;

data_logging* Logging;
sensor_data IMU;

trajectory traj;

Controller C_FL;
Controller C_FR;
Controller C_RL;
Controller C_RR;
Controller C_Trunk;

// Jacobian
Matrix2d J_FL;
Matrix2d J_FR;
Matrix2d J_RL;
Matrix2d J_RR;


Matrix2d JTrans_FL;
Matrix2d JTrans_FR;
Matrix2d JTrans_RL;
Matrix2d JTrans_RR;

// Controller output : RW PID INPUT//
Vector2d FL_output;
Vector2d FR_output;
Vector2d RL_output;
Vector2d RR_output;

// motor control input
Vector2d FL_control_input;
Vector2d FR_control_input;
Vector2d RL_control_input;
Vector2d RR_control_input;


// modification please
///////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////* actuator  *//////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
//Actuator actuator(motor_num, init_position);

//  actuator configuration: HAA, HIP, KNEE

Actuator ACT_RLHAA(11, 0);
Actuator ACT_RLHIP(10, 0.546812);
Actuator ACT_RLKNEE(9, 2.59478);

Actuator ACT_RRHAA(6, 0);
Actuator ACT_RRHIP(7, 0.546812);  
Actuator ACT_RRKNEE(8, 2.59478);

Actuator ACT_FRHAA(5, 0);
Actuator ACT_FRHIP(4, 0.546812);
Actuator ACT_FRKNEE(3, 2.59478);

Actuator ACT_FLHAA(0, 0);
Actuator ACT_FLHIP(1, 0.546812); // 31.345degree
Actuator ACT_FLKNEE(2, 2.59478); // 148.67degree 

Actuator ACT_WL(12,0);
Actuator ACT_WR(13,0);

//========================================================== 

Kinematics K_FL;
Kinematics K_FR;
Kinematics K_RL;
Kinematics K_RR;

trajectory Traj_FL;
trajectory Traj_FR;
trajectory Traj_RL;
trajectory Traj_RR;

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////* Variable declaration *////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

double constant = 1000000.0 / (Torque_constant*Gear_ratio * 45000);
int t = 0;
/**************trajecotry time for each legs**************/
int traj_t = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////* Flag *//////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

bool Traj_on = false;// 0 before initialized -> temporal stop flag
bool Homming = false;
bool Ctrl_on = false;
bool stop = false;
bool IMU_on = false;
bool safety_flag = true;

///////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////* Mode selcetion*//////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

int ctrl_mode = 0;

// Homming
double* Homming_input;

// posRW
Vector2d posRW_FL;
Vector2d posRW_FR;
Vector2d posRW_RL;
Vector2d posRW_RR;

Vector2d posRW_err_FL;
Vector2d posRW_err_FR;
Vector2d posRW_err_RL;
Vector2d posRW_err_RR;

Vector2d posRW_err_old_FL;
Vector2d posRW_err_old_FR;
Vector2d posRW_err_old_RL;
Vector2d posRW_err_old_RR;


// velRW
Vector2d velRW_FL;
Vector2d velRW_FR;
Vector2d velRW_RL;
Vector2d velRW_RR;

Vector2d velRW_err_FL;
Vector2d velRW_err_FR;
Vector2d velRW_err_RL;
Vector2d velRW_err_RR;

Vector2d velRW_err_old_FL;
Vector2d velRW_err_old_FR;
Vector2d velRW_err_old_RL;
Vector2d velRW_err_old_RR;



///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////* MAIN FUNCTION *///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {
  

  mlockall(MCL_CURRENT | MCL_FUTURE); // Lock the memory page to prevent performance degrade

  pthread_attr_t rtattr, regattr; //
  sigset_t set;
  int sig;
  cpu_set_t cpus;
  int cpu_num = 0;

  sigemptyset(&set); //
  sigaddset(&set, SIGINT);
  sigaddset(&set, SIGTERM);
  sigaddset(&set, SIGHUP);
  pthread_sigmask(SIG_BLOCK, &set, NULL); // to send the system signal into the thread, not implemented in the thread yet.

  ////* THREAD SETTING *////
  struct sched_param p;
  int ret;

  ret = pthread_attr_init(&rtattr); // initialize pthread attribute
  if (ret)
    error(1, ret, "pthread_attr_init()");

  ret = pthread_attr_setinheritsched(&rtattr, PTHREAD_EXPLICIT_SCHED); // pthread scheduling inherit setting as explicit
  if (ret)
    error(1, ret, "pthread_attr_setinheritsched()");

  ret = pthread_attr_setschedpolicy(&rtattr, SCHED_FIFO); // pthread scheduling policy setting as FIFO
  if (ret)
    error(1, ret, "pthread_attr_setschedpolicy()");

  p.sched_priority = 99;
  ret = pthread_attr_setschedparam(&rtattr, &p); // setting scheduler parameter - priority 99 (Highest)
  if (ret)
    error(1, ret, "pthread_attr_setschedparam()");

  CPU_ZERO(&cpus);
  CPU_SET(cpu_num, &cpus);
  ret = pthread_attr_setaffinity_np(&rtattr, sizeof(cpus), &cpus); // give cpu affinity to be used to calculate for the RT thread
  if (ret)
    error(1, ret, "pthread_attr_setaffinity_np()");

  ret = pthread_create(&rt, &rtattr, realtime_thread, NULL); // create RT thread
  if (ret)
    error(1, ret, "pthread_create(realtime_thread)");

  pthread_attr_destroy(&rtattr); // delete pthread attribute union

  QApplication a(argc, argv);
  w = new MainWindow;
  c = new Controlwindow;
  Logging = new data_logging(c);
  
  
  w->show();
  c->show();
  

  ret = a.exec();      // Execute the mainwindow thread and return when GUI is terminated
  sigRTthreadKill = 1; // set(send) the kill signal to the RT thread

  usleep(2000); // wait time for exit the thread
  cleanup();    // clean up the thread

  return ret;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////* FUNCTION DEFINITION */////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

static void cleanup(void)
{
     pthread_cancel(rt); //thread end
     pthread_join(rt, NULL); //wait for thread is ended
}

static void *realtime_thread(void *arg)
{

#ifdef SLAVE_GTWI
     output_GTWI_t *out_twitter_GTWI[NUMOFSLAVES];    //RxPDO mapping data (output to the slaves)
     input_GTWI_t *in_twitter_GTWI[NUMOFSLAVES];      //TxPDO mapping data (input from the slaves)
#endif

///////////////////////////////////////* PARAMETER SETING */////////////////////////////////////

// EtherCAT
    bool ecatconfig_success = false;
    int chk;

// RT thread
    struct timespec trt;
    struct itimerspec timer_conf;
    struct timespec expected;

    long t1 = 0;
    long t2 = 0;
    long old_t1 = 0;
    long delta_t1 = 0;
    long t_before_sample_start = 0;
    double sampling_ms = 0;
    int tfd;

    uint32_t overrun = 0;
    uint64_t ticks;


///////////////////////////////////////* EherCAT MASTER */////////////////////////////////////

// 1. Initialize EtherCAT Master(Init)

    if(ecat_init(IFNAME)) //If there is any error, ecat_init returns '0'
        //There is no error with ecat_init
        sigRTthreadKill = 0;

    else
        //There is error
        sigRTthreadKill = 1;

// 2. EtherCAT slave number check

    #ifdef NON_SLAVE_TS
    if(ec_slavecount == NUMOFSLAVES)
    {
        sigRTthreadKill = 0;
        //for(int i=1;i<=NUMOFSLAVES-1;i++)
    #ifdef SLAVE_GTWI                                          // SLAVE pdomapping setting /////////////////////////////////////
            for(int i=1;i<=NUMOF_GTWI_SLAVES;i++)
            {
                //PO: Pre-Operation, SO: Safe-Operation
                //Link slave's specific setups to PO -> SO
                ec_slave[i].PO2SOconfig = ecat_PDO_Config_GTWI; //Doesn't this function need argument?
                                                           //=> PO2SOconfig is also function.
            }
    #endif
    #ifdef SLAVE_PTWI
            for(int i=NUMOF_GTWI_SLAVES+1;i<=NUMOF_GTWI_SLAVES+NUMOF_PTWI_SLAVES;i++)
            {
                //PO: Pre-Operation, SO: Safe-Operation
                //Link slave's specific setups to PO -> SO
                ec_slave[i].PO2SOconfig = ecat_PDO_Config_PTWI; //Doesn't this function need argument?
                                                           //=> PO2SOconfig is also function.
            }
    #endif

    }
    else
        sigRTthreadKill = 1;
    #endif


// 3. PDO mapping

    ec_config_overlap_map(&IOmap); //Map all PDOs from slaves to IOmap with Outputs/Inputs in sequential order.
    //    ec_config_map(&IOmap);

// 4. Setting Distributed Clock

    int8 dc = ec_configdc(); //Returns ecx_configdc(ecx_contextt *context)=>returns boolean if slaves are found with DC

// 5. Change all slaves pre-OP to SafeOP.

    //ec_statecheck(slave number(0:all slaves), Requested state, Timeout value in microsec)
    ec_statecheck(0, EC_STATE_SAFE_OP, 4*EC_TIMEOUTSTATE); //EC_TIMEOUTSTATE=2,000,000 us

// 6. Calculate expected WKC

    expectedWKC = (ec_group[0].outputsWKC *2) + ec_group[0].inputsWKC; //Calculate WKCs

// 7. Change MASTER state to operational

    ec_slave[0].state = EC_STATE_OPERATIONAL;

// 8. Send one valid process data(Execute PDO communication once)

    ec_send_overlap_processdata(); //Send processdata to slave.
    //    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET); //Receive processdata from slaves.


// 9. Request OP state for all slaves

    ec_writestate(0); // Write slave state.

    // 10. PDO data Receive/Transmit

    uint32 obytes = ec_slave[1].Obytes; // Obytes: Output bytes
    uint32 ibytes = ec_slave[1].Ibytes; // Ibytes: Input bytes

    for (int i = 0; i < NUMOFSLAVES; i++) // 0 does not mean master in here
    {
        if (i < NUMOF_GTWI_SLAVES) {
#ifdef SLAVE_GTWI
                // The reason why ec_slave[]has i+1 not i, because ec_slave[0] represents master
                out_twitter_GTWI[i] = (output_GTWI_t *)ec_slave[i + 1].outputs;
                in_twitter_GTWI[i] = (input_GTWI_t *)ec_slave[i + 1].inputs;
#endif
        } else {
#ifdef SLAVE_PTWI
                out_twitter_PTWI[i] = (output_PTWI_t *)ec_slave[i + 1].outputs;
                in_twitter_PTWI[i] = (input_PTWI_t *)ec_slave[i + 1].inputs;
#endif
        }
    }
#ifdef SLAVE_TS
    in_twitter_ts[0] = (input_TS_t *)ec_slave[1].inputs;
#endif

    // 11. Real time

    clock_gettime(CLOCK_MONOTONIC, &expected); //get the system's current time

    tfd = timerfd_create(CLOCK_MONOTONIC, 0); //create timer descriptor
    if(tfd == -1) error(1, errno, "timerfd_create()");

    timer_conf.it_value = expected; // from now
    timer_conf.it_interval.tv_sec = 0;
    timer_conf.it_interval.tv_nsec = RT_PERIOD_MS*1000000; //interval with RT_PERIOD_MS

    int err = timerfd_settime(tfd, TFD_TIMER_ABSTIME, &timer_conf, NULL); //set the timer descriptor
    if(err) error(1, errno, "timerfd_setting()");

    usleep(1000);

///////////////////////////////////////* REALTIME LOOP */////////////////////////////////////

    while(!sigRTthreadKill)
    {
    // 11-1
        clock_gettime(CLOCK_REALTIME, &trt); //get the system time
        t_before_sample_start = trt.tv_nsec;

        err = read(tfd, &ticks, sizeof(ticks)); //read timer and return when the defined interval is reached (periodic)
        clock_gettime(CLOCK_REALTIME, &trt); //get the system time
        if(err<0) error(1, errno, "read()"); //

        old_t1 = t1;
        t1 = trt.tv_nsec;
        if(old_t1>trt.tv_nsec)
        {
            delta_t1 = (t1+1000000000) - old_t1;
        }
        else
        {
            delta_t1 = t1 - old_t1;
        }

        sampling_ms = (double)delta_t1*0.000001; //calculating time interval

        double jitter = sampling_ms - RT_PERIOD_MS; //calculating jitter

        if(ticks>1) overrun += ticks - 1; //calculating total overrun

    // 11-2. Sending processdata (calculated one tick before) => In order to ensure punctuality

        ec_send_overlap_processdata(); //PDO sending
        // ec_send_processdata();

        wkc = ec_receive_processdata(EC_TIMEOUTRET); //returns WKC

        if(expectedWKC>wkc)
        //In case of checked wkc less than WKC that have to be right
        //This means the etherCAT frame cannot be successfully read or
        //wrote on at least one slaves.
        //Up to user
        {
            sigRTthreadKill = 1; //Kill the entire of the RT thread
        }





    // 11-4. Control loop
//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////* Control loop start *///////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

        t++;
        
        
        a ++ ; 
       
        
        /**************** Data receive from ELMO ***************/
        
        ACT_FLHAA.DATA_Receive(in_twitter_GTWI);    ACT_FLHIP.DATA_Receive(in_twitter_GTWI);    ACT_FLKNEE.DATA_Receive(in_twitter_GTWI);   ACT_WL.DATA_Receive(in_twitter_GTWI);
        ACT_FRHAA.DATA_Receive(in_twitter_GTWI);    ACT_FRHIP.DATA_Receive(in_twitter_GTWI);    ACT_FRKNEE.DATA_Receive(in_twitter_GTWI);   ACT_WR.DATA_Receive(in_twitter_GTWI);
        ACT_RLHAA.DATA_Receive(in_twitter_GTWI);    ACT_RLHIP.DATA_Receive(in_twitter_GTWI);    ACT_RLKNEE.DATA_Receive(in_twitter_GTWI);
        ACT_RRHAA.DATA_Receive(in_twitter_GTWI);    ACT_RRHIP.DATA_Receive(in_twitter_GTWI);    ACT_RRKNEE.DATA_Receive(in_twitter_GTWI);
    
        /**************** Controller DATA set : update ***************/
        C_FL.setDelayData();
        C_FR.setDelayData();
        C_RL.setDelayData();
        C_RR.setDelayData();
        
        /**************** Kinematic error update ***************/
        K_FL.set_DelayDATA();
        K_FR.set_DelayDATA();
        K_RL.set_DelayDATA();
        K_RR.set_DelayDATA();
        
        /************* Parameter Calculation *************/
        C_FL.Cal_Parameter(ACT_FLHIP.getMotor_pos(), ACT_FLKNEE.getMotor_pos(), ACT_FLHIP.getMotor_vel(), ACT_FLKNEE.getMotor_vel());
        C_FR.Cal_Parameter(ACT_FRHIP.getMotor_pos(), ACT_FRKNEE.getMotor_pos(), ACT_FRHIP.getMotor_vel(), ACT_FRKNEE.getMotor_vel());
        C_RL.Cal_Parameter(ACT_RLHIP.getMotor_pos(), ACT_RLKNEE.getMotor_pos(), ACT_RLHIP.getMotor_vel(), ACT_RLKNEE.getMotor_vel());
        C_RR.Cal_Parameter(ACT_RRHIP.getMotor_pos(), ACT_RRKNEE.getMotor_pos(), ACT_RRHIP.getMotor_vel(), ACT_RRKNEE.getMotor_vel());
        
        /****************** Kinematics ******************/
        K_FL.Cal_RW(ACT_FLHIP.getMotor_pos(), ACT_FLKNEE.getMotor_pos(), ACT_FLHIP.getMotor_vel(), ACT_FLKNEE.getMotor_vel(), 0);
        K_FR.Cal_RW(ACT_FRHIP.getMotor_pos(), ACT_FRKNEE.getMotor_pos(), ACT_FRHIP.getMotor_vel(), ACT_FRKNEE.getMotor_vel(), 1);
        K_RL.Cal_RW(ACT_RLHIP.getMotor_pos(), ACT_RLKNEE.getMotor_pos(), ACT_RLHIP.getMotor_vel(), ACT_RLKNEE.getMotor_vel(), 2);
        K_RR.Cal_RW(ACT_RRHIP.getMotor_pos(), ACT_RRKNEE.getMotor_pos(), ACT_RRHIP.getMotor_vel(), ACT_RRKNEE.getMotor_vel(), 3);
          
        J_FL = K_FL.get_RW_Jacobian();  JTrans_FL = K_FL.get_RW_Jacobian_Trans();
        J_FR = K_FR.get_RW_Jacobian();  JTrans_FR = K_FR.get_RW_Jacobian_Trans();
        J_RL = K_RL.get_RW_Jacobian();  JTrans_RL = K_RL.get_RW_Jacobian_Trans();
        J_RR=  K_RR.get_RW_Jacobian();  JTrans_RR = K_RR.get_RW_Jacobian_Trans();

        /****************** Trajectory ******************/
        
        K_FL.pos_trajectory(traj_t, 0, C_FL.admittance(70,1,1000)); 
        K_FR.pos_trajectory(traj_t, 1, C_FR.admittance(30,1,5000)); 
        K_RL.pos_trajectory(traj_t, 2, C_RL.admittance(30,5,5000)); 
        K_RR.pos_trajectory(traj_t, 3, C_RR.admittance(30,5,5000)); 
        
        K_FL.vel_trajectory(traj_t, 0); 
        K_FR.vel_trajectory(traj_t, 1); 
        K_RL.vel_trajectory(traj_t, 2); 
        K_RR.vel_trajectory(traj_t, 3); 
        
        
        Traj_FL.leg_vel_traj(traj_t*0.001, 0);
        Traj_FR.leg_vel_traj(traj_t*0.001, 1);
        Traj_RL.leg_vel_traj(traj_t*0.001, 2);
        Traj_RR.leg_vel_traj(traj_t*0.001, 3);
        
        traj.Trunk_vel_traj(traj_t*0.001);
        
        if(Traj_on) // button stop -> stop time temporally, 
        traj_t += 1 ; 
        
          
        /****************** State ******************/ // pos RW
        //Admittance -> double, K_FL.get_posRW() ->vector2d
        posRW_FL = K_FL.get_posRW();    velRW_FL = K_FL.get_velRW();
        posRW_FR = K_FR.get_posRW();    velRW_FR = K_FR.get_velRW();
        posRW_RL = K_RL.get_posRW();    velRW_RL = K_RL.get_velRW();
        posRW_RR = K_RR.get_posRW();    velRW_RR = K_RR.get_velRW(); 
        
        /****************** Conrtoller ******************/ // index [0] : R direction output, index [1] : th direction output 
        if(Ctrl_on == true && safety_flag == true){
          FL_output[0] = C_FL.pid(K_FL.get_posRW_error(0), K_FL.get_posRW_error(1), K_FL.get_velRW_error(0), K_FL.get_velRW_error(1), 0, 0, ctrl_mode); //ctrl_mode = 0 -> pos control
          FL_output[1] = C_FL.pid(K_FL.get_posRW_error(0), K_FL.get_posRW_error(1), K_FL.get_velRW_error(0), K_FL.get_velRW_error(1), 1, 0, ctrl_mode); //ctrl_mode = 1 -> vel control
          
          FR_output[0] = C_FR.pid(K_FR.get_posRW_error(0), K_FR.get_posRW_error(1), K_FR.get_velRW_error(0), K_FR.get_velRW_error(1), 0, 1, ctrl_mode);
          FR_output[1] = C_FR.pid(K_FR.get_posRW_error(0), K_FR.get_posRW_error(1), K_FR.get_velRW_error(0), K_FR.get_velRW_error(1), 1, 1, ctrl_mode);

          
          RL_output[0] = C_RL.pid(K_RL.get_posRW_error(0), K_RL.get_posRW_error(1), K_RL.get_velRW_error(0), K_RL.get_velRW_error(1), 0, 2, ctrl_mode); // R direction output
          RL_output[1] = C_RL.pid(K_RL.get_posRW_error(0), K_RL.get_posRW_error(1), K_RL.get_velRW_error(0), K_RL.get_velRW_error(1), 1, 2, ctrl_mode); // th direction output
                
          RR_output[0] = C_RR.pid(K_RR.get_posRW_error(0), K_RR.get_posRW_error(1), K_RR.get_velRW_error(0), K_RR.get_velRW_error(1), 0, 3, ctrl_mode);
          RR_output[1] = C_RR.pid(K_RR.get_posRW_error(0), K_RR.get_posRW_error(1), K_RR.get_velRW_error(0), K_RR.get_velRW_error(1), 1, 3, ctrl_mode);
        
          C_Trunk.orientation_DOB();
        }
        else{
        for(int i = 0; i < 2; i++)
          {
            FL_output[i] = 0;
            FR_output[i] = 0;
            RL_output[i] = 0;
            RR_output[i] = 0;
          } 
        }
        
        /////////////////////// RWFOB //////////////////////
        C_FL.RWFOB(FL_control_input, JTrans_FL, ACT_FLHIP.getMotor_acc(), ACT_FLKNEE.getMotor_acc(), 0);
        C_FR.RWFOB(FR_control_input, JTrans_FR, ACT_FRHIP.getMotor_acc(), ACT_FRKNEE.getMotor_acc(), 1);
        C_RL.RWFOB(RL_control_input, JTrans_RL, ACT_RLHIP.getMotor_acc(), ACT_RLKNEE.getMotor_acc(), 2);
        C_RR.RWFOB(RR_control_input, JTrans_RR, ACT_RRHIP.getMotor_acc(), ACT_RRKNEE.getMotor_acc(), 3);

        /****************** Put the torque in Motor ******************/
        
        FL_control_input = JTrans_FL * FL_output + C_FL.RWDOB(ACT_FLHIP.getMotor_acc(), ACT_FLKNEE.getMotor_acc(), FL_control_input, J_FL, 0); 
        FR_control_input = JTrans_FR * FR_output + C_FR.RWDOB(ACT_FRHIP.getMotor_acc(), ACT_FRKNEE.getMotor_acc(), FR_control_input, J_FR, 1);    
        RL_control_input = JTrans_RL * RL_output + C_RL.RWDOB(ACT_RLHIP.getMotor_acc(), ACT_RLKNEE.getMotor_acc(), RL_control_input, J_RL, 2);
        RR_control_input = JTrans_RR * RR_output + C_RR.RWDOB(ACT_RRHIP.getMotor_acc(), ACT_RRKNEE.getMotor_acc(), RR_control_input, J_RR, 3);
        

        
        Homming_input = traj.homming();

        
        /****************** Mutex exchange ******************/
        
        if (!pthread_mutex_trylock(&data_mut)) 
        {
          C_FL.exchange_mutex(0); C_FR.exchange_mutex(1); C_RL.exchange_mutex(2); C_RR.exchange_mutex(3); 
          
          C_Trunk.Exchagne_mutex();
          
          K_FL.exchange_mutex(0); K_FR.exchange_mutex(1); K_RL.exchange_mutex(2); K_RR.exchange_mutex(3);
          
          Traj_FL.exchange_mutex(0); Traj_FR.exchange_mutex(1); Traj_RL.exchange_mutex(2); Traj_RR.exchange_mutex(3);
          
          ACT_FLHAA.exchange_mutex();   ACT_FLHIP.exchange_mutex();   ACT_FLKNEE.exchange_mutex();   ACT_WL.exchange_mutex();
          ACT_FRHAA.exchange_mutex();   ACT_FRHIP.exchange_mutex();   ACT_FRKNEE.exchange_mutex();   ACT_WR.exchange_mutex();
          ACT_RLHAA.exchange_mutex();   ACT_RLHIP.exchange_mutex();   ACT_RLKNEE.exchange_mutex();
          ACT_RRHAA.exchange_mutex();   ACT_RRHIP.exchange_mutex();   ACT_RRKNEE.exchange_mutex();

          pthread_mutex_unlock(&data_mut);
        }
        

                /****************** actuator Data send to ELMO ******************/
        
        ACT_FLHAA.DATA_Send(out_twitter_GTWI);   ACT_FLHIP.DATA_Send(out_twitter_GTWI);   ACT_FLKNEE.DATA_Send(out_twitter_GTWI);   ACT_WL.DATA_Send(out_twitter_GTWI);
        ACT_FRHAA.DATA_Send(out_twitter_GTWI);   ACT_FRHIP.DATA_Send(out_twitter_GTWI);   ACT_FRKNEE.DATA_Send(out_twitter_GTWI);   ACT_WR.DATA_Send(out_twitter_GTWI);
        ACT_RLHAA.DATA_Send(out_twitter_GTWI);   ACT_RLHIP.DATA_Send(out_twitter_GTWI);   ACT_RLKNEE.DATA_Send(out_twitter_GTWI);
        ACT_RRHAA.DATA_Send(out_twitter_GTWI);   ACT_RRHIP.DATA_Send(out_twitter_GTWI);   ACT_RRKNEE.DATA_Send(out_twitter_GTWI);
        

        // 11-6. Sync data with GUI thread

        
        if (stop == true || safety_flag == false)
        {
          for(int i = 0; i < 2; i++){
          FL_control_input[i] = 0;
          FR_control_input[i] = 0;
          RL_control_input[i] = 0;
          RR_control_input[i] = 0;
          }
        }        
  
        //// Data Logging ////
        Logging->data_log();
        
        //// IMU ////
        IMU.Init_IMU();
        
        //// SAFETY ////
        ACT_FLHAA.safety();   ACT_FLHIP.safety();   ACT_FLKNEE.safety();   ACT_WL.safety();
        ACT_FRHAA.safety();   ACT_FRHIP.safety();   ACT_FRKNEE.safety();   ACT_WR.safety();
        ACT_RLHAA.safety();   ACT_RLHIP.safety();   ACT_RLKNEE.safety();
        ACT_RRHAA.safety();   ACT_RRHIP.safety();   ACT_RRKNEE.safety();
        
        
        if(!pthread_mutex_trylock(&data_mut))
        {   
            IMU.get_IMU_data();
            IMU.exchange_mutex();
            traj.Exchagne_mutex();
//            traj.exchange_mutex(0);
            Logging->exchange_mutex();
            
            _M_sampling_time_ms = sampling_ms; //when the thread get the mutex, write data into shared global variables
            _M_overrun_cnt = overrun;

            _M_Ecat_WKC = wkc;
            _M_Ecat_expectedWKC = expectedWKC;
            
            /****************** Motor Torque ******************/
              if(!Homming)
              {
                _M_motor_torque[0] = -Homming_input[0]*constant * 2; // - constant*2;
                _M_motor_torque[1] = (FL_control_input[0])*constant;
                _M_motor_torque[2] = FL_control_input[1]*constant;
                
  
                _M_motor_torque[5] = Homming_input[5]*constant * 2;
                _M_motor_torque[4] = (-FR_control_input[0])*constant;
                _M_motor_torque[3] = (-FR_control_input[1])*constant;
   
               
                _M_motor_torque[6] = -Homming_input[6]*constant * 2;
                _M_motor_torque[7] = RR_control_input[0]*constant;
                _M_motor_torque[8] = -RR_control_input[1]*constant;
                
                _M_motor_torque[11] = Homming_input[11]*constant * 2;
                _M_motor_torque[10] = RL_control_input[0]*constant;
                _M_motor_torque[9] = RL_control_input[1]*constant;
                
                
                _M_motor_torque[13] = Homming_input[13]*constant;
              }
              else
              {
                                                
                                
                for(int i = 0; i < NUMOFSLAVES; i++)
                { 
                  if(i == 0 || i == 6)
                  _M_motor_torque[i] = -Homming_input[i]*constant * 2;
                  
                  else if( i == 5 || i == 11)
                  _M_motor_torque[i] = Homming_input[i]*constant * 2;
                  
                  
                  else if( i == 3 || i == 4 || i == 8) // 3, 4
                  {
                    _M_motor_torque[i] = -Homming_input[i]*constant ;
                  }
                  else
                    _M_motor_torque[i] = Homming_input[i]*constant ;
                  
                }
                  
                
              }
              
            /*********************** Flag **********************/  
            Homming = _M_Homming_checked;
            Traj_on = _M_Traj_ON; // temp_stop pressed 
            Ctrl_on = _M_Ctrl_on;
            stop = _M_stop;   
            safety_flag = _M_safety_flag;
                    
            /***************** Mode selection ******************/
            ctrl_mode = _M_ctrl_mode;
            
            /********************* Homming *********************/
 
            pthread_mutex_unlock(&data_mut);
        }
        t2 = trt.tv_nsec;

    }
    pthread_exit(NULL);
    return NULL;

    //////////////////////////////////////////////////////////////////////////////////////////////////

}
