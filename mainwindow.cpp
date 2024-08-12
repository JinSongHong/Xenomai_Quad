#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <pthread.h>
#include "data_mutex.h"

#include <QTimer>
#include <QThread>


QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);


/*           UI variable         */
double ref_pos[NUMOFSLAVES];
double ref_vel[NUMOFSLAVES];
double ref_current[NUMOFSLAVES];
double P_gain[NUMOFSLAVES];
double I_gain[NUMOFSLAVES];
double D_gain[NUMOFSLAVES];
double Target_torque[NUMOFSLAVES];


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  data_mut_lock_timeout.tv_nsec = timeout_ns;
  data_mut_lock_timeout.tv_sec = 0;

  tim = new QTimer(this);
  connect(tim, SIGNAL(timeout()), this, SLOT(updateWindow()));
  tim->start(50);
  
  //*************** Plot setting -> creating template***************//

    //FL
  createPlot(ui->FL_r_plot);
  createPlot(ui->FL_th_plot);
  createPlot(ui->FL_others); 
  createPlot_3data(ui->FL_current);
    //FR
  createPlot(ui->FR_r_plot);
  createPlot(ui->FR_th_plot);
  createPlot(ui->FR_others); 
  createPlot_3data(ui->FR_current);
    //RL
  createPlot(ui->RL_r_plot);
  createPlot(ui->RL_th_plot);
  createPlot(ui->RL_others); 
  createPlot_3data(ui->RL_current);
    //RR
  createPlot(ui->RR_r_plot);
  createPlot(ui->RR_th_plot);
  createPlot(ui->RR_others); 
  createPlot_3data(ui->RR_current); 
  
  //Trunk
  createPlot_3data(ui->Trunk_ang_vel);
  createPlot_3data(ui->Trunk_linear_vel);
    
    
       
  
  
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

void MainWindow::updateWindow()
{
    
  static QTime timeStart = QTime::currentTime();          // calculate two new data points:
  key = timeStart.msecsTo(QTime::currentTime()) / 1000.0; // time elapsed since start of demo, in seconds
  timeTicker->setTimeFormat("%m:%s");
  
    
  Widget_update();    
  Mutex_exchange();
  
  //*************** plot data ***************//

  // Trunk
  drawPlot_3data(ui->Trunk_ang_vel, Trunk_ori[0], Trunk_ori[1], Trunk_ori[2], ui->Trunk_ang_vel_autoscale);
  drawPlot_3data(ui->Trunk_linear_vel, Trunk_linear_vel[0], Trunk_linear_vel[1], Trunk_linear_vel[2], ui->Trunk_linear_vel_autoscale);

  //Motor Current
  drawPlot_3data(ui->FL_current, Motor_current[0], Motor_current[1], Motor_current[2],ui->FL_current_autoscale);
  ui->FL_current->yAxis->setRange(-45,45);  
  drawPlot_3data(ui->FR_current, Motor_current[5], Motor_current[4], Motor_current[3],ui->FR_current_autoscale);
  ui->FR_current->yAxis->setRange(-45,45);  
  drawPlot_3data(ui->RL_current, Motor_current[11], Motor_current[10], Motor_current[9],ui->RL_current_autoscale);
  ui->RL_current->yAxis->setRange(-45,45);
  drawPlot_3data(ui->RR_current, Motor_current[6], Motor_current[7], Motor_current[8],ui->RR_current_autoscale);
  ui->RR_current->yAxis->setRange(-45,45);
  
  
  if(ui->Select_data->currentIndex() == 0)
  {
 
 
    //FL
    drawPlot(ui->FL_r_plot, RW_r_pos[0], ref_r_pos[0], ui->FL_r_autoscale) ;
    ui->FL_r_plot->yAxis->setRange(0, 0.5);
    drawPlot(ui->FL_th_plot, RW_th_pos[0], ref_th_pos[0], ui->FL_th_autoscale);
    ui->FL_th_plot->yAxis->setRange(0, PI);

    //FR
    drawPlot(ui->FR_r_plot, RW_r_pos[1], ref_r_pos[1], ui->FR_r_autoscale) ;
    ui->FR_r_plot->yAxis->setRange(0, 0.5);
    drawPlot(ui->FR_th_plot, RW_th_pos[1], ref_th_pos[1], ui->FR_th_autoscale);
    ui->FR_th_plot->yAxis->setRange(0, PI);

    
    //RL
    drawPlot(ui->RL_r_plot, RW_r_pos[2], ref_r_pos[2], ui->RL_r_autoscale) ;
    ui->RL_r_plot->yAxis->setRange(0, 0.5);
    drawPlot(ui->RL_th_plot, RW_th_pos[2], ref_th_pos[2], ui->RL_th_autoscale);
    ui->RL_th_plot->yAxis->setRange(0, PI);
    
    //RR
    drawPlot(ui->RR_r_plot, RW_r_pos[3], ref_r_pos[3], ui->RR_r_autoscale) ;
    ui->RR_r_plot->yAxis->setRange(0, 0.5);
    drawPlot(ui->RR_th_plot, RW_th_pos[3], ref_th_pos[3], ui->RR_th_autoscale);
    ui->RR_th_plot->yAxis->setRange(0, PI);

  }
  else if(ui->Select_data->currentIndex() == 1)
  {
    //FL
    drawPlot(ui->FL_r_plot, RW_r_vel[0], ref_r_vel[0], ui->FL_r_autoscale) ;
    ui->FL_r_plot->yAxis->setRange(0, 0.5);
    drawPlot(ui->FL_th_plot, RW_th_vel[0], ref_th_vel[0], ui->FL_th_autoscale);
    ui->FL_th_plot->yAxis->setRange(0, PI);
    
    //FR
    drawPlot(ui->FR_r_plot, RW_r_vel[1], ref_r_vel[1], ui->FR_r_autoscale) ;
    ui->FR_r_plot->yAxis->setRange(0, 0.5);
    drawPlot(ui->FR_th_plot, RW_th_vel[1], ref_th_vel[1], ui->FR_th_autoscale);
    ui->FR_th_plot->yAxis->setRange(0, PI);
    
    //RL
    drawPlot(ui->RL_r_plot, RW_r_vel[2], ref_r_vel[2], ui->RL_r_autoscale) ;
    ui->RL_r_plot->yAxis->setRange(0, 0.5);
    drawPlot(ui->RL_th_plot, RW_th_vel[2], ref_th_vel[2], ui->RL_th_autoscale);
    ui->RL_th_plot->yAxis->setRange(0, PI);
    
    //RR
    drawPlot(ui->RR_r_plot, RW_r_vel[3], ref_r_vel[3], ui->RR_r_autoscale) ;
    ui->RR_r_plot->yAxis->setRange(0, 0.5);
    drawPlot(ui->RR_th_plot, RW_th_vel[3], ref_th_vel[3], ui->RR_th_autoscale);
    ui->RR_th_plot->yAxis->setRange(0, PI);
  }
  

  if(ui->Select_data_2->currentIndex() == 0)
  {
    drawPlot(ui->FL_others, tau_dhat[0][0], tau_dhat[0][1], ui->FL_others_autoscale);
    drawPlot(ui->FR_others, tau_dhat[1][0], tau_dhat[1][1], ui->FR_others_autoscale);
    drawPlot(ui->RL_others, tau_dhat[2][0], tau_dhat[2][1], ui->RL_others_autoscale);
    drawPlot(ui->RR_others, tau_dhat[3][0], tau_dhat[3][1], ui->RR_others_autoscale);
  }
  else if(ui->Select_data_2->currentIndex() == 1) 
  {
    drawPlot(ui->FL_others, forceExt_hat[0][0], forceExt_hat[0][1], ui->FL_others_autoscale);
    drawPlot(ui->FR_others, forceExt_hat[1][0], forceExt_hat[1][1], ui->FR_others_autoscale);
    drawPlot(ui->RL_others, forceExt_hat[2][0], forceExt_hat[2][1], ui->RL_others_autoscale);
    drawPlot(ui->RR_others, forceExt_hat[3][0], forceExt_hat[3][1], ui->RR_others_autoscale);
  }
  else if(ui->Select_data_2->currentIndex() == 2)
  {
    drawPlot(ui->FL_others, ori_dhat_LPF[0], ori_dhat_LPF[0], ui->FL_others_autoscale);
    drawPlot(ui->FR_others, ori_dhat_LPF[1], ori_dhat_LPF[1], ui->FR_others_autoscale);
    drawPlot(ui->RL_others, ori_dhat_LPF[2], ori_dhat_LPF[2], ui->RL_others_autoscale);
    drawPlot(ui->RR_others, ori_dhat_LPF[3], ori_dhat_LPF[3], ui->RR_others_autoscale);
  }
  else
  {
  
  }

}

void MainWindow::Widget_update(){
  


}

void MainWindow::drawPlot(QCustomPlot *plot, double data1, double data2, QCheckBox *autoscale){
  
  plot->graph(0)->addData(key,data1);
  plot->graph(1)->addData(key,data2);
  plot->xAxis->setRange(key,Plot_time_window_POS,Qt::AlignRight);


  if(autoscale->isChecked()) {
    plot->yAxis->rescale();
  }
  
  plot->graph(0)->data()->removeBefore(key-Plot_time_window_POS);
  plot->graph(1)->data()->removeBefore(key-Plot_time_window_POS);
  plot->replot();

}

void MainWindow::drawPlot_3data(QCustomPlot *plot, double data1, double data2, double data3, QCheckBox *autoscale){
  
  plot->graph(0)->addData(key,data1);
  plot->graph(1)->addData(key,data2);
  plot->graph(2)->addData(key,data3);
  plot->xAxis->setRange(key,Plot_time_window_POS,Qt::AlignRight);

  if(autoscale->isChecked()) {
    plot->yAxis->rescale();
  }
  
  plot->graph(0)->data()->removeBefore(key-Plot_time_window_POS);
  plot->graph(1)->data()->removeBefore(key-Plot_time_window_POS);
  plot->graph(2)->data()->removeBefore(key-Plot_time_window_POS);
  plot->replot();

}

void MainWindow::createPlot(QCustomPlot *plot) {
  plot->addGraph();
  plot->graph(0)->setPen(QPen(QColor(237, 237, 237)));
  plot->addGraph();
  plot->graph(1)->setPen(QPen(QColor(255, 246, 18)));
  plot->xAxis->setTicker(timeTicker);
  plot->axisRect()->setupFullAxesBox();
  connect(plot->xAxis, SIGNAL(rangeChanged(QCPRange)), plot->xAxis2, SLOT(setRange(QCPRange)));
  connect(plot->yAxis, SIGNAL(rangeChanged(QCPRange)), plot->yAxis2, SLOT(setRange(QCPRange)));
  plot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft | Qt::AlignTop);

  plot->xAxis->setBasePen(QPen(Qt::white, 1));
  plot->yAxis->setBasePen(QPen(Qt::white, 1));
  plot->xAxis->setTickPen(QPen(Qt::white, 1));
  plot->yAxis->setTickPen(QPen(Qt::white, 1));
  plot->xAxis->setSubTickPen(QPen(Qt::white, 1));
  plot->yAxis->setSubTickPen(QPen(Qt::white, 1));
  plot->xAxis->setTickLabelColor(Qt::white);
  plot->yAxis->setTickLabelColor(Qt::white);
  plot->xAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
  plot->yAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
  plot->xAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
  plot->yAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
  plot->xAxis->grid()->setSubGridVisible(true);
  plot->yAxis->grid()->setSubGridVisible(true);
  plot->xAxis->grid()->setZeroLinePen(Qt::NoPen);
  plot->yAxis->grid()->setZeroLinePen(Qt::NoPen);
  plot->setBackground(QColor(25, 35, 45));
  plot->axisRect()->setBackground(QColor(25, 35, 45));
}

void MainWindow::createPlot_3data(QCustomPlot *plot) {
  plot->addGraph();
  plot->graph(0)->setPen(QPen(QColor(237, 237, 237)));
  plot->addGraph();
  plot->graph(1)->setPen(QPen(QColor(255, 246, 18)));
  plot->addGraph();
  plot->graph(2)->setPen(QPen(QColor(255, 24, 18)));
  plot->xAxis->setTicker(timeTicker);
  plot->axisRect()->setupFullAxesBox();
  connect(plot->xAxis, SIGNAL(rangeChanged(QCPRange)), plot->xAxis2, SLOT(setRange(QCPRange)));
  connect(plot->yAxis, SIGNAL(rangeChanged(QCPRange)), plot->yAxis2, SLOT(setRange(QCPRange)));
  plot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft | Qt::AlignTop);

  plot->xAxis->setBasePen(QPen(Qt::white, 1));
  plot->yAxis->setBasePen(QPen(Qt::white, 1));
  plot->xAxis->setTickPen(QPen(Qt::white, 1));
  plot->yAxis->setTickPen(QPen(Qt::white, 1));
  plot->xAxis->setSubTickPen(QPen(Qt::white, 1));
  plot->yAxis->setSubTickPen(QPen(Qt::white, 1));
  plot->xAxis->setTickLabelColor(Qt::white);
  plot->yAxis->setTickLabelColor(Qt::white);
  plot->xAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
  plot->yAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
  plot->xAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
  plot->yAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
  plot->xAxis->grid()->setSubGridVisible(true);
  plot->yAxis->grid()->setSubGridVisible(true);
  plot->xAxis->grid()->setZeroLinePen(Qt::NoPen);
  plot->yAxis->grid()->setZeroLinePen(Qt::NoPen);
  plot->setBackground(QColor(25, 35, 45));
  plot->axisRect()->setBackground(QColor(25, 35, 45));
}


MainWindow::~MainWindow() { delete ui; }


void MainWindow::Mutex_exchange(){

    int ret = pthread_mutex_timedlock(&data_mut,&data_mut_lock_timeout);
    if (!ret) {
      //// Receive at GUI ////
      sampling_time_ms = _M_sampling_time_ms;
      overrun_cnt = _M_overrun_cnt;
      WKC = _M_Ecat_WKC;
      expectedWKC = _M_Ecat_expectedWKC;
      
     
      for (int i = 0; i < NUMOFSLAVES; i++) {
        
        Motor_pos[i] = _M_motor_position[i];
        //Motor State
        Motor_current[i] =  _M_actual_current[i];
      }
      
      for (int i = 0; i < NUMOFLEGS; i++) {

        //Leg State

        RW_r_pos[i] = _M_RW_r_pos[i];
        RW_th_pos[i] = _M_RW_th_pos[i];
        ref_r_pos[i] = _M_ref_r_pos[i];
        ref_th_pos[i] = _M_ref_th_pos[i];
        
        r_pos_error[i] = _M_r_pos_error[i];
        th_pos_error[i] = _M_th_pos_error[i];
      
        RW_r_vel[i] = _M_RW_r_vel[i];
        RW_th_vel[i] = _M_RW_th_vel[i];         
        ref_r_vel[i] = _M_ref_r_vel[i];
        ref_th_vel[i] = _M_ref_th_vel[i];
        
        tau_dhat[i] = _M_tau_dhat[i];      
        forceExt_hat[i] =_M_forceExt_hat[i]; 
      }
      
      IMU_on = _M_IMU_on;
      Trunk_ori = _M_Trunk_ori;
      Trunk_linear_vel = _M_Trunk_linear_vel;
      ori_dhat_LPF = _M_ori_dhat_LPF;
      
    } else {
      QString str_errCode;
      str_errCode.setNum(ret);
    }
    pthread_mutex_unlock(&data_mut);

    QString str_sampling_time_ms;
    QString str_overrun_cnt;
    QString str_WKC;
    QString str_expectedWKC;

    QString str_status_report;

    QString str_statusword;
    QString str_modeofoperation_disp;

    str_sampling_time_ms.clear();
    str_overrun_cnt.clear();
    str_WKC.clear();
    str_expectedWKC.clear();

    str_sampling_time_ms.setNum(sampling_time_ms,'f',4);
    str_sampling_time_ms.prepend("Real-time task: ");
    str_sampling_time_ms.append("\tms sampling with ");

    str_overrun_cnt.setNum(overrun_cnt);
    str_overrun_cnt.append("\ttimes overrun.\t\t");

    str_expectedWKC.setNum(expectedWKC);
    str_expectedWKC.prepend("EtherCAT data frame: Expected workcounter= ");
    str_expectedWKC.append(", ");
    str_WKC.setNum(WKC);
    str_WKC.prepend("Workcounter= ");

    str_status_report.clear();
    str_status_report.append(str_sampling_time_ms).append(str_overrun_cnt).append(str_expectedWKC).append(str_WKC);

    ui->statusbar->showMessage(str_status_report);
}


