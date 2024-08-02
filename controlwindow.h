#ifndef CONTROLWINDOW_H
#define CONTROLWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QThread>
#include <QPixmap>
#include <algorithm>
#include <qcustomplot.h>
#include <stdio.h>
#include <data_mutex.h>

using namespace std;
QT_BEGIN_NAMESPACE
namespace Ui { class Controlwindow;}
QT_END_NAMESPACE

class Controlwindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit Controlwindow(QWidget *parent = nullptr);
    ~Controlwindow();
    void Mutex_exchange();
    void Widget_update();
    void createPlot_3data(QCustomPlot *plot);
    void drawPlot_3data(QCustomPlot *plot, double data1, double data2, double data3);
    void Enable();
    void homming();
    void ControlWord(uint16_t k);
//    Ui::Controlwindow *ui;
    Ui::Controlwindow *ui;    
private slots:
    void on_Set_clicked();
    void updateWindow();
    void on_init_encoder_clicked();
    void on_Homming_clicked();
    void on_STOP_clicked();
    
    void on_IMU_on_clicked();
    
  private:
    
    // Variable //
    Vector4d RW_r_posPgain {Vector4d::Zero(),};
    Vector4d RW_r_posIgain {Vector4d::Zero(),};
    Vector4d RW_r_posDgain {Vector4d::Zero(),};
    Vector4d RW_r_posD_cutoff = Vector4d{10,10,10,10};
    
    Vector4d RW_th_posPgain{Vector4d::Zero(),};
    Vector4d RW_th_posIgain{Vector4d::Zero(),};
    Vector4d RW_th_posDgain{Vector4d::Zero(),};
    Vector4d RW_th_posD_cutoff = Vector4d{10,10,10,10};
    
    Vector4d RW_r_velPgain{Vector4d::Zero(),};
    Vector4d RW_r_velIgain{Vector4d::Zero(),};
    Vector4d RW_r_velDgain{Vector4d::Zero(),};
    Vector4d RW_r_velD_cutoff = Vector4d{10,10,10,10};
    
    Vector4d RW_th_velPgain{Vector4d::Zero(),};
    Vector4d RW_th_velIgain{Vector4d::Zero(),}; 
    Vector4d RW_th_velDgain{Vector4d::Zero(),};
    Vector4d RW_th_velD_cutoff = Vector4d{10,10,10,10};
    
    double Motor_current[NUMOFSLAVES];
    
    // RW state //
    double Motor_pos[NUMOFSLAVES];    
    double Motor_pos_init[NUMOFSLAVES];
    double Homming_input[NUMOFSLAVES];
    double Motor_Home_pos[NUMOFSLAVES];
    
    double key;
    double Plot_time_window_POS = 10.0;
    uint16_t k;
    
    // Controller //
    double RW_DOB_cutoff[4] {4,};
    double RW_FOB_cutoff[4] {20,};    
    
    Vector4d Ori_DOB_cutoff = Vector4d{10,10,10,10};
    
    
    // flag //
    bool Traj_on = false;
    bool Ctrl_on = false;
    int CtrlMode;
    bool Enc_init = false;
    bool Homming_checked = false;
    bool Homming_clicked = false;
    bool DataLog_flag = false;
    bool stop = false;
    bool RWDOB_on = false;
    bool OriDOB_on = false;
    bool IMU_on = false;
    int Trunk_Leg_CtrlMode = 100;
    
    
};

#endif // CONTROLWINDOW_H
