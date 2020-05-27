#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include"globaldata.h"
#include"dealusb_msg.h"
#include"savepcdthread.h"
#include"statisticsdialog.h"
#include"filesave_dialog.h"
#include"savepcdthread.h"
#include"calmeanstdthread.h"
#include"receusb_msg.h"
#include"autocalibration_dialog.h"
#include"aboutdialog.h"
#include"rawdataui_dialog.h"
#include"camerasetting_dialog.h"
#include"hist_ma_dialog.h"



namespace Ui {
class MainWindow;
}



class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void keyPressEvent(QKeyEvent *e);

    virtual void resizeEvent(QResizeEvent *event) override;


    void init_thread();    //线程的初始化
    void init_connect();   //信号与槽的初始化
    void load_ini_file();   //加载配置文件

private slots:

    void Display_log_slot(QString str);  //打印日志信息到控制信息输出窗口的槽函数

    void on_openFile_action_triggered();  //打开 本地文件TOF\PEAK

    void on_play_pushButton_clicked();   //播放 3D、2D  ，打开相关的定时器

    void show_image_timer_slot();       //播放2D图像的槽函数

    void queryPixel_showToolTip_slot(int x,int y);  //鼠标停靠处显示TOF 和 peak信息

    void on_tof_peak_change_toolButton_clicked();

    void on_gain_lineEdit_returnPressed();

    void on_filter_radioButton_clicked();

    void on_statistic_action_triggered();

    void on_saveFile_action_triggered();

    void isSaveFlagSlot(bool,QString,int);       //是否保存标识；存储路径；文件类型（预留）

    void on_rotate_horizontalSlider_sliderMoved(int position);    //旋转

    void on_scale_horizontalSlider_sliderMoved(int position);     //缩放

    void on_translate_horizontalSlider_sliderMoved(int position); //平移

    void on_front_toolButton_clicked();         //正视图

    void on_side_toolButton_clicked();          //侧视图

    void on_down_toolButton_clicked();          //俯视图

    void save3DSettingFile();

    void on_peakOffset_lineEdit_returnPressed();

    void on_averageNum_lineEdit_returnPressed();

    void on_centerShowYes_radioButton_clicked();

    void on_centerShowNo_radioButton_clicked();

    void recvStaticValueSlot(float tofMin,float tofMax,float peakMin,float peakMax,float xMin,float xMax,float yMin,float yMax,float zMin,float zMax);

    void one_Second_timer_slot();



  /////////////////  /**  USB 设备读写相关** * /////////////////////
    void on_linkUSB_pushButton_clicked();

    void on_readSys_pushButton_clicked();

    void reReadSysSlot(QString str);    //读取sys的返回结果

    void on_writeSys_pushButton_clicked();

    void on_loadSetting_pushButton_clicked();

    void on_saveSetting_pushButton_clicked();

    void USB_linkInfoSlot(int );

    void on_autoCalibration_action_triggered();


    void on_about_action_triggered();

    void on_pileUp_checkBox_clicked();


    void on_gain_peak_lineEdit_returnPressed();

    void on_RawData_action_triggered();




    void on_kalman_checkBox_clicked();

    void on_kalmanPara_lineEdit_returnPressed();

    void on_cameraPara_action_triggered();

    void on_Hist_MA_action_triggered();


    void on_startLineNum_comboBox_currentTextChanged(const QString &arg1);


    //获取积分次数的槽函数
    void beginRead_inteTime();
    void returnSendUsbToRead_inteTime_slot(int,QString);



    void on_showMeanLine_comboBox_currentIndexChanged(int index);

    void on_lineThreshold_lineEdit_returnPressed();

signals:
    void change_gain_signal(float);
    void change_tof_peak_signal();
    void isFilter_signal(bool);


    /***********USB 设备读写相关************/
    void openLink_signal(int,int);
    void closeLinkSignal();
    void readSysSignal(int,bool);
    void writeSysSignal(int,QString,bool);
    void loadSettingSignal(QString,bool);   //加载配置集
    void start_read_usb_signal();

    void loadTXT_signal(QString ,bool);   //第二种加载配置文件

    /*********读取积分次数相关的****************/
    void sendUsbToRead_inteTime_signal(int, bool);

    /*******显示18行、 两行取平均、三行取平均 的信号******************/
    void sendShowMeanLine_signal(int index);


private:
    QThread *dealMsg_thread;           //数据处理线程
    DealUsb_msg *dealMsg_obj;

    QThread *savePcd_thread;           //文件保存线程
    savePCDThread* savePcd_obj;

    QThread *savePcd_thread_2;
    savePCDThread* savePcd_obj_2;


    QThread *calThread;
    calMeanStdThread *calMeanStd_obj;     //计算均值、方差线程相关

    QThread *recvUsb_thread;
    ReceUSB_Msg *recvUsbMsg_obj;          //接收USB 数据相关线程

    QString localFileDirPath;            //本地播放文件
    statisticsDialog  statisticsDia_;   //统计信息界面

    CameraSetting_Dialog cameraSetting_dia;   //相机设置窗口
    fileSave_Dialog fileSave_dia;       //文件保存界面
    QTimer show_image_timer;            //2D图像显示的定时器
    autoCalibration_Dialog autoCal_dia;   //自动校正的窗口

    int frameCount;                   //统计帧率相关
    QLabel fpsLabel;
    QTimer one_Second_timer;

    /**********USB 连接信息相关****************/
    bool isLinkSuccess;

    /*********保存rowData信息槽函数*************/
    rawDataUI_Dialog rawData_dia;

    /***********MA算法相关******************/
    Hist_MA_Dialog histMA_dia;

    /************  读取积分次数相关****************/
    quint8 inteTime[2];


    aboutDialog about_dia;
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
