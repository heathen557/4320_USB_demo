#include "mainwindow.h"
#include "ui_mainwindow.h"

//是否显示 直方图统计功能,切换tof/peak的按钮
//#define SHOW_HISTORGRAM_BUTTON
//#define SHOW_TOF_PEAK_BUTTON

using namespace std;

//图像显示相关
extern QMutex mutex_3D;  //3D点云/2D传输的互斥锁
extern QImage tofImage;
extern QImage intensityImage;
extern pcl::PointCloud<pcl::PointXYZRGB> pointCloudRgb;
extern bool isShowPointCloud;  //是否有点云数据 ，有的话显示否则不显示

//鼠标点击显示时相关
extern QMutex mouseShowMutex;
extern float mouseShowTOF[256][64];
extern float mouseShowPEAK[256][64];
extern float mouseShowDepth[256][64];

/*保存用到的标识*/
extern bool isSaveFlag;        //是否进行存储
extern QString saveFilePath;   //保存的路径  E:/..../.../的形式
extern int saveFileIndex;      //文件标号；1作为开始

/*****是否进行数据接收的标识*****/
extern bool isRecvFlag;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //起始行
    QStringList  lineNumList;
    for(int i=0;i<64;i+=2)
    {
        lineNumList.append(QString::number(i));
    }
    ui->startLineNum_comboBox->addItems(lineNumList);


    ui->Hist_MA_action->setVisible(false);
    ui->RawData_action->setVisible(false);

    frameCount  = 0 ;     //帧率
    isLinkSuccess = false;     //USB 连接是否成功的标识
    ui->groupBox_5->setVisible(false);

    ui->statusBar->addWidget(&fpsLabel);
//    ui->statusBar->setStyleSheet(QString("QStatusBar::item{border:0px}"));

    qRegisterMetaType<QVector<double>>("vector<double>");   //注册函数


#ifndef SHOW_HISTORGRAM_BUTTON
    ui->peak_Histogram_pushButton->setVisible(false);
#endif
#ifndef SHOW_TOF_PEAK_BUTTON
    ui->tof_peak_change_toolButton->setVisible(false);
#endif

    ui->config_toolBox->setCurrentIndex(0);

    init_thread();
    init_connect();
    load_ini_file();
    dealMsg_obj->loadLocalArray();   //接收数据线程中 加载角度矩阵的矫正文件

    one_Second_timer.start(1000);
}

MainWindow::~MainWindow()
{
    delete ui;
}

//!
//! \brief init_thread
//! //线程的初始化
void MainWindow::init_thread()
{
    //数据接收线程
    dealMsg_thread = new QThread();
    dealMsg_obj = new DealUsb_msg();
    dealMsg_obj->moveToThread(dealMsg_thread);
    dealMsg_thread->start();

    //文件保存线程
    savePcd_thread = new QThread();      //文件保存线程
    savePcd_obj = new savePCDThread;
    savePcd_obj->moveToThread(savePcd_thread);
    savePcd_thread->start();

    //rowdata下半部分 保存的数据线程
    savePcd_thread_2 = new QThread();
    savePcd_obj_2 = new savePCDThread();
    savePcd_obj_2->moveToThread(savePcd_thread_2);
    savePcd_thread_2->start();


    //统计信息相关
    calThread = new QThread();
    calMeanStd_obj = new calMeanStdThread(); //
    calMeanStd_obj->moveToThread(calThread);
    calThread->start();

    //USB数据接收相关
    recvUsb_thread = new QThread();
    recvUsbMsg_obj = new ReceUSB_Msg();
    recvUsbMsg_obj->moveToThread(recvUsb_thread);
    recvUsb_thread->start();


}


//!
//! \brief init_connect
//! //信号与槽的初始化
void MainWindow::init_connect()
{
    //与数据处理线程的 信号与槽的连接
    connect(dealMsg_obj,&DealUsb_msg::Display_log_signal,this,&MainWindow::Display_log_slot);
    connect(this,&MainWindow::change_gain_signal,dealMsg_obj,&DealUsb_msg::change_gain_slot);
    connect(this,&MainWindow::change_tof_peak_signal,dealMsg_obj,&DealUsb_msg::change_tof_peak_slot);
    connect(this,&MainWindow::isFilter_signal,dealMsg_obj,&DealUsb_msg::isFilter_slot);
    connect(dealMsg_obj,&DealUsb_msg::staticValueSignal,this,&MainWindow::recvStaticValueSlot);


    //文件保存相关的信号与槽的连接
    connect(&fileSave_dia,&fileSave_Dialog::isSaveFlagSignal,this,&MainWindow::isSaveFlagSlot);
    connect(&fileSave_dia,&fileSave_Dialog::alter_fileSave_signal,dealMsg_obj,&DealUsb_msg::alter_fileSave_slot);
    connect(dealMsg_obj,&DealUsb_msg::saveTXTSignal,savePcd_obj,&savePCDThread::saveTXTSlot);


    //统计信息相关的槽函数
    connect(calMeanStd_obj,SIGNAL(statistic_MeanStdSignal(QStringList,QStringList,QStringList,QStringList)),&statisticsDia_,SLOT(statistic_MeanStdSlot(QStringList,QStringList,QStringList,QStringList)));
    connect(&statisticsDia_,SIGNAL(startStop_signal(int)),calMeanStd_obj,SLOT(startStop_slot(int)));
    connect(&statisticsDia_,SIGNAL(alterStatisticFrameNum_signal(int)),dealMsg_obj,SLOT(alterStatisticFrameNum_slot(int)));

    //2D图像的显示相关
    connect(&show_image_timer,SIGNAL(timeout()),this,SLOT(show_image_timer_slot()));

    //鼠标停靠处显示
    connect(ui->tof_label,SIGNAL(queryPixSignal(int,int)),this,SLOT(queryPixel_showToolTip_slot(int,int)));
    connect(ui->peak_label,SIGNAL(queryPixSignal(int,int)),this,SLOT(queryPixel_showToolTip_slot(int,int)));

    //USB设备连接相关
    connect(this,SIGNAL(openLink_signal(int,int)),recvUsbMsg_obj,SLOT(openLinkDevSlot(int,int)));
    connect(recvUsbMsg_obj,SIGNAL(Display_log_signal(QString)),this,SLOT(Display_log_slot(QString)));
    connect(recvUsbMsg_obj,SIGNAL(linkInfoSignal(int)),this,SLOT(USB_linkInfoSlot(int)));
    connect(this,SIGNAL(closeLinkSignal()),recvUsbMsg_obj,SLOT(closeUSB()));
    connect(this,SIGNAL(readSysSignal(int,bool)),recvUsbMsg_obj,SLOT(readSysSlot(int,bool)));
    connect(recvUsbMsg_obj,&ReceUSB_Msg::reReadSysSignal,this,&MainWindow::reReadSysSlot);
    connect(this,&MainWindow::writeSysSignal,recvUsbMsg_obj,&ReceUSB_Msg::writeSysSlot);
    connect(this,&MainWindow::loadSettingSignal,recvUsbMsg_obj,&ReceUSB_Msg::loadSettingSlot);
    connect(this,&MainWindow::start_read_usb_signal,recvUsbMsg_obj,&ReceUSB_Msg::start_read_usbImage_slot);
    connect(this,&MainWindow::loadTXT_signal,recvUsbMsg_obj,&ReceUSB_Msg::loadTXT_slot);

    //数据接收 与数据处理线程
    connect(recvUsbMsg_obj,SIGNAL(recvMsgSignal(QByteArray)),dealMsg_obj,SLOT(recvMsgSlot(QByteArray)));

    //自动校正相关
    connect(&autoCal_dia,SIGNAL(start_autoCalibration_signal(float)),dealMsg_obj,SLOT(start_autoCalibration_slot(float)));
    connect(dealMsg_obj,SIGNAL(send_cali_success_signal(QString)),&autoCal_dia,SLOT(send_cali_success_slot(QString)));

    //显示帧率相关的槽函数
    connect(&one_Second_timer,SIGNAL(timeout()),this,SLOT(one_Second_timer_slot()));


    //rawData数据接收相关的槽函数
    connect(&rawData_dia,SIGNAL(on_start_rawDataSave_signal()),recvUsbMsg_obj,SLOT(on_start_rawDataSave_slot()));
    connect(recvUsbMsg_obj,SIGNAL(receRawDataSave_signal(QByteArray)),dealMsg_obj,SLOT(receRawDataSave_slot(QByteArray)));
    connect(dealMsg_obj,SIGNAL(create_rawData_Dir_signal(int)),savePcd_obj,SLOT(create_rawData_Dir_slot(int)));
    connect(dealMsg_obj,SIGNAL(create_rawData_Dir_signal(int)),savePcd_obj_2,SLOT(create_rawData_Dir_slot(int)));
    connect(dealMsg_obj,SIGNAL(start_saveRawDataUp_signal(int,int,QStringList)),savePcd_obj,SLOT(start_saveRawData_slot(int,int,QStringList)));
    connect(dealMsg_obj,SIGNAL(start_saveRawDataDown_signal(int,int,QStringList)),savePcd_obj_2,SLOT(start_saveRawData_slot(int,int,QStringList)));


    //相机配置  修改相机焦距、积分次数、相机镜头间距
    connect(&cameraSetting_dia,SIGNAL(alter_focal_integrate_signal(float,float,int)),dealMsg_obj,SLOT(alter_focal_integrate_slot(float,float,int)));


//    //rawData MA算法相关
//    connect(&histMA_dia,&Hist_MA_Dialog::start_receRowDataMA_signal,recvUsbMsg_obj,&ReceUSB_Msg::start_receRowDataMA_slot);
//    connect(&histMA_dia,&Hist_MA_Dialog::start_RowDatahistogram_signal,dealMsg_obj,&DealUsb_msg::start_RowDatahistogram_slot);
//    connect(recvUsbMsg_obj,SIGNAL(receRawData_MA_signal(QByteArray)),dealMsg_obj,SLOT(receRawData_MA_slot(QByteArray)));
//    connect(dealMsg_obj,SIGNAL(toShowHistogram_signal(QVector<double>,int)),&histMA_dia,SLOT(toShowHistogram_slot(QVector<double>,int)));
//    connect(dealMsg_obj,SIGNAL(currentFrame_signal(int)),&histMA_dia,SLOT(currentFrame_slot(int)));


}

//!
//! \brief MainWindow::load_ini_file
//!加载配置集文件
void MainWindow::load_ini_file()
{
    QSettings configSetting("setting.ini", QSettings::IniFormat);

    int Rotate_rate = configSetting.value("operation/Rotate_rate").toInt();
    int Scale_rate = configSetting.value("operation/Scale_rate").toInt();
    int transtate_rate = configSetting.value("operation/transtate_rate").toInt();

    if(Rotate_rate==0 || Scale_rate==0 || transtate_rate==0)    //防止第一次加载时，没有配置文件，这时候初始化为初始值
    {
        Rotate_rate = 8;
        Scale_rate = 10;
        transtate_rate = 30;
        QString log_str = "[load conf file error]:setting.ini";
        Display_log_slot(log_str);
    }

    ui->widget->rotateRate = Rotate_rate;
    ui->widget->scaleRate = Scale_rate;
    ui->widget->translateRate = 110-transtate_rate;

    ui->rotate_horizontalSlider->setValue(Rotate_rate);
    ui->scale_horizontalSlider->setValue(Scale_rate);
    ui->translate_horizontalSlider->setValue(transtate_rate);
    QString log_str = "[load conf file success]:setting.ini";
    Display_log_slot(log_str);

}


//!
//! \brief MainWindow::on_openFile_action_triggered
//!打开本地文件的槽函数  弹出窗口
void MainWindow::on_openFile_action_triggered()
{
    QString filePath=QFileDialog::getExistingDirectory();
    filePath.append("/");

    if(filePath.length()<2)
        return;

    localFileDirPath = filePath;
    QString log_str_ = "[Local Dir]:" + filePath;
    Display_log_slot(log_str_);

    dealMsg_obj->playLocalFile_slot(filePath);

}

//!
//! \brief MainWindow::on_play_pushButton_clicked
//!开启播放图像的槽函数
void MainWindow::on_play_pushButton_clicked()
{
    if(ui->play_pushButton->text() == "play")
    {
        ui->widget->show3D_timer.start(90);   //3D点云的刷新频率
        show_image_timer.start(90);
        ui->play_pushButton->setText("pause");

        if(isLinkSuccess)
        {
           isRecvFlag = true;
           emit start_read_usb_signal();
           QString log_str = "[video start]";
           Display_log_slot(log_str);
        }

    }else
    {
        ui->widget->show3D_timer.stop();
        show_image_timer.stop();
        ui->play_pushButton->setText("play");
        QString log_str = "[video stop]";
        Display_log_slot(log_str);
    }
}


//!
//! \brief MainWindow::show_image_timer_slot
//! 播放2D图像的槽函数
void MainWindow::show_image_timer_slot()
{
    if(!isShowPointCloud)
        return;

    if(!tofImage.isNull() && !intensityImage.isNull())
    {
        mutex_3D.lock();
        QPixmap pixmap_tof(QPixmap::fromImage (tofImage));
        ui->tof_label->setPixmap(pixmap_tof);

        QPixmap pixmap_peak(QPixmap::fromImage (intensityImage));
        ui->peak_label->setPixmap(pixmap_peak);
        mutex_3D.unlock();
    }
}


//! \brief MainWindow::queryPixel_showToolTip_slot
//! \param x   实际的x坐标值  缩放以后的
//! \param y   实际的y坐标值  缩放以后的
//!  鼠标停靠处显示tof和peak
void MainWindow::queryPixel_showToolTip_slot(int x,int y)
{

    float width_scale = ui->tof_label->width()/256.0;
    float height_scale = ui->tof_label->height()/64.0;

    int y_index = y/height_scale;
    int x_index = x/width_scale;
//    qDebug()<<"y_index="<<y_index<<"  x_index ="<<x_index;
    int index = 256*y_index + x_index;
    mouseShowMutex.lock();
    QString str= "x="+QString::number(x_index)+",y="+QString::number(y_index)+",tof="+QString::number(mouseShowTOF[x_index][y_index])+",peak="+QString::number(mouseShowPEAK[x_index][y_index])+",depth="+QString::number(mouseShowDepth[x_index][y_index])+"m";
    mouseShowMutex.unlock();
    QToolTip::showText(QCursor::pos(),str);
}


//!
//! \brief MainWindow::Display_log_slot
//!打印控制信息的槽函数
void MainWindow::Display_log_slot(QString str)
{
    ui->control_log_textEdit->append(str);


    //读取积分次数 开始行 以及 结束行


}



//!
//! \brief MainWindow::on_tof_peak_change_toolButton_clicked
//!切换tof peak的槽函数
void MainWindow::on_tof_peak_change_toolButton_clicked()
{
    emit change_tof_peak_signal();
}


//!
//! \brief MainWindow::on_gain_lineEdit_returnPressed
//!改变tof增益的槽函数
void MainWindow::on_gain_lineEdit_returnPressed()
{
    float gain = ui->gain_lineEdit->text().toFloat();
    dealMsg_obj->gainImage_tof = gain;
//    emit change_gain_signal(gain);
}

//!
//! \brief MainWindow::on_gain_lineEdit_returnPressed
//!改变peak增益的槽函数
void MainWindow::on_gain_peak_lineEdit_returnPressed()
{
    float gain = ui->gain_peak_lineEdit->text().toFloat();
    dealMsg_obj->gainImage_peak = gain;
//    emit change_gain_signal(gain);
}

//!
//! \brief MainWindow::on_filter_radioButton_clicked
//!是否进行滤波的槽函数
void MainWindow::on_filter_radioButton_clicked()
{
    bool isChecked = ui->filter_radioButton->isChecked();
    emit isFilter_signal(isChecked);
}

//!
//! \brief MainWindow::on_statistic_action_triggered
//!统计信息 弹出界面函数
void MainWindow::on_statistic_action_triggered()
{
    QMessageBox::warning(NULL,QStringLiteral("提示"),QStringLiteral("该功能需要管理员权限，否则可能会异常退出"));
    statisticsDia_.setModal(true);
    statisticsDia_.show();
}

//!
//! \brief MainWindow::on_saveFile_action_triggered
//!文件保存界面 弹出
void MainWindow::on_saveFile_action_triggered()
{
    fileSave_dia.setModal(true);
    fileSave_dia.show();
}

//!
//! \brief isSaveFlagSlot
//! 文件保存 接收的槽函数
void MainWindow::isSaveFlagSlot(bool flag,QString filePath,int fileFormat)
{


   isSaveFlag = flag;        //是否进行存储
   saveFilePath = filePath;   //保存的路径  E:/..../.../的形式
   saveFileIndex = 1;      //文件标号；1作为开始
}


//旋转角度设置的槽函数
void MainWindow::on_rotate_horizontalSlider_sliderMoved(int position)
{
    ui->widget->rotateRate = position;
    save3DSettingFile();

}

//缩放比例设置的槽函数
void MainWindow::on_scale_horizontalSlider_sliderMoved(int position)
{
    ui->widget->scaleRate = position;
    save3DSettingFile();

}

//拖放比例的槽函数
void MainWindow::on_translate_horizontalSlider_sliderMoved(int position)
{
    ui->widget->translateRate = 110 - position;
    save3DSettingFile();

}

//鼠标控制相关的配置 会保存到配置文件当中
void MainWindow::save3DSettingFile()
{
    QSettings configSetting("setting.ini", QSettings::IniFormat);

    int rotateRate = ui->widget->rotateRate;
    int scaleRate = ui->widget->scaleRate;
    int translateRate = 110-ui->widget->translateRate;

    configSetting.setValue("operation/Rotate_rate",rotateRate);
    configSetting.setValue("operation/Scale_rate",scaleRate);
    configSetting.setValue("operation/transtate_rate",translateRate);

}

//!
//! \brief MainWindow::on_front_toolButton_clicked
//!设置正视图的视角
void MainWindow::on_front_toolButton_clicked()
{
    ui->widget->frontView_slot();
}

//!
//! \brief MainWindow::on_side_toolButton_clicked
//!侧视图视角
void MainWindow::on_side_toolButton_clicked()
{
    ui->widget->endView_slot();
}

//!
//! \brief MainWindow::on_down_toolButton_clicked
//!俯视图视角
void MainWindow::on_down_toolButton_clicked()
{
    ui->widget->verticalView_slot();
}




//显示  peak的阈值
void MainWindow::on_peakOffset_lineEdit_returnPressed()
{
    int peakOffset = ui->peakOffset_lineEdit->text().toInt();
    dealMsg_obj->peakOffset = peakOffset;

    qDebug()<<"peakOffset = "<<peakOffset;
}
//显示  平均的帧数
void MainWindow::on_averageNum_lineEdit_returnPressed()
{
    int average_frameNum = ui->averageNum_lineEdit->text().toInt();
    dealMsg_obj->averageNum = average_frameNum;
    qDebug()<<"averageNum = "<<average_frameNum;
}
//显示 只显示中心区域
void MainWindow::on_centerShowYes_radioButton_clicked()
{
    dealMsg_obj->isOnlyCenterShow_flag = true;
}
//显示 不显示中心区域
void MainWindow::on_centerShowNo_radioButton_clicked()
{
    dealMsg_obj->isOnlyCenterShow_flag = false;
}

//!
//! \brief MainWindow::recvStaticValueSlot
//! \param tofMin
//! \param tofMax
//! \param peakMin
//! \param peakMax
//! \param xMin
//! \param xMax
//! \param yMin
//! \param yMax
//! \param zMin
//! \param zMax
//!
void MainWindow::recvStaticValueSlot(float tofMin,float tofMax,float peakMin,float peakMax,float xMin,float xMax,float yMin,float yMax,float zMin,float zMax)
{
   frameCount++;
}

//!
//! \brief one_Second_timer_slot
//! 一秒钟显示一次帧率
void MainWindow::one_Second_timer_slot()
{
    QString fpsStr = "fps:" + QString::number(frameCount);
    fpsLabel.setText(fpsStr);
    frameCount = 0;
}


/********************************************  USB 配置相关的槽函数   ********************************************************************/
//!
//! \brief MainWindow::on_linkUSB_pushButton_clicked
//!连接USB 设备的槽函数
void MainWindow::on_linkUSB_pushButton_clicked()
{

    if(ui->linkUSB_pushButton->text() == QStringLiteral("连接设备"))
    {
        int vid = ui->VID_lineEdit->text().toInt(NULL,16);
        int pid = ui->PID_lineEdit->text().toInt(NULL,16);
        emit openLink_signal(vid,pid);

    }else
    {
        isRecvFlag = false;
        isLinkSuccess = false;
        emit closeLinkSignal();
        ui->linkUSB_pushButton->setText(QStringLiteral("连接设备"));
    }
}
//!
//! \brief MainWindow::on_readSys_pushButton_clicked
//! 读取系统寄存器的额槽函数
void MainWindow::on_readSys_pushButton_clicked()
{
    if(!isLinkSuccess)
    {
        QMessageBox::information(NULL,QStringLiteral("告警"),QStringLiteral("设备未连接"));
        return;
    }
    int address = ui->sysAddress_lineEdit->text().toInt(NULL,16);

    if(isRecvFlag)
    {
        isRecvFlag = false;
        emit readSysSignal(address,true);
    }else
    {
        emit readSysSignal(address,false);
    }
}

//!
//! \brief MainWindow::reReadSysSlot   //读取sys的返回结果
//! \param str
//!
void MainWindow::reReadSysSlot(QString str)
{
    int m = str.toInt();
    qDebug()<<" the data =  "<<m<<endl;
    ui->sysData_lineEdit->setText(QString::number(m,16).toUpper());
}



//!
//! \brief MainWindow::on_writeSys_pushButton_clicked
//!写入系统寄存器的槽函数
void MainWindow::on_writeSys_pushButton_clicked()
{
    if(!isLinkSuccess)
    {
        QMessageBox::information(NULL,QStringLiteral("告警"),QStringLiteral("设备未连接"));
        return;
    }
    int address = ui->sysAddress_lineEdit->text().toInt(NULL,16);
    QString data = ui->sysData_lineEdit->text();

    if(isRecvFlag)
    {
        isRecvFlag = false;
        emit writeSysSignal(address,data,true);
    }else
    {
        emit writeSysSignal(address,data,false);
    }
    ui->sysData_lineEdit->clear();

}
//!
//! \brief MainWindow::on_loadSetting_pushButton_clicked
//!加载配置集的槽函数
void MainWindow::on_loadSetting_pushButton_clicked()
{
    if(!isLinkSuccess)
    {
        QMessageBox::information(NULL,QStringLiteral("告警"),QStringLiteral("设备未连接"));
        return;
    }


    QString file_path;
    //定义文件对话框类
    QFileDialog *fileDialog = new QFileDialog(this);
    //定义文件对话框标题
    fileDialog->setWindowTitle(QStringLiteral("请选择配置文件"));
    //设置默认文件路径
    fileDialog->setDirectory(".");
    //设置视图模式
    fileDialog->setViewMode(QFileDialog::Detail);
    //打印所有选择的文件的路径

    QStringList mimeTypeFilters;
    mimeTypeFilters <<QStringLiteral("芯片配置文件(*.txt)|*.txt") ;
    fileDialog->setNameFilters(mimeTypeFilters);


    QStringList fileNames;
    if(fileDialog->exec())
    {
        fileNames = fileDialog->selectedFiles();
    }else
    {
        return;
    }
    ////////////////////////////////////////////////////////////////////////////////////////

    file_path = fileNames[0];
    qDebug()<<" file_path = "<<fileNames[0]<<endl;

    QString checkStr = file_path.right(3);
    if("txt" != checkStr)
    {
        QMessageBox::information(NULL,QStringLiteral("告警"),QStringLiteral("请选择正确的配置文件！"));
        return ;
    }

    if(isRecvFlag)
    {
        isRecvFlag = false;
        emit loadTXT_signal(file_path,  true);
    }else
    {
        emit loadTXT_signal(file_path,  false);
    }

}
//!
//! \brief MainWindow::on_saveSetting_pushButton_clicked
//!保存配置集的槽函数
void MainWindow::on_saveSetting_pushButton_clicked()
{
    if(!isLinkSuccess)
    {
        QMessageBox::information(NULL,QStringLiteral("告警"),QStringLiteral("设备未连接"));
        return;
    }
}

//!
//! \brief MainWindow::USB_linkInfoSlot
//! \param flag
//!USB 连接的返回信息
//! //向主线程发送链接信息（错误警告）
// 0：连接正常 1没找到设备
// 2:没有接收到数据  3打开设备失败
// 4：读取系统成功；5：读取系统失败；
// 6：读取设备成功；7：读取设备失败
// 8：加载配置信息成功；9：加载配置信息失败
// 10：保存配置信息成功； 11：保存配置信息失败
// 12：写入系统成功      13：写入系统失败
// 14：写入设备成功      15：写入设备失败
void MainWindow::USB_linkInfoSlot(int flag )
{
    if(0 == flag)     //连接正常
    {
        ui->linkUSB_pushButton->setText(QStringLiteral("关闭设备"));
        isLinkSuccess = true;
    }else if(3==flag)   //连接设备失败
    {
        isLinkSuccess = false;
        ui->linkUSB_pushButton->setText(QStringLiteral("连接设备"));
    }
}



//自动校准的窗口
void MainWindow::on_autoCalibration_action_triggered()
{
    autoCal_dia.setModal(true);
    autoCal_dia.show();
}

//!
//! \brief MainWindow::on_pileUp_checkBox_clicked
//! pileUp 标识
void MainWindow::on_pileUp_checkBox_clicked()
{
    bool  flag = ui->pileUp_checkBox->isChecked();
    dealMsg_obj->is_pileUp_flag = flag;


    qDebug()<<"flag = "<<flag;
}


//!
//! \brief MainWindow::on_about_action_triggered
//!显示版本信息
void MainWindow::on_about_action_triggered()
{
    about_dia.show();
}


//!
//! \brief MainWindow::keyPressEvent
//! \param e
//! 按键
void MainWindow::keyPressEvent(QKeyEvent *e)
{
    qDebug()<<"key num = "<<e->key();
    if(90 == e->key())
    {
        ui->groupBox_5->setVisible(true);
    }else if(88 == e->key())
    {
        ui->groupBox_5->setVisible(false);
    }
}



//!
//! \brief MainWindow::on_RawData_action_triggered
//!raw data弹出界面
void MainWindow::on_RawData_action_triggered()
{
    rawData_dia.show();
}


void MainWindow::resizeEvent(QResizeEvent *event)
{
//    qDebug()<<"fun has commit";
//    QSize size=  ui->tof_label->size();
//    int width = size.width();
//    int height = size.height();
//    qDebug()<<"width ="<<width<<"   height="<<height;

////    float height = size.width()*4/3;


//    ui->tof_label->set
//    ui->left_dockWidget->set

//    ui->tof_label->setBaseSize(500,300);
//    ui->tof_label->setFixedWidth(width*10/3);
//    ui->peak_label->setFixedWidth(width*10/3);

}



//!
//! \brief MainWindow::on_kalman_checkBox_clicked
//!是否开启kalman filters
void MainWindow::on_kalman_checkBox_clicked()
{
    if(ui->kalman_checkBox->isChecked())
    {
        dealMsg_obj->isKalman = true;
    }else
    {
        dealMsg_obj->isKalman = false;
    }
}

//!
//! \brief MainWindow::on_kalmanPara_lineEdit_returnPressed
//!卡尔曼滤波的系数
void MainWindow::on_kalmanPara_lineEdit_returnPressed()
{
    dealMsg_obj->kalmanOffset_para = ui->kalmanPara_lineEdit->text().toFloat();
    qDebug()<<"kalmanPara = "<<ui->kalmanPara_lineEdit->text().toFloat();
}


//相机参数设置界面
void MainWindow::on_cameraPara_action_triggered()
{
    cameraSetting_dia.show();
}

//MA窗口的槽函数
void MainWindow::on_Hist_MA_action_triggered()
{
    histMA_dia.setModal(true);
    histMA_dia.show();
}


//线阵模式改变起始行
void MainWindow::on_startLineNum_comboBox_currentTextChanged(const QString &arg1)
{

}
