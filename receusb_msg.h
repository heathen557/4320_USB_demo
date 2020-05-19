#ifndef RECEUSB_MSG_H
#define RECEUSB_MSG_H
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL中支持的点类型头文件。
#include <QObject>
#include<QDebug>
#include<QTimer>
#include<QImage>
#include<Windows.h>
#include"lusb0_usb.h"
#include<QFile>
#include<QThread>


class ReceUSB_Msg : public QObject
{
    Q_OBJECT
public:
    explicit ReceUSB_Msg(QObject *parent = 0);

    struct usb_device *findUSBDev(const unsigned short idVendor,
                                  const unsigned short idProduct);

    bool openUSB(struct usb_device *dev);


    bool System_Register_Read(int Address, QString &Data);

    bool System_Register_Write(int Address, QString &Data);

    bool Device_Register_Read(int slavedId,int Address,QString &Data);

    bool Device_Register_Write(int slavedId,int Address,QString &Data);


    bool devOpenFlg;

    usb_dev_handle *devHandle;

    struct usb_device * dev;

    QTimer *readTimer;               //此处采用轮询的方式读取USB串口数据，1ms读取一次
    QTimer showTimer;
    QImage microQimage;
    QImage macroQimage;
    QImage saveTofIntenImage;
    int lastSpadNum;

    pcl::PointCloud<pcl::PointXYZRGB> tempRgbCloud;
//    pcl::PointCloud<pcl::PointXYZI> tempcloud_XYZI;
//    pcl::PointCloud<pcl::PointXYZ>  tempcloud_RGB;
    int cloudIndex;

    float  LSB ; //时钟频率
    bool isFirstLink;   //因为USB驱动存在问题，故设次函数

    float tofMin,tofMax,peakMin,peakMax,xMin,xMax,yMin,yMax,zMin,zMax;
    float temp_x,temp_y,temp_z;

    int r,g,b,rgb;

    int tmp_tofInfo[16384];
    int tmp_peakInfo[16384];

    int idVendor_,idProduct_;

    QByteArray tmpArray;    //用于协议2的缓存 缓存到260个字节时发送给数据处理线程



    int isRawDataSave_flag;        //是否属于rawData模式接收数据    1 :USB RawData数据保存相关   2：usb RawData MA算法相关   3：USB普通数据


signals:
    void linkInfoSignal(int );      //向主线程发送链接信息（错误警告）
                                    // 0：连接正常 1没找到设备
                                    // 2:没有接收到数据  3打开设备失败
                                    // 4：读取系统成功；5：读取系统失败；
                                    // 6：读取设备成功；7：读取设备失败
                                    // 8：加载配置信息成功；9：加载配置信息失败
                                    // 10：保存配置信息成功； 11：保存配置信息失败
                                    // 12：写入系统成功      13：写入系统失败
                                    // 14：写入设备成功      15：写入设备失败
    void staticValueSignal(float,float,float,float,float,float,float,float,float,float);

    void recvMsgSignal(QByteArray); //发送给处理线程的信号

//    void recvMsgSignal_2(QByteArray); //发送给处理线程的信号2  协议更改版本 （4个字节 + 256个字节）

    void reReadSysSignal(QString);  //读取系统指令 返回信号

    void reReadDevSignal(int,QString); //读取设备指令 返回信号

    void showRunInfoSignal(QString msgStr);     //显示运行日志的槽函数

    void showWarnInfoSignal(QString warnStr);  //显示告警信息日志的槽函数

    void Display_log_signal(QString str);       //主界面显示运行日志的信号

    void receRawDataSave_signal(QByteArray);    //发送给数据处理线程供 rawData数据分析

    void receRawData_MA_signal(QByteArray);     //MA 算法相关


    /************************/
    void returnSendUsbToRead_inteTime_signal(int,QString);




public slots:
    void openLinkDevSlot(int vId, int pId);          //打开设备连接的槽函数
    void closeUSB();
    void read_usb();                                //读取USB内容的槽函数
    void start_read_usbImage_slot();                //开始接收点云数据的槽函数

    void readSysSlot(int address,bool recvFlag);
    void writeSysSlot(int addr,QString data,bool recvFlag);
    void readDevSlot(int id,int address,bool recvFlag);
    void writeDevSlot(int slavId,int addr,QString data,bool recvFlag);
    void loadSettingSlot(QString filePath,bool recvFlag);
    void saveSettingSlot(QString filePath,int deviceId,bool  recvFlag);

    void loadTXT_slot(QString filePath,bool recvFlag);



    void on_start_rawDataSave_slot(QString);   //开启rowData数据接收

    void start_receRowDataMA_slot();   //开启MA 接收


    /*********读取积分次数相关****************/
    void sendUsbToRead_inteTime_slot(int address,bool recvFlag);

};

#endif // RECEUSB_MSG_H
