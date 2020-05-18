#ifndef SAVEPCDTHREAD_H
#define SAVEPCDTHREAD_H
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL中支持的点类型头文件。
#include <QObject>
#include<QDir>
#include<QThread>

class savePCDThread : public QObject
{
    Q_OBJECT
public:
    explicit savePCDThread(QObject *parent = 0);

signals:

    void send_savedFileIndex_signal(int);

public slots:
//    void savePCDSlot(pcl::PointCloud<pcl::PointXYZRGB> cloud,int formatFlag); //0:二进制； 1：ASCII

    void writeTXT(QString text,int index);

    void saveTXTSlot(QString msgStr);





    /**************rowdata 保存相关******************/
    void create_rawData_Dir_slot(int frame);

    bool isDirExist(QString fullPath);  //判断文件夹是否存在,不存在则创建

    void start_saveRawData_slot(int,int,QStringList);
};

#endif // SAVEPCDTHREAD_H
