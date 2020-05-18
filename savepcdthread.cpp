#include "savepcdthread.h"
#include<QDebug>
#include<QMutex>
#include<QFile>

QMutex saveMutex;
QString saveTofPeak_string;
extern QString saveFilePath;   //保存的路径  E:/..../.../的形式
extern int saveFileIndex;      //文件标号；1作为开始


extern QString rawData_savePath;    //保存的路径
extern int rawData_savepixel_num;   //开始的标号


savePCDThread::savePCDThread(QObject *parent) : QObject(parent)
{
    qDebug()<<"save pcd thread start"<<endl;
}


//保存二进制效率会高的多，但不利于查看
//void savePCDThread::savePCDSlot(pcl::PointCloud<pcl::PointXYZRGB> cloud,int formatFlag)
//{
//    //第一包数据丢掉不完整
//    if(1 == saveFileIndex)
//    {
//        saveFileIndex++;
//        return;
//    }


//    QString filePathName = saveFilePath + QString::number(saveFileIndex-1)+".pcd";
//    if(0 == formatFlag)
//    {
//        pcl::io::savePCDFileBinary(filePathName.toLatin1().toStdString(),cloud);
//    }
//    else if(1 == formatFlag)
//    {
//        pcl::io::savePCDFileASCII(filePathName.toLatin1().toStdString(),cloud);
//    }
//    saveFileIndex++;
//}



void savePCDThread::saveTXTSlot(QString msgStr)
{
    if(1 == saveFileIndex)
    {
        saveFileIndex++;
        return;
    }

    writeTXT(msgStr,saveFileIndex);

}



//按照标识写文件
// input:text:写入文本的内容
// numOfFile：第几个文件
void savePCDThread::writeTXT(QString text, int index)
{
    QString sFilePath = saveFilePath + QString::number(saveFileIndex-1)+".txt";
    saveFileIndex++;
    QFile file(sFilePath);
//    file.open(QIODevice::WriteOnly|QIODevice::Text|QIODevice::Append);
    file.open(QIODevice::WriteOnly|QIODevice::Text);

    QTextStream out(&file);
    out<<text.toLocal8Bit()<<endl;
    file.close();

    emit send_savedFileIndex_signal(index);
}



/*************************保存RAWDATA数据*****************************************/
/******************************************************************/

//!
//! \brief savePCDThread::create_rawData_Dir_slot
//! \param frame
//!  1、先在rawData_savePath(全局变量下)  检测有没有名字的 frame 的文件夹
//!  2、 如果有则跳过
//!  3、 如果没有，则生成frame文件，然后在frame文件夹下生成0 -159个文件夹
void savePCDThread::create_rawData_Dir_slot(int frame)
{
    //建立帧数的文件夹
    QString frame_filePath = rawData_savePath + QString::number(frame);
    bool buscess = isDirExist(frame_filePath);


    qDebug()<<QStringLiteral("收到创建文件夹的命令")<<frame_filePath;
}


//@brief:判断文件夹是否存在,不存在则创建
bool savePCDThread::isDirExist(QString fullPath)
{
    QDir dir(fullPath);
    if(dir.exists())
    {
//      qDebug()<<fullPath<<QStringLiteral("已经存在")<<endl;
      return true;
    }
    else
    {
       QString fullDirPath = fullPath;
       bool ok = dir.mkdir(fullDirPath);//只创建一级子目录，即必须保证上级目录存在

       if(ok)   //在目录下面连续连续创建 0 -159个文件夹
       {
//           for(int i=0; i<120; i++)
//           {
//               fullDirPath = fullPath+"/"+QString::number(i);
//               dir.mkdir(fullDirPath);
//           }

           for(int i=0; i<60;i++)
           {
               fullDirPath = fullPath+"/"+QString::number(59-i);
               dir.mkdir(fullDirPath);
               fullDirPath = fullPath+"/"+QString::number(60+i);
               dir.mkdir(fullDirPath);
           }


       }

       return ok;
    }
}


//!
//! \brief savePCDThread::start_saveRawData_slot
//!开始接收并保存rowdata数据
void savePCDThread::start_saveRawData_slot(int frame ,int row ,QStringList rowDataList)
{
    int len = rowDataList.length();
    assert(len == 160);
    QString filePath = rawData_savePath + QString::number(frame)+"/"+ QString::number(row)+"/";   //E:/DATA/0/159/
    QString sFilePath;
    for(int i=0; i<len; i++)     //160个文件夹
    {
        sFilePath = filePath + QString::number(i)+".txt";      //E:/DATA/0/159/0-159.txt

//        qDebug()<<"sFilePath = "<<sFilePath<<"  threadId= "<<QThread::currentThreadId() ;
        QString text = rowDataList[i];

        QFile file(sFilePath);
        file.open(QIODevice::WriteOnly|QIODevice::Text|QIODevice::Append);
        QTextStream out(&file);
        out<<text.toLocal8Bit();
        file.close();
    }

}






