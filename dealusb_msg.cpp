﻿#include "dealusb_msg.h"
#define miu_meter 1e-3

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



/*******统计信息相关的变量***********/
extern QMutex statisticMutex;
extern vector<vector<int>> allStatisticTofPoints;   //用于统计 均值和方差的 容器  TOF
extern vector<vector<int>> allStatisticPeakPoints;   //用于统计 均值和方差的 容器  TOF

//rowData相关的全局变量
extern int exposure_num;            //曝光次数
extern int rawData_saveFrameNums;   //保存多少帧RawData数据


DealUsb_msg::DealUsb_msg(QObject *parent) : QObject(parent),
    microQimage(256,64, QImage::Format_RGB32),macroQimage(256,64, QImage::Format_RGB32)
{

    //    pointCloudRgb.width = 16384;
    //    pointCloudRgb.height = 1;
    //    pointCloudRgb.resize(pointCloudRgb.width);
    tmpIndex = 0;         //线阵第一次数据会有问题
    showLineMeanInt = 0;  //默认显示全部的18行
    mean_offset = 100;

    isKalman =true;
    exposure_num = 20;   //初始化20次曝光
    rawData_saveFrameNums = 200;

    tempRgbCloud.width = 16384;
    tempRgbCloud.height = 1 ;
    tempRgbCloud.points.resize(tempRgbCloud.width);

//    LSB = 0.031;
//    LSB = 0.0075;     //alter at 05-20
    LSB = 15.55;

    isFilterFlag = false ;    //初始化时不进行滤波
    lineSelect = false;       //初始化 不切换两行像素
    isTOF = true;
    localFile_timer = NULL;
    gainImage_tof = 1;
    gainImage_peak = 1;

    haveIndex = 0;
    peakOffset = 0;   //设置为阈值，小于这个值的认为是无效数据，将接收到的tof值设置为0  ::此功能预留，面阵_1028效果较好，但是对其他数据会滤掉大部分有效数据
    isOnlyCenterShow_flag = false;   //是否只显示中心区域的标识，设置为true则只显示中心光较强的区域（超过范围的点xyz坐标全部设置为0），设置为false则显示全部点云数据；默认false;
    averageNum = 1;            //滑动平均的帧数 , 默认为1

    /********pileUp 以及自动校准相关的参数***************/
    is_pileUp_flag = true;
    isAutoCalibration_flag = false;  //默认不适用
    ishave_Four = 0 ;               // 初始设置为0，逐渐+1 ==4 时进行判断
    calibration_mean_num = 200;    //默认100帧取平均

    //3D坐标转换
    camera_dis = 23;             //单位mm
    f = 6 ;                        //单位mm
    //初始化相机参数
    QSettings configSetting("setting.ini", QSettings::IniFormat);
    f = configSetting.value("camera/focal_length").toFloat();
    camera_dis = configSetting.value("camera/camera_diff").toFloat();
    integrateNum_ = configSetting.value("camera/integrate_num").toInt();

    qDebug()<<"dealUsbMSG ,     f="<<f<<"   camera_dis="<<camera_dis<<"  integrateNum_="<<integrateNum_;


    //总共有16384个点，针对每一个点开启一个独立的容器进行存储相关内容    统计相关
    statisticStartFlag = true;
    statisticFrameNumber = 10;
    vector<int> singlePoint;
    for(int i=0; i<16384; i++)
    {
        tempStatisticTofPoints.push_back(singlePoint);
        tempStatisticPeakPoints.push_back(singlePoint);
        allStatisticTofPoints.push_back(singlePoint);
        allStatisticPeakPoints.push_back(singlePoint);

        //kalman filters  卡尔曼滤波算法
        x_k[i] = 0;
        p_k[i] = 0;
        K[i] = 0;

        peak_x_k[i] = 0;
        peak_p_k[i] = 0;
        peak_K[i] = 0;


         tempRgbCloud.points[i].x = 0;
         tempRgbCloud.points[i].y = 0;
         tempRgbCloud.points[i].z = 0;

    }
    kalman_F = 1;
    kalman_H = 1;
    kalman_Q = 1;
    kalman_R = 5;

    peak_kalman_F =1;
    peak_kalman_H =1;
    peak_kalman_Q =1;
    peak_kalman_R = 5;

    kalmanOffset_para = 1;   //两帧之间offset的系数


    //RawData 数据相关
    current_id = 0;
    current_package_num = 0;
    last_id = 0;    //上一帧曝光时间
    last_package_num = 0;     //上一包的标号
    TDC_first_index = 0;      // 0 1 2 3
    hongPixel_line = 0;       //第几行的宏像素
    pixel_index = 0;          //0 1 2 3 ; 接收的第几个pixel的数据
    frame_index = 0;          //第几帧数据 0 1 2 3 4 。。。。
    isFirst_frame = true;

    //初始化文件保存槽函数
    init_File_save();

    //RawData MA 相关
    historgramVec_MA.resize(4096);
    histogram_maxValue = 0;



    //图片的初始化
    for(int i=0;i<256; i++)
    {
        for(int j=0; j<64; j++)
        {
            macroQimage.setPixel(i,j,0);
            microQimage.setPixel(i,j,0);
        }
    }


}


//初始化文件保存的槽函数
void DealUsb_msg::init_File_save()
{
    isSaveRawTof = true;
    isSavePileUpTof = false;
    isSaveFilterTof = false;
    isSaveRawPeak = true;
    isSaveFilterPeak = false;
    isSaveX = false;
    isSaveY = false;
    isSaveZ = false;

    tofPeakNum[0] = QStringLiteral("Rawtof,RawPeak\n");
}


//!\
//! 修改要保存的文件信息的槽函数
void DealUsb_msg::alter_fileSave_slot(bool saveRawTof,bool savePileUpTof,bool saveFilterTof,bool saveRawPeak,bool saveFilterPeak,bool saveX,bool saveY,bool saveZ)
{
    isSaveRawTof = saveRawTof;
    isSavePileUpTof = savePileUpTof;
    isSaveFilterTof = saveFilterTof;
    isSaveRawPeak = saveRawPeak;
    isSaveFilterPeak = saveFilterPeak;
    isSaveX = saveX;
    isSaveY = saveY;
    isSaveZ = saveZ;

    if(saveRawTof)
        tofPeakNum[0] = QStringLiteral("RawTof,");
    if(savePileUpTof)
        tofPeakNum[0].append(QStringLiteral("pileUpTof,"));
    if(saveFilterTof)
        tofPeakNum[0].append(QStringLiteral("FilterTof,"));
    if(saveRawPeak)
        tofPeakNum[0].append(QStringLiteral("RawPeak,"));
    if(saveFilterPeak)
        tofPeakNum[0].append(QStringLiteral("FilterPeak,"));
    if(saveX)
        tofPeakNum[0].append(QStringLiteral("X,"));
    if(saveY)
        tofPeakNum[0].append(QStringLiteral("Y(depth),"));
    if(saveZ)
        tofPeakNum[0].append(QStringLiteral("Z"));

    tofPeakNum[0].append("\n");


}


void DealUsb_msg::loadLocalArray()
{
    //加载数据的配置四个相关的配置矩阵的部分

    //加载tofOffsetArray.txt配置集
    QFile tofOffsetArray_file("tofOffsetArray.txt");
    QString tofOffsetArray_line[19210];
    QString log_str;
    if (tofOffsetArray_file.open(QIODevice::ReadOnly|QIODevice::Text))
    {
        QTextStream in(&tofOffsetArray_file);
        int i = 0;
        while (!in.atEnd())
        {
            tofOffsetArray_line[i] = in.readLine();
            i++;
        }
        tofOffsetArray_file.close();

        if(i<16383)
        {
            log_str = "[load conf file error]:tofOffsetArray.txt";
            emit Display_log_signal(log_str);

        }else{
            log_str = "[load conf file success]:tofOffsetArray.txt";
            emit Display_log_signal(log_str);
        }
    }else{
        log_str = "[load conf file error]:tofOffsetArray.txt";
        emit Display_log_signal(log_str);
    }




    //加载xf_position.txt配置集
    QFile xf_position_file("xf_position.txt");
    QString xf_position_line[300];
    if (xf_position_file.open(QIODevice::ReadOnly|QIODevice::Text))
    {
        QTextStream in(&xf_position_file);
        int i = 0;
        while (!in.atEnd())
        {
            xf_position_line[i] = in.readLine();
            i++;
        }
        xf_position_file.close();

        if(i<255)
        {
            log_str = "[load conf file error]:xf_position.txt";
            emit Display_log_signal(log_str);
        }else{
            log_str = "[load conf file success]:xf_position.txt";
            emit Display_log_signal(log_str);
        }
    }else{
        log_str = "[load conf file error]:xf_position.txt";
        emit Display_log_signal(log_str);
    }


    //加载yf_position.txt配置集
    QFile yf_position_file("yf_position.txt");
    QString yf_position_line[200];
    if (yf_position_file.open(QIODevice::ReadOnly|QIODevice::Text))
    {
        QTextStream in(&yf_position_file);
        int i = 0;
        while (!in.atEnd())
        {
            yf_position_line[i] = in.readLine();
            i++;
        }
        yf_position_file.close();

        if(i<63)
        {
            log_str = "[load conf file error]:yf_position.txt";
            emit Display_log_signal(log_str);
        }else {
            log_str = "[load conf file success]:yf_position.txt";
            emit Display_log_signal(log_str);
        }
    }else{
        log_str = "[load conf file error]:yf_position.txt";
        emit Display_log_signal(log_str);
    }



    for(int i=0;i<16384;i++)
    {
        tofOffsetArray[i] = tofOffsetArray_line[i].toFloat();
    }


    for(int i=0; i<256; i++)
    {
        xf_position[i] = xf_position_line[i].toFloat();
        //        qDebug()<<"xf_position = "<<i<<"   = "<<xf_position[i];
    }

    for(int i=0; i<64; i++)
    {
        yf_position[i] = yf_position_line[i].toFloat();
        //        qDebug()<<"yf_position = "<<i<<"  = "<<yf_position_line[i].toFloat();
    }



    //加载pileUp参数
    QFile pileUp_file("pileUp.txt");
    QString pileUp_line[10];
    if (pileUp_file.open(QIODevice::ReadOnly|QIODevice::Text))
    {
        QTextStream in(&pileUp_file);
        int i = 0;
        while (!in.atEnd())
        {
            pileUp_line[i] = in.readLine();
            i++;
        }
        pileUp_file.close();

        if(i<1)
        {
            log_str = "[load conf file error]:pileUp_filetxt";
            emit Display_log_signal(log_str);
        }else {

            QString lineText_1 = pileUp_line[0];
            QString lineText_2 = pileUp_line[1];
            QStringList lineTextList_1 = lineText_1.split(",");
            QStringList lineTextList_2 = lineText_2.split(",");
            int len1 = lineTextList_1.length();
            int len2 = lineTextList_2.length();
            if(len1 != len2)
            {
                log_str = "[load conf file error]:pileUp_filetxt";
                emit Display_log_signal(log_str);
                return;

            }
            cal.clear();
            val.clear();
            for(int i=0; i<len1;i++)
            {
                cal.push_back(lineTextList_1[i].toFloat());
                val.push_back(lineTextList_2[i].toFloat());
            }


            qDebug()<<"length = "<<cal.size();
            for(int i=0;i<cal.size();i++)
            {
                qDebug()<<" i="<<i<<"  "<<cal[i]<<"  "<<val[i];
            }



            log_str = "[load conf file success]:pileUp_file.txt";
            emit Display_log_signal(log_str);
        }

    }else{
        log_str = "[load conf file error]:pileUp_file.txt";
        emit Display_log_signal(log_str);
    }

}

//!
//! \brief DealUsb_msg::change_gain_slot
//! \param gain
//!改变图像的增益
void DealUsb_msg::change_gain_slot(float gain)
{
    gainImage_tof = gain;
}

//!
//! \brief change_tof_peak_slot
//!切换tof/peak的槽函数
void DealUsb_msg::change_tof_peak_slot()
{
    if(isTOF == true)
        isTOF = false;
    else
        isTOF = true;
}

//修改统计帧数的槽函数
void DealUsb_msg::alterStatisticFrameNum_slot(int num)
{
    statisticFrameNumber = num ;

}

//修改是否进行滤波的槽函数
void DealUsb_msg::isFilter_slot(bool isFiter)
{
    if(true == isFiter)
    {
        isFilterFlag = true;
    }else{
        isFilterFlag = false;
    }
}


//!
//! \brief DealUsb_msg::tof_filter_2
//!同步到点云信息的滤波方法
void DealUsb_msg::tof_filter_2()   //同步到点云信息
{
    /*******************对Tof 图像进行过滤0的滤波处理*******************************/
    vector<int> srcArray;
    vector<int> sortArray;
    vector<int> indexArray_x;
    vector<int> indexArray_y;
    QRgb tofColor;
    int gainIndex_tof;
    int cloudIndex;
    for(int imgX=3; imgX<157; imgX++)
    {
        for(int imgY=3; imgY<117; imgY++)
        {
            int tmp_peak = src_peak[imgX][imgY];
            for(int i=-2;i<2;i++)
            {
                for(int j=-2;j<2;j++)
                {
                    if(src_peak[imgX+i][imgY+j] > tmp_peak-5  && src_peak[imgX+i][imgY+j] < tmp_peak+5)
                    {
                        srcArray.push_back(src_peak[imgX+i][imgY+j]);   //原始顺序
                        sortArray.push_back(src_peak[imgX+i][imgY+j]);   //存储符合peak的值
                        indexArray_x.push_back(imgX+i);
                        indexArray_y.push_back(imgY+j);
                    }
                }
            }
            int len = sortArray.size();
            if(len>0)
            {
                sort(sortArray.begin(),sortArray.end());
                int midNumTof = sortArray[len/2];
                for(int k=0;k<len; k++)
                {
                    if (midNumTof == srcArray[k])
                    {
                        int tmpx = indexArray_x[k];
                        int tmpy = indexArray_y[k];

                        //                        qDebug()<<"srcTof"<<src_tof[imgX][imgY]<<"  res ="<<src_tof[tmpx][tmpy]<<" len="<<len;
                        //更新TOF 图像的像素值
                        gainIndex_tof = src_tof[tmpx][tmpy]*gainImage_tof;
                        if(gainIndex_tof<1024 && gainIndex_tof>=0)
                            tofColor = qRgb(colormap[gainIndex_tof * 3], colormap[gainIndex_tof * 3 + 1], colormap[gainIndex_tof * 3 + 2]);
                        else
                            tofColor = qRgb(colormap[1023 * 3], colormap[1023 * 3 + 1], colormap[1023 * 3 + 2]);
                        microQimage.setPixel(imgX,imgY,tofColor);         //TOF图像的赋值

                        //                        //更新点云数据的值 以及颜色
                        //                        cloudIndex = imgY*160 + imgX;
                        //                        temp_y = calibration_y(src_tof[tmpx][tmpy],imgX,imgY);         //tof转换depth公式
                        //                        temp_x = calibration_x(temp_y,imgX,imgY);
                        //                        temp_z = calibration_z(temp_y,imgX,imgY);
                        //                        QColor mColor = QColor(tofColor);
                        //                        r = mColor.red();
                        //                        g = mColor.green();
                        //                        b = mColor.blue();
                        //                        rgb = ((int)r << 16 | (int)g << 8 | (int)b);
                        //                        tempRgbCloud.points[cloudIndex].x = temp_x;
                        //                        tempRgbCloud.points[cloudIndex].y = temp_y;
                        //                        tempRgbCloud.points[cloudIndex].z = temp_z;
                        //                        tempRgbCloud.points[cloudIndex].rgb = *reinterpret_cast<float*>(&rgb);

                    }
                }
            }
            srcArray.clear();
            sortArray.clear();
            indexArray_x.clear();
            indexArray_y.clear();


        }//imgY
    }//imgX
}



void DealUsb_msg::alter_focal_integrate_slot(float focal_length,float tx_rx_diff,int integrateNum)   //修改焦距、积分次数的槽函数
{
    f = focal_length;    //焦距
    camera_dis = tx_rx_diff;   //tx_RX间距
    integrateNum_ = integrateNum;    // 公式为  peak/积分次数 * 100

    qDebug()<<"f = "<<f;
    qDebug()<<"camers_diff = "<<camera_dis<<"   jifenNum="<<integrateNum_;

}



void DealUsb_msg::tof_filter()
{
    vector<int> sortArray;
    vector<int> sortArray_3D;

    QRgb tofColor;
    int gainIndex_tof;
    int cloudIndex;

    int winSize = 2;
    int win3D_size = 4;

    for(int imgX=0; imgX<256; imgX++)
    {
        for(int imgY=0; imgY<64; imgY++)
        {
            if((0!=src_tof[0][imgY]) && (0!=src_tof[1][imgY]) && (0!=src_tof[2][imgY]) && (0!=src_tof[3][imgY]) && (0!=src_tof[4][imgY])  && (0!=src_tof[5][imgY]))       //判断该行前5个数据，如果都为0 说明该行为空，未开始行
            {
                for(int i=-winSize/2; i<winSize/2;i++)
                {
                    for(int j=-winSize/2; j<winSize/2; j++)
                    {
                        if(imgX+i>=0 && imgX+i<256 && imgY+j>=0 && imgY+j<64)
                        {
                            sortArray.push_back(src_tof[imgX+i][imgY+j]);   //存储符合peak的值
                        }// if imgX+i
                    } // for j
                } //for i

                int len = sortArray.size();

                if(len>0)
                {
                    sort(sortArray.begin(),sortArray.end());
                    int midNumTof = sortArray[len/2];   //获取到的中值

                    //把中值赋值给 lineMean_tof
                    lineMean_tof[imgX][imgY] = midNumTof;
                }
                sortArray.clear();


            } // if src_tof[0][imgY]
        } //for imgY
    }//for imgX









//    for(int imgX=0; imgX<256; imgX++)
//    {
//        for(int imgY=0; imgY<64; imgY++)
//        {
//            for(int i=-winSize/2;i<winSize/2;i++)
//            {
//                for(int j=-winSize/2;j<winSize/2;j++)
//                {
//                    if(imgX+i>=0 && imgX+i<256 && imgY+j>=0 && imgY+j<64)
//                    {
//                        sortArray.push_back(src_tof[imgX+i][imgY+j]);   //存储符合peak的值
//                    }
//                }
//            }
//            int len = sortArray.size();

//            if(len>0)
//            {
//                sort(sortArray.begin(),sortArray.end());
//                int midNumTof = sortArray[len/2];   //获取到的中值

//                gainIndex_tof = midNumTof * gainImage_tof;
//                if(gainIndex_tof<1024 && gainIndex_tof>=0)
//                    tofColor = qRgb(colormap[gainIndex_tof * 3], colormap[gainIndex_tof * 3 + 1], colormap[gainIndex_tof * 3 + 2]);
//                else
//                    tofColor = qRgb(colormap[1023 * 3], colormap[1023 * 3 + 1], colormap[1023 * 3 + 2]);
//                microQimage.setPixel(imgX,imgY,tofColor);         //TOF图像的赋值

//            }
//            sortArray.clear();

//            //以上是对二维图像采用的 窗口大小为3的窗口的滤波

//            for(int m=-win3D_size/2; m<win3D_size/2; m++ )
//            {
//                for(int n=-win3D_size/2; n<win3D_size/2; n++)
//                {
//                    if(imgX+m>=0 && imgX+m<256 && imgY+n>=0 && imgY+n<64)
//                    {
//                        sortArray_3D.push_back(src_tof[imgX+m][imgY+n]);

//                    }
//                }
//            }
//            int len_3D = sortArray_3D.size();
//            if(len_3D > 0)
//            {
//                sort(sortArray_3D.begin(),sortArray_3D.end());
//                int midNumTof_3D = sortArray_3D[len_3D/2];   //获取到的中值

//                gainIndex_tof = midNumTof_3D * gainImage_tof;
//                if(gainIndex_tof<1024 && gainIndex_tof>=0)
//                    tofColor = qRgb(colormap[gainIndex_tof * 3], colormap[gainIndex_tof * 3 + 1], colormap[gainIndex_tof * 3 + 2]);
//                else
//                    tofColor = qRgb(colormap[1023 * 3], colormap[1023 * 3 + 1], colormap[1023 * 3 + 2]);


//                //更新点云数据的值 以及颜色
//                cloudIndex = imgY*256 + imgX;
//                temp_y = calibration_y(midNumTof_3D,imgX,imgY);         //tof转换depth公式
//                temp_x = calibration_x(temp_y,imgX,imgY);
//                temp_z = calibration_z(temp_y,imgX,imgY);

//                //点云数据做校正integrateNum_
//                temp_y = temp_y + tofOffsetArray[cloudIndex];
//                temp_x = temp_x + calibration_x(tofOffsetArray[cloudIndex],imgX,imgY);
//                temp_z = temp_z + calibration_z(tofOffsetArray[cloudIndex],imgX,imgY);

//                if(temp_y<0)
//                {
//                    temp_y = 0;
//                    temp_x = 0;
//                    temp_z = 0;
//                }

//                QColor mColor = QColor(tofColor);
//                r = mColor.red();
//                g = mColor.green();
//                b = mColor.blue();
//                rgb = ((int)r << 16 | (int)g << 8 | (int)b);
//                tempRgbCloud.points[cloudIndex].x = temp_x;
//                tempRgbCloud.points[cloudIndex].y = temp_y;
//                tempRgbCloud.points[cloudIndex].z = temp_z;
//                tempRgbCloud.points[cloudIndex].rgb = *reinterpret_cast<float*>(&rgb);
//            }
//            sortArray_3D.clear();


//        }//imgY
//    }//imgX

}





//!
//! \brief DealUsb_msg::toShowAll18Line_slot
//! 显示全部的18行数据  12-20  对应于像素 24 - 41 一共18行数据
void  DealUsb_msg::toShowAll18Line_slot()
{
    int cloudIndex = 0;
//    int mean_offset = 100;     //两帧之间大于某个值时 ，就不做平均
    float resTof = 0;
    float resPeak = 0;
    float savePeak = 0;
    int showLine_index = 24;   //两行取平均的话，显示64/2=32行  起始行为16
    for(int i=0; i<64; i++)
    {
        for(int j=0;j<256; j++)
        {

            if(lineMean_tof[j][i]>0)
            {
                // 1 根据阈值来判断是否需要做平均
                resTof = lineMean_tof[j][i];
                resPeak = lineMean_peak[j][i];
                savePeak = lineMean_srcIntensity[j][i];



               // 2 赋值到二维图像
               //设置TOF图像、强度图像的颜色
               QRgb tofColor,intenColor;
               int gainIndex_tof = resTof*gainImage_tof;
               int gainIndex_intensity =resPeak * gainImage_peak;
               if(gainIndex_tof<1024 && gainIndex_tof>=0)
                   tofColor = qRgb(colormap[gainIndex_tof * 3], colormap[gainIndex_tof * 3 + 1], colormap[gainIndex_tof * 3 + 2]);
               else
                   tofColor = qRgb(colormap[1023 * 3], colormap[1023 * 3 + 1], colormap[1023 * 3 + 2]);

               if(gainIndex_intensity<1024 && gainIndex_intensity>=0)
                   intenColor = qRgb(colormap[gainIndex_intensity * 3], colormap[gainIndex_intensity * 3 + 1], colormap[gainIndex_intensity * 3 + 2]);
               else
                   intenColor = qRgb(colormap[1023 * 3], colormap[1023 * 3 + 1], colormap[1023 * 3 + 2]);

               microQimage.setPixel(j,i,tofColor);         //TOF图像的赋值
               macroQimage.setPixel(j,i,intenColor);       //强度图像的赋值


               // 3 获取 点云数据的标号
               cloudIndex = i*256 + j;   //两行中始终选取下面的一行
//               qDebug()<<"cloudIndex = "<<cloudIndex;


               // 4 同步到三维点云图像当中
               /************点云数据相关************/
               temp_y = calibration_y(resTof,j,i+1);         //tof转换depth公式
               temp_x = calibration_x(temp_y,j,i+1);
               temp_z = calibration_z(temp_y,j,i+1);
                                                             /**************点云校正**********/
               temp_y = temp_y + tofOffsetArray[cloudIndex];
               temp_x = temp_x + calibration_x(tofOffsetArray[cloudIndex],j,i+1);
               temp_z = temp_z + calibration_z(tofOffsetArray[cloudIndex],j,i+1);

               if(temp_y<=0)
               {
                   temp_x=0;
                   temp_y=0;
                   temp_z=0;
               }

               if(resPeak <peakOffset)
               {
                   temp_x = 0;
                   temp_z = 0;
                   temp_y = 0;
               }

               mouseShowMutex.lock();
               mouseShowTOF[j][i] = resTof;
               mouseShowPEAK[j][i] = savePeak;
               mouseShowDepth[j][i] = temp_y;
               mouseShowMutex.unlock();


               QColor mColor = QColor(tofColor);
               r = mColor.red();
               g = mColor.green();
               b = mColor.blue();
               rgb = ((int)r << 16 | (int)g << 8 | (int)b);

               tempRgbCloud.points[cloudIndex].x = temp_x;
               tempRgbCloud.points[cloudIndex].y = temp_y;
               tempRgbCloud.points[cloudIndex].z = temp_z;
               tempRgbCloud.points[cloudIndex].rgb = *reinterpret_cast<float*>(&rgb);


               if(isSaveFlag == true)
               {
                   if(isSaveRawTof)
                   {
                       tofPeakNum[cloudIndex+1] = QString::number(resTof)+" ";
                   }
                   if(isSaveRawPeak)
                   {
                       tofPeakNum[cloudIndex+1].append(QString::number(savePeak)+" ");
                   }

                   if(isSaveX)
                   {
                       tofPeakNum[cloudIndex+1].append(QString::number(temp_x) + " ");
                   }
                   if(isSaveY)
                   {
                       tofPeakNum[cloudIndex+1].append(QString::number(temp_y) + " ");
                   }
                   if(isSaveZ)
                   {
                       tofPeakNum[cloudIndex+1].append(QString::number(temp_z));
                   }
                   tofPeakNum[cloudIndex+1].append("\n");

               }

            }
        }

        showLine_index++;
    }
}



/******两行合并成一行数据*************/
void DealUsb_msg::toShowOneLine_2_1_slot()
{

    int cloudIndex = 0;
//    int mean_offset = 20;     //两帧之间大于某个值时 ，就不做平均
    float resTof = 0;
    float resPeak = 0;
    float savePeak = 0;
    int showLine_index = 16;   //两行取平均的话，显示64/2=32行  起始行为16
    for(int i=0; i<63; i+=2)
    {
        for(int j=0;j<256; j++)
        {

            if(lineMean_tof[j][i]>0)
            {
                // 1 根据阈值来判断是否需要做平均
               if(abs(lineMean_tof[j][i] - lineMean_tof[j][i+1])>mean_offset)
               {
                   resTof = lineMean_tof[j][i+1];
                   resPeak = lineMean_peak[j][i+1];
                   savePeak = lineMean_srcIntensity[j][i+1];
               }else
               {
                   resTof = (lineMean_tof[j][i] + lineMean_tof[j][i+1])/2.0;
                   resPeak = (lineMean_peak[j][i] + lineMean_peak[j][i+1])/2.0;
                   savePeak = (lineMean_srcIntensity[j][i] + lineMean_srcIntensity[j][i+1])/2.0;
               }



               // 2 赋值到二维图像
               //设置TOF图像、强度图像的颜色
               QRgb tofColor,intenColor;
               int gainIndex_tof = resTof*gainImage_tof;
               int gainIndex_intensity =resPeak * gainImage_peak;
               if(gainIndex_tof<1024 && gainIndex_tof>=0)
                   tofColor = qRgb(colormap[gainIndex_tof * 3], colormap[gainIndex_tof * 3 + 1], colormap[gainIndex_tof * 3 + 2]);
               else
                   tofColor = qRgb(colormap[1023 * 3], colormap[1023 * 3 + 1], colormap[1023 * 3 + 2]);

               if(gainIndex_intensity<1024 && gainIndex_intensity>=0)
                   intenColor = qRgb(colormap[gainIndex_intensity * 3], colormap[gainIndex_intensity * 3 + 1], colormap[gainIndex_intensity * 3 + 2]);
               else
                   intenColor = qRgb(colormap[1023 * 3], colormap[1023 * 3 + 1], colormap[1023 * 3 + 2]);

               microQimage.setPixel(j,showLine_index,tofColor);         //TOF图像的赋值
               macroQimage.setPixel(j,showLine_index,intenColor);       //强度图像的赋值


               // 3 获取 点云数据的标号
               cloudIndex = (i+1)*256 + j;   //两行中始终选取下面的一行
//               qDebug()<<"cloudIndex = "<<cloudIndex;


               // 4 同步到三维点云图像当中
               /************点云数据相关************/
               temp_y = calibration_y(resTof,j,i+1);         //tof转换depth公式
               temp_x = calibration_x(temp_y,j,i+1);
               temp_z = calibration_z(temp_y,j,i+1);
                                                             /**************点云校正**********/
               temp_y = temp_y + tofOffsetArray[cloudIndex];
               temp_x = temp_x + calibration_x(tofOffsetArray[cloudIndex],j,i+1);
               temp_z = temp_z + calibration_z(tofOffsetArray[cloudIndex],j,i+1);

               if(temp_y<=0)
               {
                   temp_x=0;
                   temp_y=0;
                   temp_z=0;
               }

               if(resPeak <peakOffset)
               {
                   temp_x = 0;
                   temp_z = 0;
                   temp_y = 0;
               }


               mouseShowMutex.lock();
               mouseShowTOF[j][showLine_index] = resTof;
               mouseShowPEAK[j][showLine_index] = savePeak;
               mouseShowDepth[j][showLine_index] = temp_y;
               mouseShowMutex.unlock();

               QColor mColor = QColor(tofColor);
               r = mColor.red();
               g = mColor.green();
               b = mColor.blue();
               rgb = ((int)r << 16 | (int)g << 8 | (int)b);

               tempRgbCloud.points[cloudIndex].x = temp_x;
               tempRgbCloud.points[cloudIndex].y = temp_y;
               tempRgbCloud.points[cloudIndex].z = temp_z;
               tempRgbCloud.points[cloudIndex].rgb = *reinterpret_cast<float*>(&rgb);


               if(isSaveFlag == true)
               {
                   if(isSaveRawTof)
                   {
                       tofPeakNum[cloudIndex+1] = QString::number(resTof)+" ";
                   }
                   if(isSaveRawPeak)
                   {
                       tofPeakNum[cloudIndex+1].append(QString::number(savePeak)+" ");
                   }

                   if(isSaveX)
                   {
                       tofPeakNum[cloudIndex+1].append(QString::number(temp_x) + " ");
                   }
                   if(isSaveY)
                   {
                       tofPeakNum[cloudIndex+1].append(QString::number(temp_y) + " ");
                   }
                   if(isSaveZ)
                   {
                       tofPeakNum[cloudIndex+1].append(QString::number(temp_z));
                   }
                   tofPeakNum[cloudIndex+1].append("\n");

               }



            }
        }

        showLine_index++;
    }
}



//三行数据合并成一行
void DealUsb_msg::toShowOneLine_3_1_slot()     //三行合并成一行数据
{
    int cloudIndex = 0;
//    int mean_offset = 20;     //两帧之间大于某个值时 ，就不做平均
    float resTof = 0;
    float resPeak = 0;
    float savePeak = 0;
    int showLine_index = 22;   //三行取平均的话，显示64/3=32行  起始行为22

    for(int i=0; i<62; i+=3)
    {
        for(int j=0;j<256; j++)
        {

            if(lineMean_tof[j][i]>0)
            {
                // 1 根据阈值来判断是否需要做平均
               if((abs(lineMean_tof[j][i] - lineMean_tof[j][i+1])>mean_offset) || (abs(lineMean_tof[j][i] - lineMean_tof[j][i+2])>mean_offset) || (abs(lineMean_tof[j][i+1] - lineMean_tof[j][i+2])>mean_offset))
               {
                   resTof = lineMean_tof[j][i+2];
                   resPeak = lineMean_peak[j][i+2];
                   savePeak = lineMean_srcIntensity[j][i+2];
               }else
               {
                   resTof = (lineMean_tof[j][i] + lineMean_tof[j][i+1] + lineMean_tof[j][i+2])/3.0;
                   resPeak = (lineMean_peak[j][i] + lineMean_peak[j][i+1] + lineMean_tof[j][i+2])/3.0;
                   savePeak = (lineMean_srcIntensity[j][i]+lineMean_srcIntensity[j][i+1]+lineMean_srcIntensity[j][i+2])/3.0;
               }



               // 2 赋值到二维图像
               //设置TOF图像、强度图像的颜色
               QRgb tofColor,intenColor;
               int gainIndex_tof = resTof*gainImage_tof;
               int gainIndex_intensity =resPeak * gainImage_peak;
               if(gainIndex_tof<1024 && gainIndex_tof>=0)
                   tofColor = qRgb(colormap[gainIndex_tof * 3], colormap[gainIndex_tof * 3 + 1], colormap[gainIndex_tof * 3 + 2]);
               else
                   tofColor = qRgb(colormap[1023 * 3], colormap[1023 * 3 + 1], colormap[1023 * 3 + 2]);

               if(gainIndex_intensity<1024 && gainIndex_intensity>=0)
                   intenColor = qRgb(colormap[gainIndex_intensity * 3], colormap[gainIndex_intensity * 3 + 1], colormap[gainIndex_intensity * 3 + 2]);
               else
                   intenColor = qRgb(colormap[1023 * 3], colormap[1023 * 3 + 1], colormap[1023 * 3 + 2]);

               microQimage.setPixel(j,showLine_index,tofColor);         //TOF图像的赋值
               macroQimage.setPixel(j,showLine_index,intenColor);       //强度图像的赋值


               // 3 获取 点云数据的标号
               cloudIndex = (i+2)*256 + j;   //两行中始终选取下面的一行
//               qDebug()<<"cloudIndex = "<<cloudIndex;

               // 4 同步到三维点云图像当中
               /************点云数据相关************/
               temp_y = calibration_y(resTof,j,i+1);         //tof转换depth公式
               temp_x = calibration_x(temp_y,j,i+1);
               temp_z = calibration_z(temp_y,j,i+1);
                                                             /**************点云校正**********/
               temp_y = temp_y + tofOffsetArray[cloudIndex];
               temp_x = temp_x + calibration_x(tofOffsetArray[cloudIndex],j,i+1);
               temp_z = temp_z + calibration_z(tofOffsetArray[cloudIndex],j,i+1);

               if(temp_y<=0)
               {
                   temp_x=0;
                   temp_y=0;
                   temp_z=0;
               }

               if(resPeak <peakOffset)
               {
                   temp_x = 0;
                   temp_z = 0;
                   temp_y = 0;
               }

               mouseShowMutex.lock();
               mouseShowTOF[j][showLine_index] = resTof;
               mouseShowPEAK[j][showLine_index] = savePeak;
               mouseShowDepth[j][showLine_index] = temp_y;
               mouseShowMutex.unlock();


               QColor mColor = QColor(tofColor);
               r = mColor.red();
               g = mColor.green();
               b = mColor.blue();
               rgb = ((int)r << 16 | (int)g << 8 | (int)b);

               tempRgbCloud.points[cloudIndex].x = temp_x;
               tempRgbCloud.points[cloudIndex].y = temp_y;
               tempRgbCloud.points[cloudIndex].z = temp_z;
               tempRgbCloud.points[cloudIndex].rgb = *reinterpret_cast<float*>(&rgb);


               if(isSaveFlag == true)
               {
                   if(isSaveRawTof)
                   {
                       tofPeakNum[cloudIndex+1] = QString::number(resTof)+" ";
                   }
                   if(isSaveRawPeak)
                   {
                       tofPeakNum[cloudIndex+1].append(QString::number(resPeak)+" ");
                   }

                   if(isSaveX)
                   {
                       tofPeakNum[cloudIndex+1].append(QString::number(temp_x) + " ");
                   }
                   if(isSaveY)
                   {
                       tofPeakNum[cloudIndex+1].append(QString::number(temp_y) + " ");
                   }
                   if(isSaveZ)
                   {
                       tofPeakNum[cloudIndex+1].append(QString::number(temp_z));
                   }
                   tofPeakNum[cloudIndex+1].append("\n");

               }
            }
        }

        showLine_index++;
    }
}


//!
//! \brief DealUsb_msg::sendShowMeanLine_slot
//! \param index
//! index = 0  显示18行
//! index = 1  显示两行取平均
//! index = 2 显示三行取平均
void DealUsb_msg::sendShowMeanLine_slot(int index)
{
    showLineMeanInt = index ;

    qDebug()<<" DealUsb_msg.showLineMeanInt = "<<showLineMeanInt;

    //把图像的数据清空
    // 1 图片的初始化

    mouseShowMutex.lock();
    for(int i=0;i<256; i++)
    {
        for(int j=0; j<64; j++)
        {
            macroQimage.setPixel(i,j,0);
            microQimage.setPixel(i,j,0);

            lineMean_tof[i][j] = 0;
            lineMean_peak[i][j] = 0;


            mouseShowTOF[i][j] = 0;
            mouseShowPEAK[i][j] = 0;
            mouseShowDepth[i][j] = 0;


//            mouseShowMutex.lock();
//            mouseShowTOF[i][j] = 0;     //鼠标显示处 的显示为 0
//            mouseShowPEAK[i[j] = 0;     //鼠标显示处 的显示为 0
//            mouseShowDepth[i][j] = 0;   //鼠标显示处 的显示为 0
//            mouseShowMutex.unlock();
        }
    }
    mouseShowMutex.unlock();
    //把 点云的数据清空
    for(int i=0;i<16384; i++)
    {
        tempRgbCloud.points[i].x = 0;
        tempRgbCloud.points[i].y = 0;
        tempRgbCloud.points[i].z = 0;

        tofPeakNum[1+i].clear();     //保存文件时 把相应的数据置为空
    }



}


// 160x120版本的协议     一个包324个字节 2个字节的spadNum, 2个字节的lineNum
// 从中间往上下两侧扩散
// 中间行为 59   0-59为上半部分   60-129为下半部分
//!
//! \brief DealUsb_msg::recvMsgSlot
//! \param array
//!     spadNum = 0,1,2,...119         一个spadNum 有160个点 一行数据的量 所以总共有120个点
//!  1、首先根据lineNum(0,1)来判断是属于上半部分，还是属于下半部分
//!  2、lineIndex = spadNum/4 = 0,1,2,...29      宏像素的第几个 因为是4个spadNum为一组
//!  3、InnerLineNum =  (spadNum%4)/2 = 0,1      spadNum%4 = 0 ,1,2,3       //宏像素的第一行还是第二行
//!  4、在160*120的图像中的行号为： rowImage = lineIndex*2 + InnerLineNum   (0-59)
//!                                上半区：59 - rowImage
//!                                下半区：60 + rowImage
//!
//!  5、colImage = i*2 + spadNum%2
void DealUsb_msg::recvMsgSlot(QByteArray array)
{
    tmpIndex++;

    if(tmpIndex<10)
    {
        return;
    }else
    {
        tmpIndex = 20;
    }




    int ret;
    char *MyBuffer;

    MyBuffer = array.data();

    int rowImg,colImg;
    int spadNum = (quint8)(MyBuffer[0]) +  (((quint8)(MyBuffer[1]))<<8);            // 0 1 2 3 4 5 6 7
    int line_number = (quint8)(MyBuffer[2]) +  (((quint8)(MyBuffer[3]))<<8);        // 0 1 2 3 4 5 ... 31



    lastLineNum = line_number;


    //    if(spadNum==0 && lastSpadNum==7)  //此时说明上一帧数据已经接收完毕，把整帧数据付给其他线程，供其显示，数据可以显示了
    if(spadNum < lastSpadNum)  //此时说明上一帧数据已经接收完毕，把整帧数据付给其他线程，供其显示，数据可以显示了
    {
        haveIndex++;




        //统计信息相关的 ，将统计信息的容器赋值给全局变量
        if(statisticStartFlag)
        {
            statisticMutex.lock();
            allStatisticTofPoints = tempStatisticTofPoints;
            allStatisticPeakPoints = tempStatisticPeakPoints;
            statisticMutex.unlock();
        }


        //向主线程中发送最大值、最小值等统计信息
        emit staticValueSignal(tofMin,tofMax,peakMin,peakMax,xMin,xMax,yMin,yMax,zMin,zMax);
        //重置变量
        tofMin = 10000;
        tofMax = -10000;
        peakMin = 10000;
        peakMax = -10000;
        xMin = 10000;
        xMax = -10000;
        yMin = 10000;
        yMax = -10000;
        zMin = 10000;
        zMax = -10000;



        if(true == isFilterFlag)
        {
            tof_filter();
        }


        if(0 == showLineMeanInt)
        {
            toShowAll18Line_slot();
        }else if(1 == showLineMeanInt)
        {
            toShowOneLine_2_1_slot();
        }else if(2 == showLineMeanInt)
        {
            toShowOneLine_3_1_slot();
        }



        // 1、将滤波功能放到这里进行实现，
        // 2、将滤波后的三维点云 同步到二维图像
//        if(true == isFilterFlag)
//        {

//            tof_filter();    //只针对tof的中值滤波算法，并同时同步到三维点云
//            //            tof_filter_2();
//            /*******************开启滤波功能*********************************/
//            //先用直通滤波把所有零点重复的零点过滤掉
//            /* pcl::PassThrough<pcl::PointXYZRGB> pass;                      //创建滤波器对象
//            pass.setInputCloud(tempRgbCloud.makeShared());                //设置待滤波的点云
//            pass.setFilterFieldName("y");                                 //设置在Z轴方向上进行滤波
//            pass.setFilterLimits(0, 0.10);                                //设置滤波范围(从最高点向下0.10米去除)
//            pass.setFilterLimitsNegative(true);                           //保留
//            pass.filter(tempRgbCloud_pass);                                   //滤波并存储
//            if(tempRgbCloud_pass.size()<1)
//                return;

//            //统计
//            tempRgbCloud_radius.clear();
//            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outrem(true);
//            outrem.setInputCloud(tempRgbCloud_pass.makeShared());
//            outrem.setMeanK(40);
//            outrem.setStddevMulThresh(0.25);
//            //40  0.1 不见前面噪点
//            outrem.filter(tempRgbCloud_radius);
//            int len = outrem.getRemovedIndices()->size();*/



//            //            QTime t1,t2;
//            //            pcl::VoxelGrid<pcl::PointXYZRGB> sor;//滤波处理对象
//            //            sor.setInputCloud(tempRgbCloud.makeShared());
//            //            sor.setLeafSize(0.03f, 0.03f, 0.03f);//设置滤波器处理时采用的体素大小的参数
//            //            sor.filter(tempRgbCloud_pass);

//            //            tempRgbCloud_radius.resize(0);
//            //            //            t1 = QTime::currentTime();
//            //            pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;      //设置为true以后才能获取到滤出的噪点的 个数以及点的序列号
//            //            outrem.setInputCloud(tempRgbCloud.makeShared());              //设置输入点云
//            //            outrem.setRadiusSearch(0.05);              //设置在0.8半径的范围内找邻近点
//            //            outrem.setMinNeighborsInRadius(5);       //设置查询点的邻近点集数小于2的删除  30
//            //            outrem.filter (tempRgbCloud);//执行条件滤波，存储结果到cloud_filtered



//            //            t2 = QTime::currentTime();
//            //            qDebug()<<"RadiusOutlierRemoval costs time = "<<  t1.msecsTo(t2) <<"ms"<<endl;

//            /*************************以上为滤波处理部分************************************************************/
//            mutex_3D.lock();
//            tofImage = microQimage;
//            intensityImage = macroQimage;
//            pcl::copyPointCloud(tempRgbCloud,pointCloudRgb);
//            mutex_3D.unlock();
//            /***************************************************************/

//        }else
        {                      //不进行滤波
            mutex_3D.lock();
            tofImage = microQimage;
            intensityImage = macroQimage;
            pcl::copyPointCloud(tempRgbCloud,pointCloudRgb);
            mutex_3D.unlock();
        }


        //判断是否保存数据
        if(isSaveFlag)
        {

            bool flag = true;
            for(int i=0; i<16385; i++)
            {
                tofPeakToSave_string.append(tofPeakNum[i]);
                //tofPeakNum[i].clear();    //这句代码屏蔽以后 ，当有遗漏数据的时候，会使用上一帧的数据进行存储
            }
            emit saveTXTSignal(tofPeakToSave_string);
            tofPeakToSave_string.clear();

        }

        isShowPointCloud = true;

    }//以上为处理完整的一帧数据


    int line_offset,row_offset;
    line_offset = spadNum / 2;           //取值 0 1 2 3 ；
    row_offset = (spadNum) % 2;      //表示是在第一行 还是在第二行

    for(int i=0; i<64; i++)
    {
        rowImg = line_number*2 + row_offset;
        colImg = i * 4 + line_offset;
        cloudIndex = rowImg*256 + colImg;

        //        int intensity;
        float src_intensity,raw_intensity,after_kalman_intensity,intensity;
        float tof,rawTof,after_pileup_tof,after_kalman_tof,after_offset_tof;     //tof：是用来做显示（二维、三维、最大最小值）（因为涉及到要进行校正）    tmpTof：用来存储本地数据 以及统计界面时候用


        if(isTOF == false)   //设置一个不可能的值
        {
            tof = quint8(MyBuffer[4 + i * 4]) + ((quint8(MyBuffer[4 + i * 4 +1]))<<8);
            intensity = quint8(MyBuffer[4 + i * 4 + 2]) + ((quint8(MyBuffer[4 + i * 4 + 3 ]))<<8);
            src_intensity = intensity;
            intensity = intensity/integrateNum_*100;
        }else
        {
            intensity = quint8(MyBuffer[4 + i * 4]) + ((quint8(MyBuffer[4 + i * 4 +1]))<<8);
            tof = quint8(MyBuffer[4 + i * 4 + 2]) + ((quint8(MyBuffer[4 + i * 4 + 3 ]))<<8);
            src_intensity = intensity;
            intensity = intensity/integrateNum_*100;
        }

        //1、首先对peak做kalman滤波   临时添加
        raw_intensity = intensity;
        if(true == isKalman)
        {
            intensity = kalmanFilter_peak_slot(raw_intensity,cloudIndex);
        }
        after_kalman_intensity = intensity;

        /*******************************************************************************/

        //循环赋值
        for(int n=0; n<averageNum-1; n++)
        {
            lastTOF[n][cloudIndex] = lastTOF[n+1][cloudIndex];
        }
        lastTOF[averageNum-1][cloudIndex] = tof;

        if(haveIndex >averageNum)
        {
            float zeroNum = 0;
            haveIndex = averageNum+1;
            float allTof_100 = 0;
            for(int k=0; k<averageNum; k++)     //100帧取平均   ，如果有0的数据则不进行平均处理
            {
                if(lastTOF[k][cloudIndex] == 0)
                {
                    zeroNum = zeroNum+1;
                }
                allTof_100 += lastTOF[k][cloudIndex];
            }
            if(zeroNum != averageNum)

                tof = allTof_100/(averageNum-zeroNum);
            //           qDebug()<<"  tof = "<<tof;
        }
        /************************************************************/


        /*********************pileUp 以及 offset 校正部分******************************/
        rawTof = tof;

        // 2、利用Peak值线性外插法可得对应校正量ΔTOF (Unit: mm)
        if(true == is_pileUp_flag)
        {
            tof = pileUp_calibration(rawTof,intensity);
        }
        after_pileup_tof = tof;                   //pileUp处理后的tof

        if(true == isKalman)
        {
            tof = kalmanFilter_tof_slot(tof,cloudIndex,intensity);
        }
        after_kalman_tof = tof;



//        //设置TOF图像、强度图像的颜色
//        //设置TOF图像、强度图像的颜色
//        QRgb tofColor,intenColor;
//        int gainIndex_tof = tof*gainImage_tof;
//        gainIndex_tof = gainIndex_tof>0 ?gainIndex_tof:0;
//        int gainIndex_intensity =intensity * gainImage_peak;
//        //        int gainIndex_intensity = pow(intensity,2) *gainImage_peak;
//        if(gainIndex_tof<1024 && gainIndex_tof>=0)
//            tofColor = qRgb(colormap[gainIndex_tof * 3], colormap[gainIndex_tof * 3 + 1], colormap[gainIndex_tof * 3 + 2]);
//        else
//            tofColor = qRgb(colormap[1023 * 3], colormap[1023 * 3 + 1], colormap[1023 * 3 + 2]);

//        if(gainIndex_intensity<1024 && gainIndex_intensity>=0)
//            intenColor = qRgb(colormap[gainIndex_intensity * 3], colormap[gainIndex_intensity * 3 + 1], colormap[gainIndex_intensity * 3 + 2]);
//        else
//            intenColor = qRgb(colormap[1023 * 3], colormap[1023 * 3 + 1], colormap[1023 * 3 + 2]);


        if(colImg>=0 && colImg<256 && rowImg>=0 && rowImg<64)
        {
//            microQimage.setPixel(colImg,rowImg,tofColor);         //TOF图像的赋值
//            macroQimage.setPixel(colImg,rowImg,intenColor);       //强度图像的赋值

            //tof filter 相关
            src_tof[colImg][rowImg] = tof;
            src_peak[colImg][rowImg] = intensity;


            //线阵取平均相关
            lineMean_tof[colImg][rowImg] = tof;
            lineMean_peak[colImg][rowImg] = intensity;
            lineMean_srcIntensity[colImg][rowImg] = src_intensity;


            /************点云数据相关************/
            temp_y = calibration_y(tof,colImg,rowImg);         //tof转换depth公式
            temp_x = calibration_x(temp_y,colImg,rowImg);
            temp_z = calibration_z(temp_y,colImg,rowImg);

            /**************点云校正**********/
            temp_y = temp_y + tofOffsetArray[cloudIndex];
            temp_x = temp_x + calibration_x(tofOffsetArray[cloudIndex],colImg,rowImg);
            temp_z = temp_z + calibration_z(tofOffsetArray[cloudIndex],colImg,rowImg);

            if(temp_y<=0)
            {
                temp_x=0;
                temp_y=0;
                temp_z=0;
            }

            /************鼠标点击处显示信息相关*************/
//            mouseShowMutex.lock();
//            mouseShowTOF[colImg][rowImg] = tof;
//            mouseShowPEAK[colImg][rowImg] = src_intensity;
//            mouseShowDepth[colImg][rowImg] = temp_y;
//            mouseShowMutex.unlock();
            /*****************************************/

            if(intensity <peakOffset)
            {
                temp_x = 0;
                temp_z = 0;
                temp_y = 0;
            }

            if(isOnlyCenterShow_flag)  //这里是只显示中间光强度比较大的区域 显示行数：12-52   显示列数：78-178
            {
                if(rowImg<40|| rowImg>80 || colImg<40 || colImg>120)
                {
                    temp_x = 0;
                    temp_y = 0;
                    temp_z = 0;
                }
            }

            if(true == isAutoCalibration_flag && temp_y>0 )
            {
                float mean_res_y = 0;
                vec_mean_y[cloudIndex].push_back(temp_y);
                if(vec_mean_y[cloudIndex].size() == calibration_mean_num)
                {
                    mean_res_y = std::accumulate(std::begin(vec_mean_y[cloudIndex]),std::end(vec_mean_y[cloudIndex]),0.0)/calibration_mean_num;
                    calibrate_offset_slot(cloudIndex,mean_res_y);
                    vec_mean_y[cloudIndex].clear();
                }
            }

            /*********文件保存相关*****************/
//            if(isSaveFlag == true)
//            {
//                if(isSaveRawTof)
//                {
//                    tofPeakNum[cloudIndex+1] = QString::number(rawTof)+" ";
//                }
//                if(isSavePileUpTof)
//                {
//                    tofPeakNum[cloudIndex+1].append(QString::number(after_pileup_tof)+" ");
//                }
//                if(isSaveFilterTof)
//                {
//                    tofPeakNum[cloudIndex+1].append(QString::number(tof)+" ");     //卡尔曼滤波以后 对像素进行了平移处理
//                }
//                if(isSaveRawPeak)
//                {
//                    tofPeakNum[cloudIndex+1].append(QString::number(src_intensity)+" ");
//                }
//                if(isSaveFilterPeak)
//                {
//                    tofPeakNum[cloudIndex+1].append(QString::number(intensity) + " ");
//                }
//                if(isSaveX)
//                {
//                    tofPeakNum[cloudIndex+1].append(QString::number(temp_x) + " ");
//                }
//                if(isSaveY)
//                {
//                    tofPeakNum[cloudIndex+1].append(QString::number(temp_y) + " ");
//                }
//                if(isSaveZ)
//                {
//                    tofPeakNum[cloudIndex+1].append(QString::number(temp_z));
//                }
//                tofPeakNum[cloudIndex+1].append("\n");

//            }

//            QColor mColor = QColor(tofColor);
//            r = mColor.red();
//            g = mColor.green();
//            b = mColor.blue();
//            rgb = ((int)r << 16 | (int)g << 8 | (int)b);

//            tempRgbCloud.points[cloudIndex].x = temp_x;
//            tempRgbCloud.points[cloudIndex].y = temp_y;
//            tempRgbCloud.points[cloudIndex].z = temp_z;
//            tempRgbCloud.points[cloudIndex].rgb = *reinterpret_cast<float*>(&rgb);

            /***************统计均值 、方差相关***********************/
            if(statisticStartFlag == true)
            {
                //判断每个点已经储存的个数，如果已经超过设定的范围，则进行循环储存；
                int offset = tempStatisticPeakPoints[cloudIndex].size() - statisticFrameNumber;
                if(offset >= 0)
                {
                    tempStatisticTofPoints[cloudIndex].erase(tempStatisticTofPoints[cloudIndex].begin(),tempStatisticTofPoints[cloudIndex].begin() + offset + 1);
                    tempStatisticPeakPoints[cloudIndex].erase(tempStatisticPeakPoints[cloudIndex].begin(),tempStatisticPeakPoints[cloudIndex].begin() + offset + 1);
                }
                //向每个点的容器中添加一个新的点,完成循环存储
                tempStatisticTofPoints[cloudIndex].push_back(rawTof);
                tempStatisticPeakPoints[cloudIndex].push_back(src_intensity);
            }

        }
        else
            qDebug()<<QStringLiteral("给像素赋值时出现异常 imgrow=")<<rowImg<<"   imgCol = "<<colImg<<endl;

    }
    lastSpadNum = spadNum ;
}



//!   为提高计算的精度 ，计算过程中统一采用毫米单位  最后转换为m来进行显示
//! \brief DealUsb_msg::calibration_y
//! \param cal_tof  单位LSB
//! \param x_pix    单位微米
//! \param y_pix    单位微米
//! \return
//!//tof值 x_pix的位置标号  y_pix的位置标号
float DealUsb_msg::calibration_y(float cal_tof,int x_pix,int y_pix)
{
//    if((cal_tof*LSB*1000) <= camera_dis)
//        return 0;

//    float tofLSB = cal_tof/2.0 * LSB *1000;   //mm       ps:校正公式中的tof为接收到的真实TOF的一半    03-20 alter
//    float tmp1 = 4*tofLSB*tofLSB - camera_dis*camera_dis;
//    float tmp2 = 4*tofLSB*sqrt(pow(xf_position[x_pix]*miu_meter/f,2) + pow(yf_position[y_pix]*miu_meter/f,2) + 1 ) + 2*xf_position[x_pix]*miu_meter*camera_dis/f;
//    float y = tmp1/tmp2;
//    y = y/1000.0;   //mm->m
//    return y;

    if((cal_tof*LSB)<= camera_dis)
    {
        return 0;
    }

    float tofLSB = cal_tof * LSB;
    float tmp1 = tofLSB*tofLSB - camera_dis*camera_dis;
    float tmp2 = 2*tofLSB*sqrt(pow(xf_position[x_pix]*miu_meter/f,2) + pow(yf_position[y_pix]*miu_meter/f,2) + 1 ) + 2*xf_position[x_pix]*miu_meter*camera_dis/f;
    float y = tmp1/tmp2;
    y = y/ 1000.0;
    return y;




}

//!//!   为提高计算的精度 ，计算过程中统一采用毫米单位  最后转换为m来进行显示
//! \brief DealUsb_msg::calibration_x
//! \param cal_y
//! \param x_pix
//! \param y_pix
//! \return
//! //计算后的y的值  x_pix的位置标号  y_pix的位置标号
float DealUsb_msg::calibration_x(float cal_y,int x_pix,int y_pix)
{
    cal_y = cal_y*1000;   //m->mm
    float x = -1 * cal_y * xf_position[x_pix]*miu_meter/f;
    x = x/1000.0;      //mm->m
    return -x;
}

//! //!   为提高计算的精度 ，计算过程中统一采用毫米单位  最后转换为m来进行显示
//! \brief DealUsb_msg::calibration_z
//! \param cal_y
//! \param x_pix
//! \param y_pix
//! \return
//!  //计算后的z的值  x_pix的位置标号  y_pix的位置标号
float DealUsb_msg::calibration_z(float cal_y,int x_pix,int y_pix)
{
    cal_y = cal_y*1000;
    float z = -1 * cal_y *yf_position[y_pix]*miu_meter/f;
    z = z/1000.0;      //mm->m
    return z;
}

//!
//! \brief DealUsb_msg::pileUp_calibration
//! \param srcTof
//! \param peak
//! \return
//!利用Peak值线性外插法可得对应校正量ΔTOF (Unit: mm)
float DealUsb_msg::pileUp_calibration(int srcTof,float peak)
{
    //    float a = 2.475;
    //    float b = 0.04636;
    //    float bias_tof = a * exp(b*peak)/31.0;
    //    float resTof = srcTof + bias_tof;

    //    return resTof;

    float a = cal[0];
    float b = cal[1];
    float c = cal[2];

    float biasTof = (a * pow(peak,b) +c)/15.55;

    float resTof_ = srcTof + biasTof;
//    qDebug()<<"a="<<a<<"  b="<<b<<"  c="<<c<<"peak ="<<peak<<"   biasTof="<<biasTof<<" dst = "<<QString::number(resTof_)<<"   src="<<srcTof;

    return resTof_;

    //    float cal[] = {0, 18.608, 27.9257, 42.1217, 54.9701, 67.1999, 81.18035, 91.876348, 96.589933, 98.880245, 99.529313, 99.7605085, 99.9482045, 110};
    //    float val[] = {0,  0,     2.12,    11.37,   56.45,   69.755,  144.285,   180.995,   305.605,   447.575,  468.335,   523.65,      693.385,   800};


//    qDebug()<<"  cal[0] = "<<cal[0]<<"  val[0]="<<val[0];

    if(cal.size()<1 || val.size()<1)
        return srcTof;


    float begin_tof,end_tof,offset_start,offset_end;
    float resTof = 0;
    float bias_tof = 0;

    //    qDebug()<<"len = "<<sizeof(cal)/sizeof(cal[0]);
    int len = cal.size();
    for(int i=0; i<len-1; i++)
    {
        if(peak>=cal[i] && peak<cal[i+1])
        {
            begin_tof = cal[i];
            end_tof = cal[1+i];
            offset_start = val[i]/31.0;
            offset_end = val[1+i]/31.0;
            bias_tof = (peak-begin_tof)*(offset_end-offset_start)/(end_tof-begin_tof) + offset_start;
            resTof = srcTof + bias_tof;
            //            qDebug()<<"resTof = "<<resTof;
            return  resTof;
        }
    }

    return srcTof + val[len-1]/31.0;

}


//!
//! \brief DealUsb_msg::kalmanFilter_peak_slot
//! \param srcPeak   原始peak值
//! \param index    点云序号
//! \return  滤波后的peak值
//!
float DealUsb_msg::kalmanFilter_peak_slot(float srcPeak,int index)
{

    float p1 = -0.01034;
    float p2 = 1.032;
    float p3 = -0.04257;

    if(0 == haveIndex)
    {
        peak_x_k[index] = srcPeak;
        return peak_x_k[index];
    }

    peak_kalman_R = p1 *srcPeak*srcPeak + p2*srcPeak+ p3;             //取值范围[1 15]
    peak_kalman_R = peak_kalman_R<1?1:peak_kalman_R;

    //更新 K[index]
    peak_p_k[index] = peak_kalman_F * peak_p_k[index] *  peak_kalman_F + peak_kalman_Q;
    peak_K[index] = peak_p_k[index] * peak_kalman_H /(peak_kalman_H * peak_p_k[index] * peak_kalman_H + peak_kalman_R);
    peak_p_k[index] = (1 - peak_K[index] * peak_kalman_H) * peak_p_k[index];
    //更新 peak_x_k[index]
    peak_x_k[index] = peak_kalman_F * peak_x_k[index];
    peak_x_k[index] = peak_x_k[index] + peak_K[index]*(srcPeak - peak_kalman_H * peak_x_k[index]);


    //    qDebug()<<"srcPeak="<<srcPeak<<"  resPeak="<<peak_x_k[index];
    return peak_x_k[index] ;
}





//!
//! \brief DealUsb_msg::kalmanFilter_slot
//! \param srcTof   原始Tof值
//! \param index    标号（0-19119）
//! \return peak    强度值
//! \return         滤波算法以后的tof
//! 卡尔曼滤波算法  TOF
float DealUsb_msg::kalmanFilter_tof_slot(float srcTof,int index,float peak)
{


    if(0 == haveIndex)   //第一帧数据,初始化xk为观测值
    {
        x_k[index] = srcTof;
        return x_k[index];
    }

    float tof_offset = kalmanOffset_para*(10.86/sqrt(peak)+0.22) ;

    if(abs(srcTof-x_k[index])>tof_offset)
        return srcTof;



    kalman_R = 85.0/peak;               //取值范围[1 15]170
    kalman_R = kalman_R<1?1:kalman_R;
    kalman_R = kalman_R>15?15:kalman_R;

    //    kalman_R = 5;

    //更新 K[index]
    p_k[index] = kalman_F * p_k[index] *  kalman_F + kalman_Q;
    K[index] = p_k[index] * kalman_H /(kalman_H * p_k[index] * kalman_H + kalman_R);
    p_k[index] = (1 - K[index] * kalman_H) * p_k[index];
    //更新 x_k[index]
    x_k[index] = kalman_F * x_k[index];
    x_k[index] = x_k[index] + K[index]*(srcTof - kalman_H * x_k[index]);

    //    qDebug()<<"srcTof="<<srcTof<<"  x_k="<<x_k[index];
    return x_k[index];
}









//!
//! \brief autoCalibration_Dialog::on_startCalibration_pushButton_clicked
//! 1、开始校正
//! 2、发送信号到数据处理线程
//! 3、首先清空tofOffsetArray数组，设置为0
//! 4、设置校正的 flag = true
//! 5、把 59*160+79=9519        59*160+80=9520   60*160+79= 9679  60*160+80=9680 的y值传入给处理函数                         （31 32）/（127 128）
//! 6、数据处理函数首先判断够不够100帧，满100帧时，分别取y的均值，分别逆运算为resTof
//! 7、计算理论的realTof值，根据用户指定的距离(单位:m);
//! 8、offset = realTof - resTof
//! 9、写入本地文件，程序重新设置tofOffsetArray数组，isAutoCalibration_flagfalse
void  DealUsb_msg::start_autoCalibration_slot(float meters)
{
    for(int i=0;i<16384;i++)
    {
        tofOffsetArray[i] = 0;
    }
    calibration_real_dis = meters;
    isAutoCalibration_flag = true;

}

//!
//! \brief DealUsb_msg::calibrate_offset_slot
//!  计算校验的offset
//! 1、获取100帧数据的y坐标的均值（中间4个点的）
//! 2、根据公式反推出realTof
//! 3、offset = realTof - tof
//!
//!
//!
//!
//! 59*160+79=9519        59*160+80=9520   60*160+79= 9679  60*160+80=9680


//  x = realTof*LSB*1000
//! y = calibration_real_dis *1000
//! a = camera_dis
//! M = xf*miu_meter/f
//! N = yf*miu_meter/f
//!
//! tmp1 = sqrt((pow(a,2) + 2*a*M*y + pow(y,2)*(M*M+N*N+1))/4)
//! tmp2 = y*sqrt(M*M+N*N+1)/2.0
//!
//! x = tmp2 + tmp1   ||  x= tmp2-tmp1
//!
//! 经过测试 tmp2<tmp1  所以最终结论是 x= tmp2+tmp1
void DealUsb_msg::calibrate_offset_slot(int index,float mean_tof)
{

    ishave_Four++;

    float res_y_offset = calibration_real_dis - mean_tof;
    y_offset[index] = QString::number(res_y_offset);
    if(ishave_Four == 4608)
    {
        QFile file("tofOffsetArray.txt");
        file.open(QIODevice::WriteOnly|QIODevice::Text);
        QTextStream out(&file);
        for(int i=0; i<16384; i++)
        {
            if(y_offset[i].isEmpty())
            {
                y_offset[i] = QString::number(0);
            }

            tofOffsetArray[i] = y_offset[i].toFloat();
            out<<y_offset[i].toLocal8Bit()<<endl;
        }


        ishave_Four = 0;
        QString msg = "raw_Depth="+QString::number(calibration_real_dis)+",dst_Depth="+QString::number(mean_tof)+",offset="+QString::number(res_y_offset);

        QString DisplayNote = "[Auto Calibration success]:"+msg;
        emit send_cali_success_signal(msg);         //发送给设置自动校准的界面
        emit Display_log_signal(DisplayNote);       //在日志显示窗口显示校准信息

        isAutoCalibration_flag = false;
    }

}



/********************************* **************** ****************  read local_file ************************ **************** **************** **************** *****************************/

void DealUsb_msg::playLocalFile_slot(QString sPath)
{
    filePath = sPath;

    if( NULL == localFile_timer)
    {
        localFile_timer = new QTimer();
        connect(localFile_timer,SIGNAL(timeout()),this,SLOT(readLocalPCDFile()));
    }
    fileIndex = 1;
    localFile_timer->start(80);

}

void DealUsb_msg::readLocalPCDFile()
{
    QString fileName;
    QString line[20000];
    QStringList tofPeakList;
    int imgRow,imgCol;
    float intensity;
    float tof,rawTof,after_pileup_tof,after_offset_tof;


    fileName = filePath + QString::number(fileIndex)+".txt";
    fileIndex++;

    QFile file(fileName);

    int countNum = 0;
    if (file.open(QIODevice::ReadOnly))
    {
        QTextStream in(&file);
        while (!in.atEnd())
        {
            line[countNum] = in.readLine();
            countNum++;
        }
        file.close();
    }else
    {
        qDebug()<<"read file error!"<<endl;
        //            localFile_timer->stop();
        fileIndex = 1;
        return;
    }
    for(int i=0; i<countNum; i++)            //去掉空的数据
    {

        if(line[i].isEmpty())
            continue;

        tofPeakList = line[i].split(",");
        if(tofPeakList.size()<2)
            return;

        if(isTOF)
        {
            tof = tofPeakList[0].toFloat();
            intensity = tofPeakList[1].toFloat();
        }else
        {
            intensity = tofPeakList[0].toFloat();
            tof = tofPeakList[1].toFloat();
        }

        //循环赋值
        for(int n=0; n<averageNum-1; n++)
        {
            lastTOF[n][i] = lastTOF[n+1][i];
        }
        lastTOF[averageNum-1][i] = tof;

        if(haveIndex >averageNum)
        {
            float zeroNum = 0;
            haveIndex = averageNum+1;
            float allTof_100 = 0;
            for(int k=0; k<averageNum; k++)     //100帧取平均   ，如果有0的数据则不进行平均处理
            {
                if(lastTOF[k][i] == 0)
                {
                    zeroNum = zeroNum+1;
                }
                allTof_100 += lastTOF[k][i];
            }
            if(zeroNum != averageNum)

                tof = allTof_100/(averageNum-zeroNum);
        }







        /*********************pileUp 以及 offset 校正部分******************************/
        rawTof = tof;
        if(true == is_pileUp_flag)
        {
            tof = pileUp_calibration(rawTof,intensity);
        }
        after_pileup_tof = tof;                   //pileUp处理后的tof
        tof = tof;
        tof = tof + tofOffsetArray[cloudIndex];   //offset校正以后的tof
        after_offset_tof = tof;



        //行列以及颜色传递给图像
        imgRow = i%256;
        imgCol = i/256;
        cloudIndex = i;      //在点云数据中的标号

        //            qDebug()<<"imgRow="<<imgRow<<"  imgCol="<<imgCol<<endl;

        //设置TOF图像、强度图像的颜色
        QRgb tofColor,intenColor;
        int gainIndex_tof = tof*gainImage_tof;
        //        int gainIndex_intensity =intensity * gainImage_peak;
        int gainIndex_intensity = intensity * gainImage_peak;


        if(gainIndex_tof<1024 && gainIndex_tof>=0)
            tofColor = qRgb(colormap[gainIndex_tof * 3], colormap[gainIndex_tof * 3 + 1], colormap[gainIndex_tof * 3 + 2]);
        else
            tofColor = qRgb(colormap[1023 * 3], colormap[1023 * 3 + 1], colormap[1023 * 3 + 2]);

        if(gainIndex_intensity<1024 && gainIndex_intensity>=0)
            intenColor = qRgb(colormap[gainIndex_intensity * 3], colormap[gainIndex_intensity * 3 + 1], colormap[gainIndex_intensity * 3 + 2]);
        else
            intenColor = qRgb(colormap[1023 * 3], colormap[1023 * 3 + 1], colormap[1023 * 3 + 2]);




        if(imgRow>=0 && imgRow<256 && imgCol>=0 && imgCol<64)
        {
            microQimage.setPixel(imgRow,imgCol,tofColor);         //TOF图像的赋值
            macroQimage.setPixel(imgRow,imgCol,intenColor);       //强度图像的赋值

            /************点云数据相关************/
            temp_y = calibration_y(tof,imgRow,imgCol);
            temp_x = calibration_x(temp_y,imgRow,imgCol);
            temp_z = calibration_z(temp_y,imgRow,imgCol);
            if(intensity<peakOffset)
            {
                temp_y = 0;
                temp_x = 0;
                temp_z = 0;
            }


            //            qDebug()<<"src_temp_x="<<temp_x<<"    src_temp_z="<<temp_z;
            temp_y = temp_y + tofOffsetArray[cloudIndex];
            temp_x = temp_x + calibration_x(tofOffsetArray[cloudIndex],imgRow,imgCol);
            temp_z = temp_z + calibration_z(tofOffsetArray[cloudIndex],imgRow,imgCol);
            //            qDebug()<<"res_temp_x="<<temp_x<<"    res_temp_z="<<temp_z;

            //            qDebug()<<temp_y;

            //自动校正相关
            if(true == isAutoCalibration_flag)
            {
                float mean_res_y = 0;
                vec_mean_y[cloudIndex].push_back(temp_y);
                if(vec_mean_y[cloudIndex].size() == calibration_mean_num)
                {
                    mean_res_y = std::accumulate(std::begin(vec_mean_y[cloudIndex]),std::end(vec_mean_y[cloudIndex]),0.0)/calibration_mean_num;
                    calibrate_offset_slot(cloudIndex,mean_res_y);
                    vec_mean_y[cloudIndex].clear();
                }
            }




            QColor mColor = QColor(tofColor);
            r = mColor.red();
            g = mColor.green();
            b = mColor.blue();
            rgb = ((int)r << 16 | (int)g << 8 | (int)b);
            tempRgbCloud.points[cloudIndex].x = temp_x;
            tempRgbCloud.points[cloudIndex].y = temp_y;
            tempRgbCloud.points[cloudIndex].z = temp_z;
            tempRgbCloud.points[cloudIndex].rgb = *reinterpret_cast<float*>(&rgb);

            //            qDebug()<<" cloudIndex = "<<cloudIndex<<endl;





            /************鼠标点击处显示信息相关*************/
            mouseShowMutex.lock();
            mouseShowTOF[imgRow][imgCol] = tof;
            mouseShowPEAK[imgRow][imgCol] = intensity;
            mouseShowDepth[imgRow][imgCol] = temp_y;
            mouseShowMutex.unlock();

            src_tof[imgRow][imgCol] = tof;
            src_peak[imgRow][imgCol] = intensity;


            /***************统计均值 、方差相关***********************/
            if(statisticStartFlag == true)
            {
                //判断每个点已经储存的个数，如果已经超过设定的范围，则进行循环储存；
                int offset = tempStatisticPeakPoints[cloudIndex].size() - statisticFrameNumber;
                if(offset >= 0)
                {
                    tempStatisticTofPoints[cloudIndex].erase(tempStatisticTofPoints[cloudIndex].begin(),tempStatisticTofPoints[cloudIndex].begin() + offset + 1);
                    tempStatisticPeakPoints[cloudIndex].erase(tempStatisticPeakPoints[cloudIndex].begin(),tempStatisticPeakPoints[cloudIndex].begin() + offset + 1);
                }
                //向每个点的容器中添加一个新的点,完成循环存储
                tempStatisticTofPoints[cloudIndex].push_back(tof);
                tempStatisticPeakPoints[cloudIndex].push_back(intensity);
            }


            /******统计点云空间坐标最大值、最小值**********/
            xMax = (temp_x>xMax) ? temp_x : xMax;
            xMin = (temp_x<xMin) ? temp_x : xMin;
            yMax = (temp_y>yMax) ? temp_y : yMax;
            yMin = (temp_y<yMin) ? temp_y : yMin;
            zMax = (temp_z>zMax) ? temp_z : zMax;
            zMin = (temp_z<zMin) ? temp_z : zMin;

            //统计二维图像
            tofMax = (tof>tofMax) ? tof : tofMax;
            tofMin = (tof<tofMin) ? tof : tofMin;
            peakMax = (intensity>peakMax) ? intensity : peakMax;
            peakMin = (intensity<peakMin) ? intensity : peakMin;
        }
        else
            qDebug()<<QStringLiteral("给像素赋值时出现异常 imgrow=")<<imgRow<<"   imgCol = "<<imgCol<<endl;

    }  //一帧数据已经读取完成


    haveIndex++;


    //统计信息相关的 ，将统计信息的容器赋值给全局变量
    if(statisticStartFlag)
    {
        statisticMutex.lock();
        allStatisticTofPoints = tempStatisticTofPoints;
        allStatisticPeakPoints = tempStatisticPeakPoints;
        statisticMutex.unlock();
    }


    //向主线程中发送最大值、最小值等统计信息
    emit staticValueSignal(tofMin,tofMax,peakMin,peakMax,xMin,xMax,yMin,yMax,zMin,zMax);
    //重置变量
    tofMin = 10000;
    tofMax = -10000;
    peakMin = 10000;
    peakMax = -10000;
    xMin = 10000;
    xMax = -10000;
    yMin = 10000;
    yMax = -10000;
    zMin = 10000;
    zMax = -10000;






    // 1、将滤波功能放到这里进行实现，
    // 2、将滤波后的三维点云 同步到二维图像
    if(true == isFilterFlag)
    {
        tof_filter();

        /*******************开启滤波功能*********************************/
        //先用直通滤波把所有零点重复的零点过滤掉
        //        pcl::PassThrough<pcl::PointXYZRGB> pass;                      //创建滤波器对象
        //        pass.setInputCloud(tempRgbCloud.makeShared());                //设置待滤波的点云
        //        pass.setFilterFieldName("y");                                 //设置在Z轴方向上进行滤波
        //        pass.setFilterLimits(0, 0.10);                                //设置滤波范围(从最高点向下0.10米去除)
        //        pass.setFilterLimitsNegative(true);                           //保留
        //        pass.filter(tempRgbCloud_pass);                                   //滤波并存储
        //        if(tempRgbCloud_pass.size()<1)
        //                return;

        //  统计
        //        tempRgbCloud_radius.clear();
        //        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outrem(true);
        //        outrem.setInputCloud(tempRgbCloud.makeShared());
        //        outrem.setMeanK(40);
        //        outrem.setStddevMulThresh(0.25);
        //        //40  0.1 不见前面噪点
        //        outrem.filter(tempRgbCloud_radius);
        //        int len = outrem.getRemovedIndices()->size();
        //        qDebug()<<"after filter the points'Number = "<<tempRgbCloud_radius.size()<<endl;


        //        tempRgbCloud_radius.resize(0);
        //        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem(true);      //设置为true以后才能获取到滤出的噪点的 个数以及点的序列号
        //        outrem.setInputCloud(tempRgbCloud.makeShared());              //设置输入点云
        //        outrem.setRadiusSearch(0.05);              //设置在0.8半径的范围内找邻近点   0.1   30
        //        outrem.setMinNeighborsInRadius(5);       //设置查询点的邻近点集数小于2的删除  30
        //        outrem.filter (tempRgbCloud_radius);//执行条件滤波，存储结果到cloud_filtered
        //        int len = outrem.getRemovedIndices()->size();

        /*************************以上为滤波处理部分************************************************************/

        /***********************接下来 根据点云的序号 去除二维图像中的噪声************************/

        mutex_3D.lock();
        tofImage = microQimage;
        intensityImage = macroQimage;
        pcl::copyPointCloud(tempRgbCloud,pointCloudRgb);
        mutex_3D.unlock();
        /***************************************************************/

    }else{                      //不进行滤波
        mutex_3D.lock();
        tofImage = microQimage;
        intensityImage = macroQimage;
        pcl::copyPointCloud(tempRgbCloud,pointCloudRgb);
        mutex_3D.unlock();
    }
    isShowPointCloud = true;
}



/*************************************************************Raw data 部分*********************************************************************v**************************************************/
/**********************************************************************************************************************************v**************************************************/

//!
//! \brief DealUsb_msg::receRawDataSave_slot
//! \param array
void DealUsb_msg::receRawDataSave_slot(QByteArray array)
{
    char *MyBuffer;
    MyBuffer = array.data();
    if(isFirst_frame)
    {
        current_id = (quint8)(MyBuffer[512]) +  (((quint8)(MyBuffer[513]))<<8);
        isFirst_frame = false;
        return;
    }
    int nextPackage_id = (quint8)(MyBuffer[512]) +  (((quint8)(MyBuffer[513]))<<8);
    int data1 = (quint8)(MyBuffer[0]) +  (((quint8)(MyBuffer[1]))<<8);
//    qDebug()<<"current_id="<<current_id<<"  nextPackage_id="<<nextPackage_id<<"  data1="<<data1;


    if(current_id < last_id) //一帧数据已经接收完毕
    {

        if(frame_index>rawData_saveFrameNums)
            return;

        frame_index++;

        //一帧已经接收完毕 ，开启保存
        for(int i=0; i<16384; i++)
        {
           rawDataResult_str.append(rawDataSingle_str[i]).append('\n');
           rawDataSingle_str[i].clear();
        }
        emit saveTXTSignal(rawDataResult_str);
        rawDataResult_str.clear();


        if(frame_index>1)
        {
            QString log_str = "[Recevive RawData frame:]" + QString::number(frame_index-1);
            emit Display_log_signal(log_str);
        }


    }



    //开始解析单包的数据
    int hongPixel_line = current_id/exposure_num%32;      //  0...31
    int pixel_index = current_id/exposure_num/32;        //  0....7

    int mic_line = pixel_index%2;   //宏像素的第一行 还是第二行   0  /  1
    int mic_col = pixel_index/2;   //宏像素的 第一列、第二列、第三列、第四列

    int pixel_row = hongPixel_line * 2 + mic_line;      //微像素所在的行
//    qDebug()<<"hongPixel_line = "<<hongPixel_line<<" pixel_index= "<<pixel_index<<"  pixel_row = "<<pixel_row<<"   mic_col = "<<mic_col;


    int index = 0;
    int tmpValue_0_1 = 0,tmpValue_0_2 = 0,tmpValue_0_3 = 0,tmpValue_0_4 = 0;
    int tmpValue_1_1 = 0,tmpValue_1_2 = 0,tmpValue_1_3 = 0,tmpValue_1_4 = 0;
    int tmpValue_2_1 = 0,tmpValue_2_2 = 0,tmpValue_2_3 = 0,tmpValue_2_4 = 0;
    int tmpValue_3_1 = 0,tmpValue_3_2 = 0,tmpValue_3_3 = 0,tmpValue_3_4 = 0;
    int cloudIndex = 0;
    for(int i=0; i<32; i+=2)   //每包包含16个数据 也就是32个字节  index =0...15
    {
        tmpValue_0_1 = (quint8)(MyBuffer[i]) +  (((quint8)(MyBuffer[i+1]))<<8);
        tmpValue_0_2 = (quint8)(MyBuffer[32+i]) +  (((quint8)(MyBuffer[32+i+1]))<<8);
        tmpValue_0_3 = (quint8)(MyBuffer[64+i]) +  (((quint8)(MyBuffer[64+i+1]))<<8);
        tmpValue_0_4 = (quint8)(MyBuffer[96+i]) +  (((quint8)(MyBuffer[96+i+1]))<<8);
        cloudIndex = pixel_row*256 + mic_col + (4*(index))*4 + 0*4;  //4代表宏像素的四列  4代表相隔4个TDC
        rawDataSingle_str[cloudIndex].append(QString::number(tmpValue_0_1)).append(" ").append(QString::number(tmpValue_0_2)).append(" ").append(QString::number(tmpValue_0_3)).append(" ").append(QString::number(tmpValue_0_4)).append(" ");
//        qDebug()<<"cloudIndex_1 = "<<cloudIndex;

        tmpValue_1_1 = (quint8)(MyBuffer[128+i]) +  (((quint8)(MyBuffer[128+i+1]))<<8);
        tmpValue_1_2 = (quint8)(MyBuffer[128+32+i]) +  (((quint8)(MyBuffer[128+32+i+1]))<<8);
        tmpValue_1_3 = (quint8)(MyBuffer[128+64+i]) +  (((quint8)(MyBuffer[128+64+i+1]))<<8);
        tmpValue_1_4 = (quint8)(MyBuffer[128+96+i]) +  (((quint8)(MyBuffer[128+96+i+1]))<<8);
        cloudIndex = pixel_row*256 + mic_col + (4*(index))*4 + 1*4;
        rawDataSingle_str[cloudIndex].append(QString::number(tmpValue_1_1)).append(" ").append(QString::number(tmpValue_1_2)).append(" ").append(QString::number(tmpValue_1_3)).append(" ").append(QString::number(tmpValue_1_4)).append(" ");
//        qDebug()<<"cloudIndex_2 = "<<cloudIndex;

        tmpValue_2_1 = (quint8)(MyBuffer[256+i]) +  (((quint8)(MyBuffer[256+i+1]))<<8);
        tmpValue_2_2 = (quint8)(MyBuffer[256+32+i]) +  (((quint8)(MyBuffer[256+32+i+1]))<<8);
        tmpValue_2_3 = (quint8)(MyBuffer[256+64+i]) +  (((quint8)(MyBuffer[256+64+i+1]))<<8);
        tmpValue_2_4 = (quint8)(MyBuffer[256+96+i]) +  (((quint8)(MyBuffer[256+96+i+1]))<<8);
        cloudIndex = pixel_row*256 + mic_col + (4*(index))*4 + 2*4;
        rawDataSingle_str[cloudIndex].append(QString::number(tmpValue_2_1)).append(" ").append(QString::number(tmpValue_2_2)).append(" ").append(QString::number(tmpValue_2_3)).append(" ").append(QString::number(tmpValue_2_4)).append(" ");
//        qDebug()<<"cloudIndex_3 = "<<cloudIndex;

        tmpValue_3_1 = (quint8)(MyBuffer[384+i]) +  (((quint8)(MyBuffer[384+i+1]))<<8);
        tmpValue_3_2 = (quint8)(MyBuffer[384+32+i]) +  (((quint8)(MyBuffer[384+32+i+1]))<<8);
        tmpValue_3_3 = (quint8)(MyBuffer[384+64+i]) +  (((quint8)(MyBuffer[384+64+i+1]))<<8);
        tmpValue_3_4 = (quint8)(MyBuffer[384+96+i]) +  (((quint8)(MyBuffer[384+96+i+1]))<<8);
        cloudIndex = pixel_row*256 + mic_col + (4*(index))*4 + 3*4;
        rawDataSingle_str[cloudIndex].append(QString::number(tmpValue_3_1)).append(" ").append(QString::number(tmpValue_3_2)).append(" ").append(QString::number(tmpValue_3_3)).append(" ").append(QString::number(tmpValue_3_4)).append(" ");
//        qDebug()<<"cloudIndex_4 = "<<cloudIndex;


        index++;
    }

    last_id = current_id;
    current_id  = nextPackage_id;

}


//开始接收rowData数据
void DealUsb_msg::on_start_rawDataSave_slot(QString fiePath)
{
//    currentRawDataFrame = 0;
    saveFilePath = fiePath;   //保存的路径  E:/..../.../的形式
    saveFileIndex = 1;      //文件标号；1作为开始
    qDebug()<<"rawData's savePath = "<<fiePath;


}


/************************** RowData MA 相关 ***********************************************/
void DealUsb_msg::start_RowDatahistogram_slot(int exposureNum,int IntegrationNum,int row,int col,bool flag)
{
    exposure_num = exposureNum;   //根据曝光次数来决定 没包数据的解析
    histogram_frame = IntegrationNum/exposureNum-1;   //根据积分次数 来决定 多少帧来统计一次直方图
    histogram_row = row;
    histogram_col = col;
    isFirst_frame = true;

    qDebug()<<"row ="<<row<<"  col="<<col<<"flag = "<<flag;
}


void DealUsb_msg::receRawData_MA_slot(QByteArray array)
{
    char *MyBuffer;
    MyBuffer = array.data();
    current_id = (quint8)(MyBuffer[0]) +  (((quint8)(MyBuffer[1]))<<8);                     // 0,......曝光此时
    current_package_num = (quint8)(MyBuffer[2]) +  (((quint8)(MyBuffer[3]))<<8);            // 0、1、2、3  包号码

    if(current_id< last_id  || isFirst_frame ==true)   //一帧数据已经解析完毕
    {
        if(!isFirst_frame)
            frame_index++;

        emit currentFrame_signal(frame_index);
        if(frame_index>histogram_frame)   //发送给可视化线程进行显示
        {
            emit toShowHistogram_signal(historgramVec_MA,histogram_maxValue);
            histogram_maxValue = 0;
            historgramVec_MA.clear();
            historgramVec_MA.resize(4096);
            frame_index = 0;

        }

        isFirst_frame =false;

        QString log_str = "[Recevive RawData frame:]" + QString::number(frame_index);
        emit Display_log_signal(log_str);
    }

    hongPixel_line = current_id/(4*exposure_num);   //4*20次曝光   //曝光次数可以设置       0....29
    pixel_index = current_id%(4*exposure_num)/exposure_num;

    int mic_line = pixel_index/2;  //宏像素的第一行 还是第二行
    int mic_col = pixel_index%2;   //宏像素的第一列 还是第二列
    row_up = 59 - ( 2*hongPixel_line + mic_line);      //上半部分像素所在的行
    row_down = 60 + ( 2*hongPixel_line + mic_line);    //下班部分像素所在的行

    if((row_up!=histogram_row)&&(row_down!=histogram_row))
        return;


    int col;    //像素所在的列
    int tmpValue_1,tmpValue_2,tmpValue_3,tmpValue_4;
    for(int i=0; i<40; i++)
    {
        if(i<20)     //上半区
        {
            col = mic_col + current_package_num*2 + i*8;         //0 8 16 24 ..... 152

            if(col!=histogram_col)
                continue;

            tmpValue_1 = quint8(MyBuffer[4 + i*2 ]) + ((quint8(MyBuffer[4 + i*2+1]))<<8);;
            tmpValue_2 = quint8(MyBuffer[4 +80+ i*2 ]) + ((quint8(MyBuffer[4 + 80+ i*2+1]))<<8);
            tmpValue_3 = quint8(MyBuffer[4 +160+ i*2 ]) + ((quint8(MyBuffer[4 + 160+ i*2+1]))<<8);
            tmpValue_4 = quint8(MyBuffer[4 +240+ i*2 ]) + ((quint8(MyBuffer[4 + 240+ i*2+1]))<<8);


            historgramVec_MA[tmpValue_1] = historgramVec_MA[tmpValue_1]+1;
            historgramVec_MA[tmpValue_2] = historgramVec_MA[tmpValue_2]+1;
            historgramVec_MA[tmpValue_3] = historgramVec_MA[tmpValue_3]+1;
            historgramVec_MA[tmpValue_4] = historgramVec_MA[tmpValue_4]+1;

            //            histogram_maxValue = historgramVec_MA[tmpValue_1]>histogram_maxValue ? historgramVec_MA[tmpValue_1]:histogram_maxValue;
            //            histogram_maxValue = historgramVec_MA[tmpValue_2]>histogram_maxValue ? historgramVec_MA[tmpValue_2]:histogram_maxValue;
            //            histogram_maxValue = historgramVec_MA[tmpValue_3]>histogram_maxValue ? historgramVec_MA[tmpValue_3]:histogram_maxValue;
            //            histogram_maxValue = historgramVec_MA[tmpValue_4]>histogram_maxValue ? historgramVec_MA[tmpValue_4]:histogram_maxValue;


        }else        //下半区
        {
            col = mic_col + current_package_num*2 + (i-20)*8;    //0 8 16 24  ..... 152

            if(col!=histogram_col)
                continue;

            tmpValue_1 = quint8(MyBuffer[4 + i*2 ]) + ((quint8(MyBuffer[4 + i*2+1]))<<8);;
            tmpValue_2 = quint8(MyBuffer[4 +80+ i*2 ]) + ((quint8(MyBuffer[4 + 80+ i*2+1]))<<8);
            tmpValue_3 = quint8(MyBuffer[4 +160+ i*2 ]) + ((quint8(MyBuffer[4 + 160+ i*2+1]))<<8);
            tmpValue_4 = quint8(MyBuffer[4 +240+ i*2 ]) + ((quint8(MyBuffer[4 + 240+ i*2+1]))<<8);

            historgramVec_MA[tmpValue_1] = historgramVec_MA[tmpValue_1]+1;
            historgramVec_MA[tmpValue_2] = historgramVec_MA[tmpValue_2]+1;
            historgramVec_MA[tmpValue_3] = historgramVec_MA[tmpValue_3]+1;
            historgramVec_MA[tmpValue_4] = historgramVec_MA[tmpValue_4]+1;

            //            histogram_maxValue = historgramVec_MA[tmpValue_1]>histogram_maxValue ? historgramVec_MA[tmpValue_1]:histogram_maxValue;
            //            histogram_maxValue = historgramVec_MA[tmpValue_2]>histogram_maxValue ? historgramVec_MA[tmpValue_2]:histogram_maxValue;
            //            histogram_maxValue = historgramVec_MA[tmpValue_3]>histogram_maxValue ? historgramVec_MA[tmpValue_3]:histogram_maxValue;
            //            histogram_maxValue = historgramVec_MA[tmpValue_4]>histogram_maxValue ? historgramVec_MA[tmpValue_4]:histogram_maxValue;
        }
    }


    last_id = current_id;
}
