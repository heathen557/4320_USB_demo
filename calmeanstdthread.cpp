#include "calmeanstdthread.h"
#include<qdebug.h>


extern QMutex statisticMutex;
extern vector<vector<int>> allStatisticTofPoints;   //用于统计 均值和方差的 容器  TOF
extern vector<vector<int>> allStatisticPeakPoints;   //用于统计 均值和方差的 容器  TOF

calMeanStdThread::calMeanStdThread(QObject *parent) : QObject(parent)
{
    updateTimer = NULL;
}


void calMeanStdThread::startStop_slot(int flag)
{
    if(NULL == updateTimer)
    {
        updateTimer = new QTimer();
        connect(updateTimer,SIGNAL(timeout()),this,SLOT(updateSlot()));
    }
    if(1 == flag)
    {
        updateTimer->start(1000);
    }else {
        updateTimer->stop();
    }
}


//刷新均值和方差的槽函数
void calMeanStdThread::updateSlot()
{
    statisticMutex.lock();
    useStatisticTofPoints = allStatisticTofPoints;
    useStatisticPeakPoints = allStatisticPeakPoints;
    statisticMutex.unlock();

    tofMean_string.clear();
    tofStd_string.clear();
    peakMean_string.clear();
    peakStd_string.clear();



    for(int i =0; i<16384; i++)
    {
        frameSize = useStatisticTofPoints[i].size();

//        qDebug()<<QStringLiteral("统计的帧数为： ")<< frameSize<<endl;

        if(frameSize>1)                              //理论上应该与设置的帧数相等，此处是为了防止除数为零时引起程序异常；
        {
            //tof的均值 标准差的统计
            tofMean = std::accumulate(std::begin(useStatisticTofPoints[i]),std::end(useStatisticTofPoints[i]),0.0)/frameSize;
            tofAccum = 0.0;
            std::for_each (std::begin(useStatisticTofPoints[i]), std::end(useStatisticTofPoints[i]), [&](const double d) {
                    tofAccum  += (d-tofMean)*(d-tofMean);
                });
            tofStd = sqrt(tofAccum/(frameSize-1));


            //peak的均值 标准差的统计
            peakMean = std::accumulate(std::begin(useStatisticPeakPoints[i]),std::end(useStatisticPeakPoints[i]),0.0)/frameSize;
            peakAccum = 0.0;
            std::for_each (std::begin(useStatisticPeakPoints[i]), std::end(useStatisticPeakPoints[i]), [&](const double d) {
                    peakAccum  += (d-peakMean)*(d-peakMean);
                });
            peakStd = sqrt(peakAccum/(frameSize-1));

            tofMean_string.append(QString::number(tofMean));
            tofStd_string.append(QString::number(tofStd));
            peakMean_string.append(QString::number(peakMean));
            peakStd_string.append(QString::number(peakStd));
        }
    }


    emit statistic_MeanStdSignal(tofMean_string,tofStd_string,peakMean_string,peakStd_string);

    qDebug()<<QStringLiteral("统计信息信号已经发出")<<endl;


}
