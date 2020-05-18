#ifndef CALMEANSTDTHREAD_H
#define CALMEANSTDTHREAD_H

#include <QObject>
#include<numeric>
#include<vector>
#include<QMutex>
#include<QTimer>
#include<omp.h>
using namespace std;

class calMeanStdThread : public QObject
{
    Q_OBJECT
public:
    explicit calMeanStdThread(QObject *parent = 0);

    QTimer *updateTimer;

    float tofMean;
    float tofAccum;
    float tofStd;

    float peakMean;
    float peakAccum;
    float peakStd;

    int frameSize;

    vector<vector<int>> useStatisticTofPoints;
    vector<vector<int>> useStatisticPeakPoints;

    QStringList tofMean_string;
    QStringList tofStd_string;

    QStringList peakMean_string;
    QStringList peakStd_string;

signals:

    void statistic_MeanStdSignal(QStringList,QStringList,QStringList,QStringList);

public slots:
    void updateSlot();      //刷新均值和方差的槽函数

    void startStop_slot(int flag);
};

#endif // CALMEANSTDTHREAD_H
