#ifndef STATISTICSDIALOG_H
#define STATISTICSDIALOG_H

#include <QDialog>
#include"surfacegraph.h"
#include<QHBoxLayout>
#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMessageBox>
#include <QtGui/QPainter>
#include <QtGui/QScreen>
#include <QCloseEvent>


namespace Ui {
class statisticsDialog;
}

class statisticsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit statisticsDialog(QWidget *parent = 0);
    ~statisticsDialog();

    void initTofMean_slot();
    void initTofStd_slot();
    void initPeakMean_slot();
    void initPeakStd_slot();

    Q3DSurface *tofMean_graph;
    QWidget *tofMean_container;
    SurfaceGraph *tofMean_modifier;

    Q3DSurface *tofStd_graph;
    QWidget *tofStd_container;
    SurfaceGraph *tofStd_modifier;

    Q3DSurface *peakMean_graph;
    QWidget *peakMean_container;
    SurfaceGraph *peakMean_modifier;

    Q3DSurface *peakStd_graph;
    QWidget *peakStd_container;
    SurfaceGraph *peakStd_modifier;



    bool isRun;
    void closeEvent(QCloseEvent *event);
private slots:
    void on_start_pushButton_clicked();

    void on_start_pushButton_2_clicked();

     void statistic_MeanStdSlot(QStringList,QStringList,QStringList,QStringList);

signals:
     void alterStatisticFrameNum_signal(int);

     void sendTofMean_signal(QStringList);

     void sendTofStd_signal(QStringList);

     void sendPeakMean_signal(QStringList);

     void sendPeakStd_signal(QStringList);

     void startStop_signal(int);



     void sendTofMeanclear_signal();

     void sendTofStdclear_signal();

     void sendPeakMeanclear_signal();

     void sendPeakStdclear_signal();


private:
    Ui::statisticsDialog *ui;
};

#endif // STATISTICSDIALOG_H
