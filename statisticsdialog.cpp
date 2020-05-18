#include "statisticsdialog.h"
#include "ui_statisticsdialog.h"
#include<QDebug>



statisticsDialog::statisticsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::statisticsDialog)
{
    ui->setupUi(this);

    setWindowFlags(Qt::Widget);
    isRun = false;

    initTofMean_slot();
    initTofStd_slot();
    initPeakMean_slot();
    initPeakStd_slot();

}

//初始化TOF均值界面
void statisticsDialog::initTofMean_slot()
{

    tofMean_graph = new Q3DSurface();
    tofMean_container = QWidget::createWindowContainer(tofMean_graph);

    //! [0]

    if (!tofMean_graph->hasContext()) {
        QMessageBox msgBox;
        msgBox.setText("Couldn't initialize the OpenGL context.");
        msgBox.exec();
        return ;
    }

    QSize screenSize = tofMean_graph->screen()->size();
   qDebug()<<"screen size  = "<<screenSize.width() <<"   "<<screenSize.height()<<endl;


//    QSize screenSize = ui->tofMean_widget->size();
    tofMean_container->setMinimumSize(QSize(screenSize.width()/2.5 , screenSize.height()/3.2));
//     container->setMinimumSize(screenSize);
    tofMean_container->setMaximumSize(screenSize);
//    container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
//    container->setFocusPolicy(Qt::StrongFocus);

    //! [1]
    QWidget *widget = new QWidget;
//    QWidget *widget = ui->tofMean_widget;
    QHBoxLayout *hLayout = new QHBoxLayout(ui->tofMean_widget);
    QVBoxLayout *vLayout = new QVBoxLayout();
    hLayout->addWidget(tofMean_container, 1);
    hLayout->addLayout(vLayout);
    vLayout->setAlignment(Qt::AlignTop);
    //! [1]

    widget->setWindowTitle(QStringLiteral("Surface example"));

    QRadioButton *sqrtSinModelRB = new QRadioButton(widget);
    sqrtSinModelRB->setText(QStringLiteral("Sqrt111 && Sin"));
    sqrtSinModelRB->setChecked(false);



    QSlider *axisMinSliderX = new QSlider(Qt::Horizontal);
    axisMinSliderX->setMinimum(0);
    axisMinSliderX->setTickInterval(1);
    axisMinSliderX->setEnabled(true);
    QSlider *axisMaxSliderX = new QSlider(Qt::Horizontal);
    axisMaxSliderX->setMinimum(1);
    axisMaxSliderX->setTickInterval(1);
    axisMaxSliderX->setEnabled(true);
    QSlider *axisMinSliderZ = new QSlider(Qt::Horizontal);
    axisMinSliderZ->setMinimum(0);
    axisMinSliderZ->setTickInterval(1);
    axisMinSliderZ->setEnabled(true);
    QSlider *axisMaxSliderZ = new QSlider(Qt::Horizontal);
    axisMaxSliderZ->setMinimum(1);
    axisMaxSliderZ->setTickInterval(1);
    axisMaxSliderZ->setEnabled(true);

//    widget->show();

    tofMean_modifier = new SurfaceGraph(tofMean_graph);

    QObject::connect(sqrtSinModelRB, &QRadioButton::toggled,
                     tofMean_modifier, &SurfaceGraph::enableSqrtSinModel);

    QObject::connect(this, SIGNAL(sendTofMean_signal(QStringList)),
                     tofMean_modifier, SLOT(fillSqrtSinProxy_2(QStringList)));

    QObject::connect(this, SIGNAL(sendTofMeanclear_signal()),
                     tofMean_modifier, SLOT(clearSlot()));


    tofMean_modifier->setAxisMinSliderX(axisMinSliderX);
    tofMean_modifier->setAxisMaxSliderX(axisMaxSliderX);
    tofMean_modifier->setAxisMinSliderZ(axisMinSliderZ);
    tofMean_modifier->setAxisMaxSliderZ(axisMaxSliderZ);
    sqrtSinModelRB->setChecked(true);
//    modeItemRB->setChecked(true);
//    themeList->setCurrentIndex(2);

    tofMean_modifier->setAxisYRange(0,700,QStringLiteral("均值"));





}

//初始化TOF标准差界面
void statisticsDialog::initTofStd_slot()
{

    tofStd_graph = new Q3DSurface();
    tofStd_container = QWidget::createWindowContainer(tofStd_graph);
    //! [0]

    if (!tofStd_graph->hasContext()) {
        QMessageBox msgBox;
        msgBox.setText("Couldn't initialize the OpenGL context.");
        msgBox.exec();
        return ;
    }

    QSize screenSize = tofStd_graph->screen()->size();
   qDebug()<<"screen size  = "<<screenSize.width() <<"   "<<screenSize.height()<<endl;


//    QSize screenSize = ui->tofMean_widget->size();
    tofStd_container->setMinimumSize(QSize(screenSize.width()/2.5 , screenSize.height()/3.2));
//     container->setMinimumSize(screenSize);
    tofStd_container->setMaximumSize(screenSize);
//    container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
//    container->setFocusPolicy(Qt::StrongFocus);

    //! [1]
    QWidget *widget = new QWidget;
//    QWidget *widget = ui->tofMean_widget;
    QHBoxLayout *hLayout = new QHBoxLayout(ui->tofStd_widget);
    QVBoxLayout *vLayout = new QVBoxLayout();
    hLayout->addWidget(tofStd_container, 1);
    hLayout->addLayout(vLayout);
    vLayout->setAlignment(Qt::AlignTop);
    //! [1]

    widget->setWindowTitle(QStringLiteral("Surface example"));

    QRadioButton *sqrtSinModelRB = new QRadioButton(widget);
    sqrtSinModelRB->setText(QStringLiteral("Sqrt111 && Sin"));
    sqrtSinModelRB->setChecked(false);



    QSlider *axisMinSliderX = new QSlider(Qt::Horizontal);
    axisMinSliderX->setMinimum(0);
    axisMinSliderX->setTickInterval(1);
    axisMinSliderX->setEnabled(true);
    QSlider *axisMaxSliderX = new QSlider(Qt::Horizontal);
    axisMaxSliderX->setMinimum(1);
    axisMaxSliderX->setTickInterval(1);
    axisMaxSliderX->setEnabled(true);
    QSlider *axisMinSliderZ = new QSlider(Qt::Horizontal);
    axisMinSliderZ->setMinimum(0);
    axisMinSliderZ->setTickInterval(1);
    axisMinSliderZ->setEnabled(true);
    QSlider *axisMaxSliderZ = new QSlider(Qt::Horizontal);
    axisMaxSliderZ->setMinimum(1);
    axisMaxSliderZ->setTickInterval(1);
    axisMaxSliderZ->setEnabled(true);

//    widget->show();

    tofStd_modifier = new SurfaceGraph(tofStd_graph);

    QObject::connect(sqrtSinModelRB, &QRadioButton::toggled,
                     tofStd_modifier, &SurfaceGraph::enableSqrtSinModel);
    QObject::connect(this, SIGNAL(sendTofStd_signal(QStringList)),
                     tofStd_modifier, SLOT(fillSqrtSinProxy_2(QStringList)));
    QObject::connect(this, SIGNAL(sendTofStdclear_signal()),
                     tofStd_modifier, SLOT(clearSlot()));


    tofStd_modifier->setAxisMinSliderX(axisMinSliderX);
    tofStd_modifier->setAxisMaxSliderX(axisMaxSliderX);
    tofStd_modifier->setAxisMinSliderZ(axisMinSliderZ);
    tofStd_modifier->setAxisMaxSliderZ(axisMaxSliderZ);

    sqrtSinModelRB->setChecked(true);
//    modeItemRB->setChecked(true);

    tofStd_modifier->setAxisYRange(0,4,QStringLiteral("标准差"));

}

//初始化PEAK均值界面
void statisticsDialog::initPeakMean_slot()
{
    peakMean_graph = new Q3DSurface();
    peakMean_container = QWidget::createWindowContainer(peakMean_graph);
    //! [0]

    if (!peakMean_graph->hasContext()) {
        QMessageBox msgBox;
        msgBox.setText("Couldn't initialize the OpenGL context.");
        msgBox.exec();
        return ;
    }

    QSize screenSize = peakMean_graph->screen()->size();
   qDebug()<<"screen size  = "<<screenSize.width() <<"   "<<screenSize.height()<<endl;


//    QSize screenSize = ui->tofMean_widget->size();
    peakMean_container->setMinimumSize(QSize(screenSize.width()/2.5 , screenSize.height()/3.2));
//     container->setMinimumSize(screenSize);
    peakMean_container->setMaximumSize(screenSize);
//    container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
//    container->setFocusPolicy(Qt::StrongFocus);

    //! [1]
    QWidget *widget = new QWidget;
//    QWidget *widget = ui->tofMean_widget;
    QHBoxLayout *hLayout = new QHBoxLayout(ui->peakMean_widget);
    QVBoxLayout *vLayout = new QVBoxLayout();
    hLayout->addWidget(peakMean_container, 1);
    hLayout->addLayout(vLayout);
    vLayout->setAlignment(Qt::AlignTop);
    //! [1]

    widget->setWindowTitle(QStringLiteral("Surface example"));

    QRadioButton *sqrtSinModelRB = new QRadioButton(widget);
    sqrtSinModelRB->setText(QStringLiteral("Sqrt111 && Sin"));
    sqrtSinModelRB->setChecked(false);



    QSlider *axisMinSliderX = new QSlider(Qt::Horizontal);
    axisMinSliderX->setMinimum(0);
    axisMinSliderX->setTickInterval(1);
    axisMinSliderX->setEnabled(true);
    QSlider *axisMaxSliderX = new QSlider(Qt::Horizontal);
    axisMaxSliderX->setMinimum(1);
    axisMaxSliderX->setTickInterval(1);
    axisMaxSliderX->setEnabled(true);
    QSlider *axisMinSliderZ = new QSlider(Qt::Horizontal);
    axisMinSliderZ->setMinimum(0);
    axisMinSliderZ->setTickInterval(1);
    axisMinSliderZ->setEnabled(true);
    QSlider *axisMaxSliderZ = new QSlider(Qt::Horizontal);
    axisMaxSliderZ->setMinimum(1);
    axisMaxSliderZ->setTickInterval(1);
    axisMaxSliderZ->setEnabled(true);

//    widget->show();

    peakMean_modifier = new SurfaceGraph(peakMean_graph);

    QObject::connect(sqrtSinModelRB, &QRadioButton::toggled,
                     peakMean_modifier, &SurfaceGraph::enableSqrtSinModel);

    QObject::connect(this, SIGNAL(sendPeakMean_signal(QStringList)),
                     peakMean_modifier, SLOT(fillSqrtSinProxy_2(QStringList)));
    QObject::connect(this, SIGNAL(sendPeakMeanclear_signal()),
                     peakMean_modifier, SLOT(clearSlot()));


    peakMean_modifier->setAxisMinSliderX(axisMinSliderX);
    peakMean_modifier->setAxisMaxSliderX(axisMaxSliderX);
    peakMean_modifier->setAxisMinSliderZ(axisMinSliderZ);
    peakMean_modifier->setAxisMaxSliderZ(axisMaxSliderZ);
    sqrtSinModelRB->setChecked(true);
//    modeItemRB->setChecked(true);
//    themeList->setCurrentIndex(2);

    peakMean_modifier->setAxisYRange(0,700,QStringLiteral("均值"));
}

//初始化PEAK标准差界面
void statisticsDialog::initPeakStd_slot()
{
    peakStd_graph = new Q3DSurface();
    peakStd_container = QWidget::createWindowContainer(peakStd_graph);
    //! [0]

    if (!peakStd_graph->hasContext()) {
        QMessageBox msgBox;
        msgBox.setText("Couldn't initialize the OpenGL context.");
        msgBox.exec();
        return ;
    }

    QSize screenSize = peakStd_graph->screen()->size();
   qDebug()<<"screen size  = "<<screenSize.width() <<"   "<<screenSize.height()<<endl;


//    QSize screenSize = ui->tofMean_widget->size();
    tofStd_container->setMinimumSize(QSize(screenSize.width()/2.5 , screenSize.height()/3.2));
//     container->setMinimumSize(screenSize);
    tofStd_container->setMaximumSize(screenSize);
//    container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
//    container->setFocusPolicy(Qt::StrongFocus);

    //! [1]
    QWidget *widget = new QWidget;
//    QWidget *widget = ui->tofMean_widget;
    QHBoxLayout *hLayout = new QHBoxLayout(ui->peakStd_widget);
    QVBoxLayout *vLayout = new QVBoxLayout();
    hLayout->addWidget(peakStd_container, 1);
    hLayout->addLayout(vLayout);
    vLayout->setAlignment(Qt::AlignTop);
    //! [1]

    widget->setWindowTitle(QStringLiteral("Surface example"));

    QRadioButton *sqrtSinModelRB = new QRadioButton(widget);
    sqrtSinModelRB->setText(QStringLiteral("Sqrt111 && Sin"));
    sqrtSinModelRB->setChecked(false);



    QSlider *axisMinSliderX = new QSlider(Qt::Horizontal);
    axisMinSliderX->setMinimum(0);
    axisMinSliderX->setTickInterval(1);
    axisMinSliderX->setEnabled(true);
    QSlider *axisMaxSliderX = new QSlider(Qt::Horizontal);
    axisMaxSliderX->setMinimum(1);
    axisMaxSliderX->setTickInterval(1);
    axisMaxSliderX->setEnabled(true);
    QSlider *axisMinSliderZ = new QSlider(Qt::Horizontal);
    axisMinSliderZ->setMinimum(0);
    axisMinSliderZ->setTickInterval(1);
    axisMinSliderZ->setEnabled(true);
    QSlider *axisMaxSliderZ = new QSlider(Qt::Horizontal);
    axisMaxSliderZ->setMinimum(1);
    axisMaxSliderZ->setTickInterval(1);
    axisMaxSliderZ->setEnabled(true);

//    widget->show();

    peakStd_modifier = new SurfaceGraph(peakStd_graph);

    QObject::connect(sqrtSinModelRB, &QRadioButton::toggled,
                     peakStd_modifier, &SurfaceGraph::enableSqrtSinModel);
    QObject::connect(this, SIGNAL(sendPeakStd_signal(QStringList)),
                     peakStd_modifier, SLOT(fillSqrtSinProxy_2(QStringList)));
    QObject::connect(this, SIGNAL(sendPeakStdclear_signal()),
                     peakStd_modifier, SLOT(clearSlot()));


    peakStd_modifier->setAxisMinSliderX(axisMinSliderX);
    peakStd_modifier->setAxisMaxSliderX(axisMaxSliderX);
    peakStd_modifier->setAxisMinSliderZ(axisMinSliderZ);
    peakStd_modifier->setAxisMaxSliderZ(axisMaxSliderZ);

    sqrtSinModelRB->setChecked(true);
//    modeItemRB->setChecked(true);

    peakStd_modifier->setAxisYRange(0,4,QStringLiteral("标准差"));
}



statisticsDialog::~statisticsDialog()
{
    delete ui;
}


//开始统计的槽函数
void statisticsDialog::on_start_pushButton_clicked()
{
    int frameNumber = ui->frame_lineEdit->text().toInt();
    emit alterStatisticFrameNum_signal(frameNumber);

//    initTofMean_slot();
    isRun =true;
    startStop_signal(1);
}



void statisticsDialog::on_start_pushButton_2_clicked()
{
//    updateTimer.stop();
    startStop_signal(0);
}

//接收统计均值和方差的槽函数
void statisticsDialog::statistic_MeanStdSlot(QStringList tofMeanlist, QStringList tofStdlist, QStringList peakMeanlist, QStringList peakStdlist)
{
    qDebug()<<"tofMean's length  = "<<tofMeanlist.length()<<endl;

    if(isRun)
    {
        emit sendTofMean_signal(tofMeanlist);

        emit sendTofStd_signal(tofStdlist);

        emit sendPeakMean_signal(peakMeanlist);

        emit sendPeakStd_signal(peakStdlist);
    }




//    ui->tofMean_textEdit->setText(tofMean);
//    ui->tofStd_textEdit->setText(tofStd);

//    ui->peakMean_textEdit->setText(peakMean);
//    ui->peakStd_textEdit->setText(peakStd);
}


void statisticsDialog::closeEvent(QCloseEvent *event)
{
    qDebug()<<" QCloseEvent "<<endl;
    isRun = false;
     startStop_signal(0);


     emit sendTofMeanclear_signal();

     emit sendTofStdclear_signal();

     emit sendPeakMeanclear_signal();

     emit sendPeakStdclear_signal();


    this->hide();
    event->ignore();
    return;
}
