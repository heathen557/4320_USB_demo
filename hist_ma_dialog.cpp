#include "hist_ma_dialog.h"
#include "ui_hist_ma_dialog.h"

Hist_MA_Dialog::Hist_MA_Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Hist_MA_Dialog)
{
    ui->setupUi(this);
    binSize = 512;
    selectRow = 59;
    selectCol = 79;
    summary = 0;
    channelNum = 1;   //默认采用1个通道
    init_UI();
    init_histogram();
    connect(&filterWin_dia,SIGNAL(send_filterWindow_signal(QStringList)),this,SLOT(send_filterWindow_slot(QStringList)));
}


//!
//! \brief Hist_MA_Dialog::enableUI_trueOrfalse
//!使能控件的槽函数
void  Hist_MA_Dialog::enableUI_trueOrfalse(bool flag)
{
    ui->groupBox->setEnabled(flag);
    ui->binning_size_comboBox->setEnabled(flag);
    ui->binning_groupBox_1->setEnabled(flag);
    ui->binning_groupBox_2->setEnabled(flag);
    ui->binning_groupBox_3->setEnabled(flag);
    ui->binning_groupBox_4->setEnabled(flag);
}



//加载窗口
void Hist_MA_Dialog::on_filters_pushButton_clicked()
{
    summary= 0;
    filterWindow.clear();
    filterWin_dia.setModal(true);
    filterWin_dia.show();
}

//获取滤波窗口的值
void Hist_MA_Dialog::send_filterWindow_slot(QStringList valueList)
{
    foreach (QString str, valueList) {
        qDebug()<<str;
    }


    foreach (QString str, valueList) {
        filterWindow.append(str.toFloat());
        summary += str.toFloat();
    }

    int winSize = filterWindow.length();
    ui->windowSize_label->setText(QString::number(winSize));

    foreach (float f, filterWindow) {
        qDebug()<<f;
    }

}




//初始化界面控件相关
void Hist_MA_Dialog::init_UI()
{
    //单个pixel的行、列的初始化
    for(int row=0; row<120; row++)
    {
        ui->channel_1_row_comboBox->addItem(QString::number(row));
    }
    ui->channel_1_row_comboBox->setCurrentIndex(59);

    for(int col=0;col<160;col++)
    {
        ui->channel_1_col_comboBox->addItem(QString::number(col));
    }
    ui->channel_1_col_comboBox->setCurrentIndex(79);


    //binning 的行、列的初始化
    for(int i=0; i<120;i++)
    {
        ui->channel1_bin_row_comboBox->addItem(QString::number(i));
        ui->channel2_bin_row_comboBox->addItem(QString::number(i));
        ui->channel3_bin_row_comboBox->addItem(QString::number(i));
        ui->channel4_bin_row_comboBox->addItem(QString::number(i));
    }
    for(int j=0; j<160; j++)
    {
        ui->channel1_bin_col_comboBox->addItem(QString::number(j));
        ui->channel2_bin_col_comboBox->addItem(QString::number(j));
        ui->channel3_bin_col_comboBox->addItem(QString::number(j));
        ui->channel4_bin_col_comboBox->addItem(QString::number(j));
    }

}


//初始化=直方图界面
void Hist_MA_Dialog::init_histogram()
{
    for(int i=0;i<4;i++)
    {
        HistorgramTicks[i].resize(binSize);
        HistorgramLabels[i].resize(binSize);
    }

    //对四个直方图的标签 进行初始化
    for(int i=0;i<4;i++)
    {
        for(int j=0; j<binSize; j++)
        {
            HistorgramTicks[i][j] = j;
            HistorgramLabels[i][j] = "";
            if(0 == j%40)     //相隔4个数据打一个标签
            {
                HistorgramLabels[i][j] = QString::number(j);
            }
        }
    }


    //   初始化直方图1
    regen[0] = new QCPBars(ui->MA_1_widget->xAxis,ui->MA_1_widget->yAxis);
    regen[0]->setAntialiased(false);
    regen[0]->setStackingGap(2);
    regen[0]->setName("Histogram");
    regen[0]->setPen(QPen(QColor(0, 168, 140).lighter(130)));
    regen[0]->setBrush(QColor(0, 168, 140));
    ui->MA_1_widget->xAxis->setTickLabelRotation(60);//设置标签角度旋转
    ui->MA_1_widget->xAxis->setSubTicks(false);//设置是否显示子标签
    ui->MA_1_widget->xAxis->setTickLength(0, 4);
    ui->MA_1_widget->xAxis->setRange(0, 512);     //设置x轴区间
    ui->MA_1_widget->xAxis->setBasePen(QPen(Qt::black));
    ui->MA_1_widget->xAxis->setTickPen(QPen(Qt::black));
    ui->MA_1_widget->xAxis->grid()->setVisible(true);//设置网格是否显示
    ui->MA_1_widget->xAxis->grid()->setPen(QPen(QColor(130, 130, 130), 0, Qt::DotLine));
    ui->MA_1_widget->xAxis->setTickLabelColor(Qt::black);//设置标记标签颜色
    ui->MA_1_widget->xAxis->setLabelColor(Qt::black);
    // prepare y axis: //设置y轴
    ui->MA_1_widget->yAxis->setRange(0, 100);
    ui->MA_1_widget->yAxis->setPadding(5); // a bit more space to the left border 设置左边留空间
    //    ui->Histogram_widget->yAxis->setLabel("Power Consumption in\nKilowatts per Capita (2007)");
    ui->MA_1_widget->yAxis->setBasePen(QPen(Qt::black));
    ui->MA_1_widget->yAxis->setTickPen(QPen(Qt::black));
    ui->MA_1_widget->yAxis->setSubTickPen(QPen(Qt::black));//设置SubTick颜色，SubTick指的是轴上的
    //刻度线
    ui->MA_1_widget->yAxis->grid()->setSubGridVisible(true);
    ui->MA_1_widget->yAxis->setTickLabelColor(Qt::black);//设置标记标签颜色（y轴标记标签）
    ui->MA_1_widget->yAxis->setLabelColor(Qt::black);//设置标签颜色（y轴右边标签）
    ui->MA_1_widget->yAxis->grid()->setPen(QPen(QColor(130, 130, 130), 0, Qt::SolidLine));
    ui->MA_1_widget->yAxis->grid()->setSubGridPen(QPen(QColor(130, 130, 130), 0, Qt::DotLine));
    // setup legend: 设置标签
    //    ui->Histogram_widget->legend->setVisible(true);
    ui->MA_1_widget->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop|Qt::AlignLeft);
    ui->MA_1_widget->legend->setBrush(QColor(255, 255, 255, 100));
    ui->MA_1_widget->legend->setBorderPen(Qt::NoPen);
    QFont legendFont = font();
    legendFont.setPointSize(7);
    ui->MA_1_widget->legend->setFont(legendFont);
    ui->MA_1_widget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);//设置 可拖动，可放大缩
    //初始化MA曲线显示相关
    ui->MA_1_widget->legend->setVisible(true);
    ui->MA_1_widget->addGraph();
    ui->MA_1_widget->graph(0)->setName(QStringLiteral("MA"));
    ui->MA_1_widget->graph(0)->setPen(QPen(QColor(255, 0, 0).lighter(130)));




    //   初始化直方图2
    regen[1] = new QCPBars(ui->MA_2_widget->xAxis,ui->MA_2_widget->yAxis);
    regen[1]->setAntialiased(false);
    regen[1]->setStackingGap(2);
    regen[1]->setName("Histogram");
    regen[1]->setPen(QPen(QColor(0, 168, 140).lighter(130)));
    regen[1]->setBrush(QColor(0, 168, 140));
    ui->MA_2_widget->xAxis->setTickLabelRotation(60);//设置标签角度旋转
    ui->MA_2_widget->xAxis->setSubTicks(false);//设置是否显示子标签
    ui->MA_2_widget->xAxis->setTickLength(0, 4);
    ui->MA_2_widget->xAxis->setRange(0, 512);     //设置x轴区间
    ui->MA_2_widget->xAxis->setBasePen(QPen(Qt::black));
    ui->MA_2_widget->xAxis->setTickPen(QPen(Qt::black));
    ui->MA_2_widget->xAxis->grid()->setVisible(true);//设置网格是否显示
    ui->MA_2_widget->xAxis->grid()->setPen(QPen(QColor(130, 130, 130), 0, Qt::DotLine));
    ui->MA_2_widget->xAxis->setTickLabelColor(Qt::black);//设置标记标签颜色
    ui->MA_2_widget->xAxis->setLabelColor(Qt::black);
    // prepare y axis: //设置y轴
    ui->MA_2_widget->yAxis->setRange(0, 100);
    ui->MA_2_widget->yAxis->setPadding(5); // a bit more space to the left border 设置左边留空间
    //    ui->Histogram_widget->yAxis->setLabel("Power Consumption in\nKilowatts per Capita (2007)");
    ui->MA_2_widget->yAxis->setBasePen(QPen(Qt::black));
    ui->MA_2_widget->yAxis->setTickPen(QPen(Qt::black));
    ui->MA_2_widget->yAxis->setSubTickPen(QPen(Qt::black));//设置SubTick颜色，SubTick指的是轴上的
    //刻度线
    ui->MA_2_widget->yAxis->grid()->setSubGridVisible(true);
    ui->MA_2_widget->yAxis->setTickLabelColor(Qt::black);//设置标记标签颜色（y轴标记标签）
    ui->MA_2_widget->yAxis->setLabelColor(Qt::black);//设置标签颜色（y轴右边标签）
    ui->MA_2_widget->yAxis->grid()->setPen(QPen(QColor(130, 130, 130), 0, Qt::SolidLine));
    ui->MA_2_widget->yAxis->grid()->setSubGridPen(QPen(QColor(130, 130, 130), 0, Qt::DotLine));
    // setup legend: 设置标签
    //    ui->Histogram_widget->legend->setVisible(true);
    ui->MA_2_widget->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop|Qt::AlignLeft);
    ui->MA_2_widget->legend->setBrush(QColor(255, 255, 255, 100));
    ui->MA_2_widget->legend->setBorderPen(Qt::NoPen);
    ui->MA_2_widget->legend->setFont(legendFont);
    ui->MA_2_widget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);//设置 可拖动，可放大缩
    //初始化MA曲线显示相关
    ui->MA_2_widget->legend->setVisible(true);
    ui->MA_2_widget->addGraph();
    ui->MA_2_widget->graph(0)->setName(QStringLiteral("MA"));
    ui->MA_2_widget->graph(0)->setPen(QPen(QColor(255, 0, 0).lighter(130)));


    //   初始化直方图3
    regen[2] = new QCPBars(ui->MA_3_widget->xAxis,ui->MA_3_widget->yAxis);
    regen[2]->setAntialiased(false);
    regen[2]->setStackingGap(2);
    regen[2]->setName("Histogram");
    regen[2]->setPen(QPen(QColor(0, 168, 140).lighter(130)));
    regen[2]->setBrush(QColor(0, 168, 140));
    ui->MA_3_widget->xAxis->setTickLabelRotation(60);//设置标签角度旋转
    ui->MA_3_widget->xAxis->setSubTicks(false);//设置是否显示子标签
    ui->MA_3_widget->xAxis->setTickLength(0, 4);
    ui->MA_3_widget->xAxis->setRange(0, 512);     //设置x轴区间
    ui->MA_3_widget->xAxis->setBasePen(QPen(Qt::black));
    ui->MA_3_widget->xAxis->setTickPen(QPen(Qt::black));
    ui->MA_3_widget->xAxis->grid()->setVisible(true);//设置网格是否显示
    ui->MA_3_widget->xAxis->grid()->setPen(QPen(QColor(130, 130, 130), 0, Qt::DotLine));
    ui->MA_3_widget->xAxis->setTickLabelColor(Qt::black);//设置标记标签颜色
    ui->MA_3_widget->xAxis->setLabelColor(Qt::black);
    // prepare y axis: //设置y轴
    ui->MA_3_widget->yAxis->setRange(0, 100);
    ui->MA_3_widget->yAxis->setPadding(5); // a bit more space to the left border 设置左边留空间
    //    ui->Histogram_widget->yAxis->setLabel("Power Consumption in\nKilowatts per Capita (2007)");
    ui->MA_3_widget->yAxis->setBasePen(QPen(Qt::black));
    ui->MA_3_widget->yAxis->setTickPen(QPen(Qt::black));
    ui->MA_3_widget->yAxis->setSubTickPen(QPen(Qt::black));//设置SubTick颜色，SubTick指的是轴上的
    //刻度线
    ui->MA_3_widget->yAxis->grid()->setSubGridVisible(true);
    ui->MA_3_widget->yAxis->setTickLabelColor(Qt::black);//设置标记标签颜色（y轴标记标签）
    ui->MA_3_widget->yAxis->setLabelColor(Qt::black);//设置标签颜色（y轴右边标签）
    ui->MA_3_widget->yAxis->grid()->setPen(QPen(QColor(130, 130, 130), 0, Qt::SolidLine));
    ui->MA_3_widget->yAxis->grid()->setSubGridPen(QPen(QColor(130, 130, 130), 0, Qt::DotLine));
    // setup legend: 设置标签
    //    ui->Histogram_widget->legend->setVisible(true);
    ui->MA_3_widget->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop|Qt::AlignLeft);
    ui->MA_3_widget->legend->setBrush(QColor(255, 255, 255, 100));
    ui->MA_3_widget->legend->setBorderPen(Qt::NoPen);
    ui->MA_3_widget->legend->setFont(legendFont);
    ui->MA_3_widget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);//设置 可拖动，可放大缩
    //初始化MA曲线显示相关
    ui->MA_3_widget->legend->setVisible(true);
    ui->MA_3_widget->addGraph();
    ui->MA_3_widget->graph(0)->setName(QStringLiteral("MA"));
    ui->MA_3_widget->graph(0)->setPen(QPen(QColor(255, 0, 0).lighter(130)));



    //   初始化直方图4
    regen[3] = new QCPBars(ui->MA_4_widget->xAxis,ui->MA_4_widget->yAxis);
    regen[3]->setAntialiased(false);
    regen[3]->setStackingGap(2);
    regen[3]->setName("Histogram");
    regen[3]->setPen(QPen(QColor(0, 168, 140).lighter(130)));
    regen[3]->setBrush(QColor(0, 168, 140));
    ui->MA_4_widget->xAxis->setTickLabelRotation(60);//设置标签角度旋转
    ui->MA_4_widget->xAxis->setSubTicks(false);//设置是否显示子标签
    ui->MA_4_widget->xAxis->setTickLength(0, 4);
    ui->MA_4_widget->xAxis->setRange(0, 512);     //设置x轴区间
    ui->MA_4_widget->xAxis->setBasePen(QPen(Qt::black));
    ui->MA_4_widget->xAxis->setTickPen(QPen(Qt::black));
    ui->MA_4_widget->xAxis->grid()->setVisible(true);//设置网格是否显示
    ui->MA_4_widget->xAxis->grid()->setPen(QPen(QColor(130, 130, 130), 0, Qt::DotLine));
    ui->MA_4_widget->xAxis->setTickLabelColor(Qt::black);//设置标记标签颜色
    ui->MA_4_widget->xAxis->setLabelColor(Qt::black);
    // prepare y axis: //设置y轴
    ui->MA_4_widget->yAxis->setRange(0, 100);
    ui->MA_4_widget->yAxis->setPadding(5); // a bit more space to the left border 设置左边留空间
    //    ui->Histogram_widget->yAxis->setLabel("Power Consumption in\nKilowatts per Capita (2007)");
    ui->MA_4_widget->yAxis->setBasePen(QPen(Qt::black));
    ui->MA_4_widget->yAxis->setTickPen(QPen(Qt::black));
    ui->MA_4_widget->yAxis->setSubTickPen(QPen(Qt::black));//设置SubTick颜色，SubTick指的是轴上的
    //刻度线
    ui->MA_4_widget->yAxis->grid()->setSubGridVisible(true);
    ui->MA_4_widget->yAxis->setTickLabelColor(Qt::black);//设置标记标签颜色（y轴标记标签）
    ui->MA_4_widget->yAxis->setLabelColor(Qt::black);//设置标签颜色（y轴右边标签）
    ui->MA_4_widget->yAxis->grid()->setPen(QPen(QColor(130, 130, 130), 0, Qt::SolidLine));
    ui->MA_4_widget->yAxis->grid()->setSubGridPen(QPen(QColor(130, 130, 130), 0, Qt::DotLine));
    // setup legend: 设置标签
    //    ui->Histogram_widget->legend->setVisible(true);
    ui->MA_4_widget->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop|Qt::AlignLeft);
    ui->MA_4_widget->legend->setBrush(QColor(255, 255, 255, 100));
    ui->MA_4_widget->legend->setBorderPen(Qt::NoPen);
    ui->MA_4_widget->legend->setFont(legendFont);
    ui->MA_4_widget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);//设置 可拖动，可放大缩
    //初始化MA曲线显示相关
    ui->MA_4_widget->legend->setVisible(true);
    ui->MA_4_widget->addGraph();
    ui->MA_4_widget->graph(0)->setName(QStringLiteral("MA"));
    ui->MA_4_widget->graph(0)->setPen(QPen(QColor(255, 0, 0).lighter(130)));



}


//初始化直方图的标签
void Hist_MA_Dialog::init_histogramLabel()
{
    for(int i=0;i<4;i++)
    {
        HistorgramTicks[i].resize(binSize);
        HistorgramLabels[i].resize(binSize);
    }

    //对四个直方图的标签 进行初始化
    for(int i=0;i<4;i++)
    {
        for(int j=0; j<binSize; j++)
        {
            HistorgramTicks[i][j] = j;
            HistorgramLabels[i][j] = "";
            if(0 == j%40)     //相隔4个数据打一个标签
            {
                HistorgramLabels[i][j] = QString::number(j);
            }
        }
    }
}


//!
//! \brief Hist_MA_Dialog::on_channel_1_row_comboBox_activated
//! \param index
//!改变channel_1 pixel 的row
void Hist_MA_Dialog::on_channel_1_row_comboBox_activated(int index)
{
    selectRow = index;
}

//!
//! \brief Hist_MA_Dialog::on_channel_1_col_comboBox_activated
//! \param index
//!改变channel_1 pixel 的col
void Hist_MA_Dialog::on_channel_1_col_comboBox_activated(int index)
{
    selectCol = index;
}


Hist_MA_Dialog::~Hist_MA_Dialog()
{
    delete ui;
}

//开始pixel_historgram
void Hist_MA_Dialog::on_start_pushButton_clicked()
{
    int exposure_num = ui->exposure_num_lineEdit->text().toInt();
    int integration_num = ui->IntegrationTime_lineEdit->text().toInt();
    if("start" == ui->start_pushButton->text())
    {
        emit start_RowDatahistogram_signal(exposure_num,integration_num,selectRow,selectCol,true);
        emit start_receRowDataMA_signal();
        ui->start_pushButton->setText("stop");
    }else
    {
        emit start_RowDatahistogram_signal(exposure_num,integration_num,selectRow,selectCol,false);
        ui->start_pushButton->setText("start");
    }
}






//!
//! \brief Hist_MA_Dialog::currentFrame_slot
//! \param frame
//!获取到当前的帧数
void Hist_MA_Dialog::currentFrame_slot(int frame)
{
    int exposure_num = ui->exposure_num_lineEdit->text().toInt();
    int current_exposureNum = frame * exposure_num;
    ui->currentExposureNum_label->setText(QString::number(current_exposureNum));
}

//binning 的选择binning 大小（2x2  或者4x4）后行列像素同步改变
void Hist_MA_Dialog::on_binning_size_comboBox_activated(int index)
{
    qDebug()<<"index = "<<index;
    ui->channel1_bin_col_comboBox->clear();
    ui->channel1_bin_row_comboBox->clear();

    ui->channel1_bin_row_comboBox->clear();
    ui->channel2_bin_row_comboBox->clear();
    ui->channel3_bin_row_comboBox->clear();
    ui->channel4_bin_row_comboBox->clear();
    ui->channel1_bin_col_comboBox->clear();
    ui->channel2_bin_col_comboBox->clear();
    ui->channel3_bin_col_comboBox->clear();
    ui->channel4_bin_col_comboBox->clear();

    if(0 == index)
    {
        for(int i=0; i<120;i++)
        {
            ui->channel1_bin_row_comboBox->addItem(QString::number(i));
            ui->channel2_bin_row_comboBox->addItem(QString::number(i));
            ui->channel3_bin_row_comboBox->addItem(QString::number(i));
            ui->channel4_bin_row_comboBox->addItem(QString::number(i));
        }
        for(int j=0; j<160; j++)
        {
            ui->channel1_bin_col_comboBox->addItem(QString::number(j));
            ui->channel2_bin_col_comboBox->addItem(QString::number(j));
            ui->channel3_bin_col_comboBox->addItem(QString::number(j));
            ui->channel4_bin_col_comboBox->addItem(QString::number(j));
        }
    }else if(1 == index)
    {
        for(int i=0; i<120;i+=2)
        {
            ui->channel1_bin_row_comboBox->addItem(QString::number(i));
            ui->channel2_bin_row_comboBox->addItem(QString::number(i));
            ui->channel3_bin_row_comboBox->addItem(QString::number(i));
            ui->channel4_bin_row_comboBox->addItem(QString::number(i));
        }
        for(int j=0; j<160; j+=2)
        {
            ui->channel1_bin_col_comboBox->addItem(QString::number(j));
            ui->channel2_bin_col_comboBox->addItem(QString::number(j));
            ui->channel3_bin_col_comboBox->addItem(QString::number(j));
            ui->channel4_bin_col_comboBox->addItem(QString::number(j));
        }
    }else if(2 == index)
    {
        for(int i=0; i<120;i+=4)
        {
            ui->channel1_bin_row_comboBox->addItem(QString::number(i));
            ui->channel2_bin_row_comboBox->addItem(QString::number(i));
            ui->channel3_bin_row_comboBox->addItem(QString::number(i));
            ui->channel4_bin_row_comboBox->addItem(QString::number(i));

        }
        for(int j=0;j<160;j+=4)
        {
            ui->channel1_bin_col_comboBox->addItem(QString::number(j));
            ui->channel2_bin_col_comboBox->addItem(QString::number(j));
            ui->channel3_bin_col_comboBox->addItem(QString::number(j));
            ui->channel4_bin_col_comboBox->addItem(QString::number(j));
        }
    }

    ui->channel1_bin_row_comboBox->setCurrentIndex(0);
    ui->channel2_bin_row_comboBox->setCurrentIndex(0);
    ui->channel3_bin_row_comboBox->setCurrentIndex(0);
    ui->channel4_bin_row_comboBox->setCurrentIndex(0);
    ui->channel1_bin_col_comboBox->setCurrentIndex(0);
    ui->channel2_bin_col_comboBox->setCurrentIndex(0);
    ui->channel3_bin_col_comboBox->setCurrentIndex(0);
    ui->channel4_bin_col_comboBox->setCurrentIndex(0);
}

//!
//! \brief Hist_MA_Dialog::on_bin_start_pushButton_clicked
//!  开始进行binning 直方图显示
void Hist_MA_Dialog::on_bin_start_pushButton_clicked()
{
    //    开启binning rawData测试  曝光次数，积分次数，初始行、初始列、 窗口大小（2、4）

    int exposure_num = ui->exposure_num_lineEdit->text().toInt();
    int integration_num = ui->IntegrationTime_lineEdit->text().toInt();
    int currentIndex = ui->binning_size_comboBox->currentIndex();
    int winSize = 1;

    //获取binning的大小  1x1 、2x2 、 4x4
    if(0==currentIndex)
    {
        winSize = 1;
    }else if(1 == currentIndex)
    {
        winSize = 2;
    }else if(2 == currentIndex)
    {
        winSize = 4;
    }

    //每个binning 的左上角起始点的坐标
    QVector<int> position_vec(8);   //默认会存储，4个channel的起始点的坐标
    position_vec[0] = ui->channel1_bin_row_comboBox->currentText().toInt();
    position_vec[1] = ui->channel1_bin_col_comboBox->currentText().toInt();

    position_vec[2] = ui->channel2_bin_row_comboBox->currentText().toInt();
    position_vec[3] = ui->channel2_bin_col_comboBox->currentText().toInt();

    position_vec[4] = ui->channel3_bin_row_comboBox->currentText().toInt();
    position_vec[5] = ui->channel3_bin_col_comboBox->currentText().toInt();

    position_vec[6] = ui->channel4_bin_row_comboBox->currentText().toInt();
    position_vec[7] = ui->channel4_bin_col_comboBox->currentText().toInt();



    if("start" == ui->bin_start_pushButton->text())
    {
        emit start_RowData_bin_histogram_signal(exposure_num,integration_num,position_vec,winSize,channelNum,true);
        emit start_receRowDataMA_signal();
        ui->bin_start_pushButton->setText("stop");
        enableUI_trueOrfalse(false);
    }else
    {
        emit start_RowData_bin_histogram_signal(exposure_num,integration_num,position_vec,winSize,channelNum,false);
        ui->bin_start_pushButton->setText("start");
        enableUI_trueOrfalse(true);
    }
}

//comboBox binSzie改变
void Hist_MA_Dialog::on_binSize_comboBox_activated(const QString &arg1)
{
    binSize = arg1.toInt();
    qDebug()<<"binSize = "<<binSize;
    init_histogramLabel();
}


//!
//! \brief Hist_MA_Dialog::on_channels_comboBox_currentIndexChanged
//! \param arg1  1、2、3、4
//!通道切换的槽函数
void Hist_MA_Dialog::on_channels_comboBox_currentIndexChanged(const QString &arg1)
{
    channelNum = arg1.toInt();
    qDebug()<<"channelNum = "<<channelNum;
    if(1==channelNum)
    {
        ui->binning_groupBox_1->setVisible(true);
        ui->binning_groupBox_2->setVisible(false);
        ui->binning_groupBox_3->setVisible(false);
        ui->binning_groupBox_4->setVisible(false);

        ui->MA_groupBox_1->setVisible(true);
        ui->MA_groupBox_2->setVisible(false);
        ui->MA_groupBox_3->setVisible(false);
        ui->MA_groupBox_4->setVisible(false);

    }else if(2 == channelNum)
    {

        ui->binning_groupBox_1->setVisible(true);
        ui->binning_groupBox_2->setVisible(true);
        ui->binning_groupBox_3->setVisible(false);
        ui->binning_groupBox_4->setVisible(false);

        ui->MA_groupBox_1->setVisible(true);
        ui->MA_groupBox_2->setVisible(true);
        ui->MA_groupBox_3->setVisible(false);
        ui->MA_groupBox_4->setVisible(false);
    }else if(4 == channelNum)
    {
        ui->binning_groupBox_1->setVisible(true);
        ui->binning_groupBox_2->setVisible(true);
        ui->binning_groupBox_3->setVisible(true);
        ui->binning_groupBox_4->setVisible(true);

        ui->MA_groupBox_1->setVisible(true);
        ui->MA_groupBox_2->setVisible(true);
        ui->MA_groupBox_3->setVisible(true);
        ui->MA_groupBox_4->setVisible(true);
    }
}



//显示直方图  、 同时计算并显示MA曲线
void Hist_MA_Dialog::toShowHistogram_channel1_slot(QVector<double> histogram_vec,int maxValue)
{
    QVector<double> showHistogram_vec;
    showHistogram_vec = histogram_vec;
    showHistogram_vec.resize(binSize);

    int maxY =0;
    for(int i=0;i<binSize; i++)
    {
        maxY = histogram_vec[i]>maxY ? histogram_vec[i]:maxY;
    }

    //显示直方图
    QSharedPointer<QCPAxisTickerText> textTicker(new QCPAxisTickerText);
    textTicker->setTicks(HistorgramTicks[0], HistorgramLabels[0]);
    ui->MA_1_widget->xAxis->setTicker(textTicker);
    ui->MA_1_widget->xAxis->setRange(0,binSize);
    ui->MA_1_widget->yAxis->setRange(0,maxY);
    regen[0]->setData(HistorgramTicks[0],showHistogram_vec);


    //计算MA曲线
    QVector<double> label_x(binSize);
    QVector<double> srcDataValue;
    QVector<double> MA_resValue(binSize);
    srcDataValue = showHistogram_vec;     //接下来需要对 srcDataValue 进行padding 操作
    int winSize = filterWindow.length();
    int padding = (winSize-1)/2;     //原始数据的前面和后面都要添加 padding的长度
    for(int i=0;i<padding;i++)
    {
        srcDataValue.insert(0,0);
    }
    srcDataValue.resize(binSize+winSize);

    qDebug()<<"srcDataValue.size="<<srcDataValue.length();

    //开始计算滤波
    float peakValue = 0;
    int tofIndex = 0;
    for(int index=0; index<binSize; index++)
    {
        label_x[index] = index;
        for(int len=0;len<winSize;len++)
        {
            MA_resValue[index] += srcDataValue[index+len]*filterWindow[len];
        }
        MA_resValue[index] = MA_resValue[index]/summary;
        if(MA_resValue[index]>peakValue)
        {
            peakValue = MA_resValue[index];
            tofIndex = index;
        }
    }

    ui->binningTof_label->setText(QString::number(tofIndex));
    ui->binningPeak_label->setText(QString::number(peakValue*summary));

    ui->MA_1_widget->graph(0)->setData(label_x,MA_resValue);
    ui->MA_1_widget->replot();

}

//!
//! \brief Hist_MA_Dialog::toShowHistogram_channel2_slot
//! \param histogram_vec
//! \param maxValue
//! 2 channel 的MA
void  Hist_MA_Dialog::toShowHistogram_channel2_slot(QVector<double> histogram_vec,int maxValue)
{
    QVector<double> showHistogram_vec;
    showHistogram_vec = histogram_vec;
    showHistogram_vec.resize(binSize);

    int maxY =0;
    for(int i=0;i<binSize; i++)
    {
        maxY = histogram_vec[i]>maxY ? histogram_vec[i]:maxY;
    }

    //显示直方图
    QSharedPointer<QCPAxisTickerText> textTicker(new QCPAxisTickerText);
    textTicker->setTicks(HistorgramTicks[1], HistorgramLabels[1]);
    ui->MA_2_widget->xAxis->setTicker(textTicker);
    ui->MA_2_widget->xAxis->setRange(0,binSize);
    ui->MA_2_widget->yAxis->setRange(0,maxY);
    regen[1]->setData(HistorgramTicks[0],showHistogram_vec);


    //计算MA曲线
    QVector<double> label_x(binSize);
    QVector<double> srcDataValue;
    QVector<double> MA_resValue(binSize);
    srcDataValue = showHistogram_vec;     //接下来需要对 srcDataValue 进行padding 操作
    int winSize = filterWindow.length();
    int padding = (winSize-1)/2;     //原始数据的前面和后面都要添加 padding的长度
    for(int i=0;i<padding;i++)
    {
        srcDataValue.insert(0,0);
    }
    srcDataValue.resize(binSize+winSize);

    qDebug()<<"srcDataValue.size="<<srcDataValue.length();

    //开始计算滤波
    float peakValue = 0;
    int tofIndex = 0;
    for(int index=0; index<binSize; index++)
    {
        label_x[index] = index;
        for(int len=0;len<winSize;len++)
        {
            MA_resValue[index] += srcDataValue[index+len]*filterWindow[len];
        }
        MA_resValue[index] = MA_resValue[index]/summary;
        if(MA_resValue[index]>peakValue)
        {
            peakValue = MA_resValue[index];
            tofIndex = index;
        }
    }

    ui->binningTof_label_2->setText(QString::number(tofIndex));
    ui->binningPeak_label_2->setText(QString::number(peakValue*summary));

    ui->MA_2_widget->graph(0)->setData(label_x,MA_resValue);
    ui->MA_2_widget->replot();
}

//!
//! \brief Hist_MA_Dialog::toShowHistogram_channel3_slot
//! \param histogram_vec
//! \param maxValue
//! 3channel 的 MA
void Hist_MA_Dialog::toShowHistogram_channel3_slot(QVector<double> histogram_vec,int maxValue)
{
    QVector<double> showHistogram_vec;
    showHistogram_vec = histogram_vec;
    showHistogram_vec.resize(binSize);

    int maxY =0;
    for(int i=0;i<binSize; i++)
    {
        maxY = histogram_vec[i]>maxY ? histogram_vec[i]:maxY;
    }

    //显示直方图
    QSharedPointer<QCPAxisTickerText> textTicker(new QCPAxisTickerText);
    textTicker->setTicks(HistorgramTicks[2], HistorgramLabels[2]);
    ui->MA_3_widget->xAxis->setTicker(textTicker);
    ui->MA_3_widget->xAxis->setRange(0,binSize);
    ui->MA_3_widget->yAxis->setRange(0,maxY);
    regen[2]->setData(HistorgramTicks[0],showHistogram_vec);


    //计算MA曲线
    QVector<double> label_x(binSize);
    QVector<double> srcDataValue;
    QVector<double> MA_resValue(binSize);
    srcDataValue = showHistogram_vec;     //接下来需要对 srcDataValue 进行padding 操作
    int winSize = filterWindow.length();
    int padding = (winSize-1)/2;     //原始数据的前面和后面都要添加 padding的长度
    for(int i=0;i<padding;i++)
    {
        srcDataValue.insert(0,0);
    }
    srcDataValue.resize(binSize+winSize);

    qDebug()<<"srcDataValue.size="<<srcDataValue.length();

    //开始计算滤波
    float peakValue = 0;
    int tofIndex = 0;
    for(int index=0; index<binSize; index++)
    {
        label_x[index] = index;
        for(int len=0;len<winSize;len++)
        {
            MA_resValue[index] += srcDataValue[index+len]*filterWindow[len];
        }
        MA_resValue[index] = MA_resValue[index]/summary;
        if(MA_resValue[index]>peakValue)
        {
            peakValue = MA_resValue[index];
            tofIndex = index;
        }
    }

    ui->binningTof_label_3->setText(QString::number(tofIndex));
    ui->binningPeak_label_3->setText(QString::number(peakValue*summary));

    ui->MA_3_widget->graph(0)->setData(label_x,MA_resValue);
    ui->MA_3_widget->replot();
}

//!
//! \brief Hist_MA_Dialog::toShowHistogram_channel4_slot
//! \param histogram_vec
//! \param maxValue
//! 4 Channel 的MA
void Hist_MA_Dialog::toShowHistogram_channel4_slot(QVector<double> histogram_vec,int maxValue)
{
    QVector<double> showHistogram_vec;
    showHistogram_vec = histogram_vec;
    showHistogram_vec.resize(binSize);

    int maxY =0;
    for(int i=0;i<binSize; i++)
    {
        maxY = histogram_vec[i]>maxY ? histogram_vec[i]:maxY;
    }

    //显示直方图
    QSharedPointer<QCPAxisTickerText> textTicker(new QCPAxisTickerText);
    textTicker->setTicks(HistorgramTicks[3], HistorgramLabels[3]);
    ui->MA_4_widget->xAxis->setTicker(textTicker);
    ui->MA_4_widget->xAxis->setRange(0,binSize);
    ui->MA_4_widget->yAxis->setRange(0,maxY);
    regen[3]->setData(HistorgramTicks[0],showHistogram_vec);


    //计算MA曲线
    QVector<double> label_x(binSize);
    QVector<double> srcDataValue;
    QVector<double> MA_resValue(binSize);
    srcDataValue = showHistogram_vec;     //接下来需要对 srcDataValue 进行padding 操作
    int winSize = filterWindow.length();
    int padding = (winSize-1)/2;     //原始数据的前面和后面都要添加 padding的长度
    for(int i=0;i<padding;i++)
    {
        srcDataValue.insert(0,0);
    }
    srcDataValue.resize(binSize+winSize);

    qDebug()<<"srcDataValue.size="<<srcDataValue.length();

    //开始计算滤波
    float peakValue = 0;
    int tofIndex = 0;
    for(int index=0; index<binSize; index++)
    {
        label_x[index] = index;
        for(int len=0;len<winSize;len++)
        {
            MA_resValue[index] += srcDataValue[index+len]*filterWindow[len];
        }
        MA_resValue[index] = MA_resValue[index]/summary;
        if(MA_resValue[index]>peakValue)
        {
            peakValue = MA_resValue[index];
            tofIndex = index;
        }
    }

    ui->binningTof_label_4->setText(QString::number(tofIndex));
    ui->binningPeak_label_4->setText(QString::number(peakValue*summary));

    ui->MA_4_widget->graph(0)->setData(label_x,MA_resValue);
    ui->MA_4_widget->replot();
}
