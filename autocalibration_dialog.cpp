#include "autocalibration_dialog.h"
#include "ui_autocalibration_dialog.h"
#include<QMessageBox>

autoCalibration_Dialog::autoCalibration_Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::autoCalibration_Dialog)
{
    ui->setupUi(this);
}

autoCalibration_Dialog::~autoCalibration_Dialog()
{
    delete ui;
}


//!
//! \brief autoCalibration_Dialog::on_startCalibration_pushButton_clicked
//! 1、开始校正
//! 2、发送信号到数据处理线程
//! 3、首先清空tofOffsetArray数组，设置为0
//! 4、设置校正的 flag = true
//! 5、把 31*256 +127 31*256+128 32*256+127 32*256+128 的tof均值传入给处理函数                         （31 32）/（127 128）
//! 6、数据处理函数首先判断够不够100帧，满100帧时，分别取y的均值，分别逆运算为resTof
//! 7、计算理论的realTof值，根据用户指定的距离(单位:m);
//! 8、offset = realTof - resTof
//! 9、写入本地文件，程序重新设置tofOffsetArray数组，flag=false
void autoCalibration_Dialog::on_startCalibration_pushButton_clicked()
{
    ui->status_label->setText(QStringLiteral("校正中..."));
    float meters = ui->lineEdit->text().toFloat();
    emit start_autoCalibration_signal(meters);
}


void autoCalibration_Dialog::send_cali_success_slot(QString msg)
{
    ui->status_label->setText(QStringLiteral("校准完成"));
    QMessageBox::information(NULL,QStringLiteral("校准完成"),msg);
}


