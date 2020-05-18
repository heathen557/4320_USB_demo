#include "filesave_dialog.h"
#include "ui_filesave_dialog.h"

fileSave_Dialog::fileSave_Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::fileSave_Dialog)
{
    ui->setupUi(this);
    ui->lineEdit->setReadOnly(true);

    ui->RawTof_checkBox->setEnabled(false);

//    disable_checkBoxUI();
}

void fileSave_Dialog::disable_checkBoxUI()
{
    ui->RawTof_checkBox->setVisible(false);
    ui->pileUpTof_checkBox->setVisible(false);
    ui->filterTof_checkBox->setVisible(false);
    ui->RawPeak_checkBox->setVisible(false);
    ui->filterPeak_checkBox->setVisible(false);
    ui->X_checkBox->setVisible(false);
    ui->Y_checkBox->setVisible(false);
    ui->Z_checkBox->setVisible(false);

}


fileSave_Dialog::~fileSave_Dialog()
{
    delete ui;
}


//!
//! \brief fileSave_Dialog::on_SelFilePath_pushButton_clicked
//!  选出一个文件路径
void fileSave_Dialog::on_SelFilePath_pushButton_clicked()
{
    file_path = QFileDialog::getExistingDirectory(this,QStringLiteral("请选择文件保存路径..."),"./");
    if(file_path.isEmpty())
    {
        qDebug()<<QStringLiteral("没有选择路径")<<endl;
        QMessageBox::information(NULL,QStringLiteral("告警"),QStringLiteral("保存路径不能为空"));
        return;
    }
    else
    {
        file_path.append("/");
        qDebug() << file_path << endl;
        ui->lineEdit->setText(file_path);
    }
}

//!
//! \brief fileSave_Dialog::on_start_saveFile_pushButton_clicked
//!开始保存槽函数
void fileSave_Dialog::on_start_saveFile_pushButton_clicked()
{
    bool isRawTof = ui->RawTof_checkBox->isChecked();
    bool isPileUpTof = ui->pileUpTof_checkBox->isChecked();
    bool isFilterTof = ui->filterTof_checkBox->isChecked();
    bool isRawPeak = ui->RawPeak_checkBox->isChecked();
    bool isFilterPeak = ui->filterPeak_checkBox->isChecked();
    bool isX = ui->X_checkBox->isChecked();
    bool isY = ui->Y_checkBox->isChecked();
    bool isZ = ui->Z_checkBox->isChecked();


    if(!file_path.isEmpty())
    {
        ui->SavedFrame_label->setText("0");
        emit alter_fileSave_signal(isRawTof,isPileUpTof,isFilterTof,isRawPeak,isFilterPeak,isX,isY,isZ);
        emit isSaveFlagSignal(true,file_path,0);
    }else
    {
        QMessageBox::information(NULL,QStringLiteral("告警"),QStringLiteral("保存路径不能为空"));
    }


}

//!
//! \brief fileSave_Dialog::on_cancel_save_pushButton_clicked
//!取消保存 槽函数
void fileSave_Dialog::on_cancel_save_pushButton_clicked()
{
    emit isSaveFlagSignal(false,file_path,0);

}

//!
//! \brief fileSave_Dialog::send_savedFileIndex_slot
//! \param index
//!显示已经保存的第几个文件
void fileSave_Dialog::send_savedFileIndex_slot(int index)
{
    ui->SavedFrame_label->setText(QString::number(index));
}
