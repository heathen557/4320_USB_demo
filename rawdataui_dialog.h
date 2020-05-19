#ifndef RAWDATAUI_DIALOG_H
#define RAWDATAUI_DIALOG_H

#include <QDialog>
#include"globaldata.h"

namespace Ui {
class rawDataUI_Dialog;
}

class rawDataUI_Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit rawDataUI_Dialog(QWidget *parent = 0);
    ~rawDataUI_Dialog();

public slots:
    void on_SelFilePath_pushButton_clicked();

    void on_start_pushButton_clicked();

    void send_savedFileIndex_slot(int);  //显示存储了多少帧


signals:



     void on_start_rawDataSave_signal(QString );

private:
    Ui::rawDataUI_Dialog *ui;

    QString file_path;
};

#endif // RAWDATAUI_DIALOG_H
