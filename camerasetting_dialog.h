#ifndef CAMERASETTING_DIALOG_H
#define CAMERASETTING_DIALOG_H

#include <QDialog>
#include<QSettings>

namespace Ui {
class CameraSetting_Dialog;
}

class CameraSetting_Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit CameraSetting_Dialog(QWidget *parent = 0);
    ~CameraSetting_Dialog();

    void init_parameterFile();
private slots:
    void on_ok_pushButton_clicked();

    void on_cancel_pushButton_clicked();

private:
    Ui::CameraSetting_Dialog *ui;

signals:
    void alter_focal_integrate_signal(float,float,int);   //修改焦距、积分次数的槽函数
};

#endif // CAMERASETTING_DIALOG_H
