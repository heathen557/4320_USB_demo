#ifndef FILTERWINDOW_DIALOG_H
#define FILTERWINDOW_DIALOG_H

#include <QDialog>
#include<QTableWidgetItem>
#include<QDebug>
#include<qfiledialog.h>
#include<QMessageBox>


namespace Ui {
class filterWindow_Dialog;
}

class filterWindow_Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit filterWindow_Dialog(QWidget *parent = 0);
    ~filterWindow_Dialog();
    void init_tableWidget();

    QTableWidgetItem  valueItem[100];


private slots:

    void clearItem();

    void on_ok_pushButton_clicked();

    void on_loadLocal_pushButton_clicked();

    void on_saveLocal_pushButton_clicked();

signals:
    void send_filterWindow_signal(QStringList);

private:
    Ui::filterWindow_Dialog *ui;
};

#endif // FILTERWINDOW_DIALOG_H
