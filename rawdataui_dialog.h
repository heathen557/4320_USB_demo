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

private slots:
    void on_SelFilePath_pushButton_clicked();

    void on_start_pushButton_clicked();

signals:

    void on_start_rawDataSave_signal();

private:
    Ui::rawDataUI_Dialog *ui;

    QString file_path;
};

#endif // RAWDATAUI_DIALOG_H
