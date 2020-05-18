#ifndef AUTOCALIBRATION_DIALOG_H
#define AUTOCALIBRATION_DIALOG_H

#include <QDialog>

namespace Ui {
class autoCalibration_Dialog;
}

class autoCalibration_Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit autoCalibration_Dialog(QWidget *parent = 0);
    ~autoCalibration_Dialog();

private slots:
    void on_startCalibration_pushButton_clicked();

    void send_cali_success_slot(QString);

signals:
    void start_autoCalibration_signal(float );

private:
    Ui::autoCalibration_Dialog *ui;
};

#endif // AUTOCALIBRATION_DIALOG_H
