#ifndef MYLABEL_H
#define MYLABEL_H

//#include <QWidget>
#include<QLabel>
#include <QMouseEvent>

class myLabel : public QLabel
{
    Q_OBJECT
public:
    explicit myLabel(QWidget *parent = 0);

    void mousePressEvent(QMouseEvent *event);        //单击

    QSize sizeHint() const;


signals:
    void queryPixSignal(int,int);

public slots:
};

#endif // MYLABEL_H
