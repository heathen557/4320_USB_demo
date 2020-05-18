#include "mylabel.h"
#include<QDebug>

myLabel::myLabel(QWidget *parent) : QLabel(parent)
{

}
void myLabel::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton){

        int x = event->pos().x();
        int y = event->pos().y();
        emit queryPixSignal(x,y);
    }
}


QSize myLabel::sizeHint() const
{
     return QSize( 360, 250 );
}
