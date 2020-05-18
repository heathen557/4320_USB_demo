#ifndef CONTROL_TEXTEDIT_H
#define CONTROL_TEXTEDIT_H
#include<qtextedit.h>

class Control_textEdit : public QTextEdit
{
    Q_OBJECT
public:
    explicit Control_textEdit(QWidget *parent = 0);

    QSize sizeHint() const;

signals:

public slots:
};

#endif // CONTROL_TEXTEDIT_H
