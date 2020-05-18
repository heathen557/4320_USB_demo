#include "control_textedit.h"

Control_textEdit::Control_textEdit(QWidget *parent) : QTextEdit(parent)
{

}

QSize Control_textEdit::sizeHint() const
{
return QSize( 350, 100 );
}
