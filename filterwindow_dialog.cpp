#include "filterwindow_dialog.h"
#include "ui_filterwindow_dialog.h"

filterWindow_Dialog::filterWindow_Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::filterWindow_Dialog)
{
    ui->setupUi(this);

    init_tableWidget();
}

void filterWindow_Dialog::init_tableWidget()
{

    //    ui->tableWidget->horizontalHeader()->setStretchLastSection(true);
        ui->tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        ui->tableWidget->setRowCount(100);
        ui->tableWidget->setColumnCount(1);

        ui->tableWidget->horizontalHeader()->setStyleSheet("QHeaderView::section{background:rgb(240,240,240)}"); //设置表头背景色
        ui->tableWidget->verticalHeader()->setStyleSheet("QHeaderView::section{background:rgb(240,240,240)}");

        for(int i=0; i<100 ; i++)
        {
            ui->tableWidget->setItem(i,0,&valueItem[i]);
//            ui->tableWidget->setItem(i,1,&valueItem[i]);

//            addressItem[i].setTextAlignment(Qt::AlignCenter);
            valueItem[i].setTextAlignment(Qt::AlignCenter);
        }
}

void filterWindow_Dialog::clearItem()
{
    for(int i=0;i<100;i++)
    {
        valueItem[i].setText("");
    }
}




filterWindow_Dialog::~filterWindow_Dialog()
{
    delete ui;
}

void filterWindow_Dialog::on_ok_pushButton_clicked()
{
    QStringList valueList;
    QString valStr;
    int index=0;
    while(1)
    {
        valStr = valueItem[index].text().toUpper();
        if(valStr.isEmpty())
        {
            break;
        }

        valueList.append(valStr);
        index++;
    }

    emit send_filterWindow_signal(valueList);
    this->hide();



}

//!
//! \brief filterWindow_Dialog::on_loadLocal_pushButton_clicked
//!加载本地
void filterWindow_Dialog::on_loadLocal_pushButton_clicked()
{
    clearItem();   //首先清空内容

    QString file_path;
    //定义文件对话框类
    QFileDialog *fileDialog = new QFileDialog(this);
    //定义文件对话框标题
    fileDialog->setWindowTitle(QStringLiteral("请选择配置文件"));
    //设置默认文件路径
    fileDialog->setDirectory(".");
    //设置视图模式
    fileDialog->setViewMode(QFileDialog::Detail);
    //打印所有选择的文件的路径
    QStringList mimeTypeFilters;
    mimeTypeFilters <<QStringLiteral("寄存器配置文件(*.para)|*.para") ;
    fileDialog->setNameFilters(mimeTypeFilters);
    QStringList fileNames;
    if(fileDialog->exec())
    {
        fileNames = fileDialog->selectedFiles();
    }else
    {
        return;
    }
    file_path = fileNames[0];
    qDebug()<<" file_path = "<<fileNames[0]<<endl;

    QString checkStr = file_path.right(4);
    if("para" != checkStr)
    {
        QMessageBox::information(NULL,QStringLiteral("告警"),QStringLiteral("请选择正确的配置文件！"));
        return ;
    }

    QFile file(file_path);
    QString line[100];

    if (file.open(QIODevice::ReadOnly))
    {
        QTextStream in(&file);
        int i = 0;
        while (!in.atEnd())
        {
            line[i] = in.readLine();
            valueItem[i].setText(line[i].mid(0,2));
            i++;
        }
        file.close();
    }


}


//!
//! \brief filterWindow_Dialog::on_saveLocal_pushButton_clicked
//!保存本地
void filterWindow_Dialog::on_saveLocal_pushButton_clicked()
{
    QString filePath;

    QFileDialog *fileDialog = new QFileDialog(this);//创建一个QFileDialog对象，构造函数中的参数可以有所添加。
    fileDialog->setWindowTitle(tr("Save As"));//设置文件保存对话框的标题
    fileDialog->setAcceptMode(QFileDialog::AcceptSave);//设置文件对话框为保存模式
    fileDialog->setFileMode(QFileDialog::AnyFile);//设置文件对话框弹出的时候显示任何文件，不论是文件夹还是文件
    fileDialog->setViewMode(QFileDialog::Detail);//文件以详细的形式显示，显示文件名，大小，创建日期等信息；
    fileDialog->setGeometry(10,30,300,200);//设置文件对话框的显示位置
    fileDialog->setDirectory(".");//设置文件对话框打开时初始打开的位置
    QStringList mimeTypeFilters;
    mimeTypeFilters <<QStringLiteral("窗口配置文件(*.para)|*.para") ;
    fileDialog->setNameFilters(mimeTypeFilters);


    if(fileDialog->exec() == QDialog::Accepted)
    {
        filePath = fileDialog->selectedFiles()[0];//得到用户选择的文件名
        qDebug()<<" filePath = "<<filePath<<endl;
    }else
    {
        return ;
    }

    //获取当前控件上的内容,组装成 字符串
    int index = 0;
    QString textString;
    QString addrStr,valStr;
    while(1)
    {
        valStr = valueItem[index].text().toUpper();
        if(valStr.isEmpty())
        {
            break;
        }
        textString.append(valStr).append("\n");
        index++;
    }


    QFile file(filePath);
    file.open(QIODevice::WriteOnly|QIODevice::Text);
    QTextStream out(&file);
    out<<textString.toLocal8Bit()<<endl;

    QString str = QStringLiteral("文件保存成功，路径：") + filePath;
    QMessageBox::information(NULL,QStringLiteral("提示"),str);
}
