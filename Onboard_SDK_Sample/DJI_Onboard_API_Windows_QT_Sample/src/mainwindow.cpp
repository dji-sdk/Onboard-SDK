#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "DJI_Pro_Test.h"
#include <stdint.h>
#include <QSerialPortInfo>
#include <QtSerialPort/QSerialPort>
#include <QDebug>
#include <DJI_Pro_Hw.h>
#include <QMessageBox>
#include "DJI_Pro_Hw.h"
#include "DJI_Pro_Codec.h"
#include <QTimer>
#include  "about.h"
#include <DJI_Pro_Test.h>
#include <QSettings>
#include <QFile>
#include <QFileDialog>
#include <QByteArray>
extern int activation_callback_flag;


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    time = new QTimer(this);
    TakeoffDelay = new QTimer(this);
    Activation = new QTimer(this);
    keybuf = new QByteArray(DEFAULT_KEY);

    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
           qDebug() <<"Name        : "<<info.portName().toStdString().data();
           qDebug() <<"Description : "<<info.description().toStdString().data();
           qDebug() <<"Manufacturer: "<<info.manufacturer().toStdString().data();
           // Example use QSerialPort
           ui->serial_comboBox->addItem(info.portName());
           printf(" \n");

       }
    ui->btn_open_serialport->setText(OPEN_SERIAL);
    ui->btn_nav_open_close->setText(NAV_OPEN);
    connect(time, SIGNAL(timeout()), this, SLOT(Timeout_handle()));
    connect(TakeoffDelay, SIGNAL(timeout()), this, SLOT(TakeoffDelay_handle()));
    connect(Activation, SIGNAL(timeout()), this, SLOT(Activation_handle()));
    connect(ui->actionAbout, SIGNAL(triggered()),this, SLOT(btn_About()));
    time->start(20);


    QFile *f = new QFile("Setting.ini");
    if(f->open(QIODevice::ReadOnly | QIODevice::Text))
    {
        f->close();
        QSettings *configIni = new QSettings(SETTING_FILENAMES, QSettings::IniFormat);
        int sum = configIni->value("Check",0).toInt();
        if(Get_Check(configIni) != sum || 0 == sum)
        {
            Set_Default_Ini();
        }
        delete configIni;
    }
    else
    {
        Set_Default_Ini();
    }
    delete f;

    Read_Setting();

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_BtnActivation_clicked()
{
    activation_msg.app_id = ui->AppID->text().toInt();
    activation_msg.app_api_level = ui->AppLevel->currentText().toInt();
    activation_msg.app_ver = 1;
    memcpy(activation_msg.app_bundle_id,"1234567890123456789012", 32);
    *keybuf = ui->AppKey->text().toLocal8Bit();
    key = keybuf->data();
    printf("key = %s\n",key);
    Save_Setting();
    DJI_Onboard_API_Activation();
    Activation->start(2000);;
}

void MainWindow::on_btn_open_serialport_clicked()
{
    if(OPEN_SERIAL == ui->btn_open_serialport->text())
    {

        if(true == Pro_Hw.Pro_Hw_Setup(ui->serial_comboBox->currentText(),\
                                              ui->baud_comboBox->currentText().toInt()))
         {
             Pro_Hw.start();
             //QMessageBox::information(this, tr(" "), "Port Open Succeed", QMessageBox::Ok);
             ui->btn_open_serialport->setIcon(QIcon(":/icon/Images/ok.png"));
             ui->btn_open_serialport->setText(CLOSE_SERIAL);
         }
        else
         {
             //QMessageBox::warning(this, tr("Warning"), "Port Open Failed", QMessageBox::Ok);
             ui->btn_open_serialport->setIcon(QIcon(":/icon/Images/err.png"));
        }
    }
    else if(CLOSE_SERIAL == ui->btn_open_serialport->text())
    {
        Pro_Hw.Pro_Hw_Close();
        ui->btn_open_serialport->setText(OPEN_SERIAL);
        ui->btn_open_serialport->setIcon(QIcon(":/icon/Images/err.png"));
    }

}

void MainWindow::on_btn_nav_open_close_clicked()
{
    if(NAV_OPEN == ui->btn_nav_open_close->text())
    {
        DJI_Onboard_API_Control(1);
        ui->btn_nav_open_close->setText(NAV_CLOSE);
    }
    else
    {
       DJI_Onboard_API_Control(0);
       ui->btn_nav_open_close->setText(NAV_OPEN);
    }
}

void MainWindow::on_btn_Takeoff_clicked()
{
   DJI_Onboard_API_Takeoff();
   ui->btn_Takeoff->setEnabled(false);
   ui->btn_Landing->setEnabled(false);
   TakeoffDelay->start(13*1000);
}

void MainWindow::on_btn_Landing_clicked()
{
    DJI_Onboard_API_Landing();
    ui->btn_Takeoff->setEnabled(false);
    ui->btn_Landing->setEnabled(false);
    TakeoffDelay->start(12*1000);
}

void MainWindow::on_btn_GoHome_clicked()
{
    DJI_Onboard_API_Gohome();
}

void MainWindow::on_btn_update_com_clicked()
{
    ui->serial_comboBox->clear();
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
           qDebug() << "Name        : " << info.portName();
           qDebug() << "Description : " << info.description();
           qDebug() << "Manufacturer: " << info.manufacturer();
           // Example use QSerialPort
           ui->serial_comboBox->addItem(info.portName());
           qDebug()<<"\n";
       }
}

void MainWindow::on_btn_simpletask_clicked()
{
    int ret = QMessageBox::warning(this, tr("Warning"),
                                   tr("Attention for flight safety.\n"
                                      "Make sure you are in a wide area if you are not using simulator."),
                                   QMessageBox::Ok | QMessageBox::Cancel);
    if(QMessageBox::Ok == ret)
    {
        simple_task_num = 4;
        printf("OK\n");
    }
    if(QMessageBox::Cancel == ret)
    {
        printf("Cancel\n");
    }
}

void MainWindow::Timeout_handle()
{
    unsigned char battery, actavation_status, ctrl_device;
    statusBar()->showMessage(tr("RX: %1")
                                   .arg(Pro_Hw.load_con));

    DJI_Get_Info(&battery, &actavation_status, &ctrl_device);
    //if(actavation_status == 0x0)
   //     ui->label_Activation_Status->setText("Activation pass");
   // else
    //    ui->label_Activation_Status->setText("unknown");

    if(battery == 0xFF)
        ui->label_Battery_Capacity->setText("invalid");
    else
    {
        QString str;
        str.setNum((uint)battery);
        str += '%';
        ui->label_Battery_Capacity->setText(str);
    }


    if(ctrl_device == 0x0)
        ui->label_Control_Device->setText("RC");
    else if(ctrl_device == 0x1)
        ui->label_Control_Device->setText("APP");
    else if(ctrl_device == 0x2)
        ui->label_Control_Device->setText("Third party onboard device");
    else
        ui->label_Control_Device->setText("unknown");


    if(activation_callback_flag!=0)
    {
        if(1==activation_callback_flag)
        {
             char result[][50]={{"ACTIVATION_SUCCESS"},{"PARAM_ERROR"},{"DATA_ENC_ERROR"},{"NEW_DEVICE_TRY_AGAIN"},{"DJI_APP_TIMEOUT"},{" DJI_APP_NO_INTERNET"},{"SERVER_REFUSED"},{"LEVEL_ERROR"}};
             ui->label_Activation_Status->setText(*(result+actavation_status));
             if(3 != actavation_status)
             {
                 Activation->stop();
                 QMessageBox::information(NULL, "Warning", *(result+actavation_status), QMessageBox::Ok);

             }


             activation_callback_flag=0;
             if(3 != actavation_status)
             {
                 Activation->stop();
             }
        }
        else
        {
            QMessageBox::warning(NULL, "Warning", "Activation Failed", QMessageBox::Ok);
            activation_callback_flag=0;
        }
    }
}

void MainWindow::TakeoffDelay_handle()
{
    TakeoffDelay->stop();
    ui->btn_Takeoff->setEnabled(true);
    ui->btn_Landing->setEnabled(true);
}


void MainWindow::Activation_handle()
{
    DJI_Onboard_API_Activation();
}
void MainWindow::btn_About()
{
    About about;
    about.exec();
}

int MainWindow::Get_Check(QSettings *set)
{
    QStringList keys = set->allKeys();
    int sum = 0;
    QString str;
    keys.removeOne("Check");
    foreach (str, keys)
    {
        int i=0,len = 0;
        len = set->value(str,-1).toString().size();
        for(i=0; i<len; i++)
        {
            sum += (int)set->value(str,-1).toByteArray().at(i);
        }
    }
    return sum;
}

void MainWindow::Set_Default_Ini()
{
    QSettings *configIni = new QSettings(SETTING_FILENAMES, QSettings::IniFormat);
    configIni->beginGroup("user");
    configIni->setValue("AppID", 10086);
    configIni->setValue("Key",DEFAULT_KEY);
    configIni->setValue("ApiLevel", 2);
    configIni->endGroup();

    configIni->setValue("Check",Get_Check(configIni));

    activation_msg.app_id = ui->AppID->text().toInt();
    activation_msg.app_api_level = ui->AppLevel->currentText().toInt();
    activation_msg.app_ver = 1;
    memcpy(activation_msg.app_bundle_id,"1234567890123456789012", 32);
    *keybuf = ui->AppKey->text().toLocal8Bit();
    key = keybuf->data();

    delete configIni;

    Read_Setting();
}
void MainWindow::Read_Setting()
{
     QSettings *configIni = new QSettings(SETTING_FILENAMES, QSettings::IniFormat);
     int sum = configIni->value("Check",0).toInt();
     if(Get_Check(configIni) == sum)
     {
        configIni->beginGroup("user");
        ui->AppID->setText(configIni->value("AppID").toString());
        ui->AppKey->setText(configIni->value("Key").toString());
        ui->AppLevel->setCurrentIndex(ui->AppLevel->findText(configIni->value("ApiLevel").toString()));
        configIni->endGroup();
     }
     else
     {
         printf("%s,SettingFile Err\n",
                __func__);
         Set_Default_Ini();
     }

}

void MainWindow::Save_Setting()
{
    QSettings *configIni = new QSettings(SETTING_FILENAMES, QSettings::IniFormat);

    activation_msg.app_id = ui->AppID->text().toInt();
    activation_msg.app_api_level = ui->AppLevel->currentText().toInt();
    activation_msg.app_ver = 1;
    memcpy(activation_msg.app_bundle_id,"1234567890123456789012", 32);
    *keybuf = ui->AppKey->text().toLocal8Bit();
    key = keybuf->data();

    configIni->beginGroup("user");
    configIni->setValue("AppID", activation_msg.app_id);
    configIni->setValue("Key",key);
    configIni->setValue("ApiLevel", activation_msg.app_api_level);
    configIni->endGroup();

    configIni->setValue("Check",Get_Check(configIni));

    delete configIni;
}
