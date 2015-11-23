#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <stdint.h>
#include <QSerialPortInfo>
#include <QtSerialPort/QSerialPort>
#include <QDebug>
#include <QMessageBox>
#include <QTimer>
#include  "about.h"
#include <QSettings>
#include <QFile>
#include <QFileDialog>
#include <QByteArray>
#include "DJI_Pro_Sample.h"

MainWindow *MainWindow::mainwindow_object = (MainWindow*)NULL;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    time = new QTimer(this);
    TakeoffDelay = new QTimer(this);
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
    connect(ui->actionAbout, SIGNAL(triggered()),this, SLOT(btn_About()));
    connect(this,SIGNAL(recv_data_signal(QByteArray)),this,SLOT(recv_data(QByteArray)));
    time->start(50);

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

    DJI_Pro_Register_Transparent_Transmission_Callback(MainWindow_Transparent_Transmission_Callback);

}

MainWindow::~MainWindow()
{
    delete ui;
}

MainWindow* MainWindow::Create_Instance(void)
{
    if(mainwindow_object == (MainWindow*)NULL)
    {
        mainwindow_object = new MainWindow;
    }
    return mainwindow_object;
}

MainWindow* MainWindow::Get_Instance(void)
{
    return Create_Instance();
}


void MainWindow::MainWindow_Activate_Callback(unsigned short res)
{
    char result[][50]={{"SDK_ACTIVATION_SUCCESS"},{"SDK_ACTIVE_PARAM_ERROR"},{"SDK_ACTIVE_DATA_ENC_ERROR"},\
                       {"SDK_ACTIVE_NEW_DEVICE"},{"SDK_ACTIVE_DJI_APP_NOT_CONNECT"},{" SDK_ACTIVE_DIJ_APP_NO_INTERNET"},\
                       {"SDK_ACTIVE_SERVER_REFUSED"},{"SDK_ACTIVE_LEVEL_ERROR"},{"SDK_ACTIVE_SDK_VERSION_ERROR"}};

    if(res >= 0 && res < 9)
    {
        MainWindow::Get_Instance()->ui->label_Activation_Status->setText(*(result+res));
    }
    else
    {
        MainWindow::Get_Instance()->ui->label_Activation_Status->setText("Unkown ERROR");
    }
}

void MainWindow::on_BtnActivation_clicked()
{
    user_act_data.app_id = ui->AppID->text().toInt();
    user_act_data.app_api_level = ui->AppLevel->currentText().toInt();
    user_act_data.app_ver = SDK_VERSION;
    user_act_data.app_bundle_id[0] = user_act_data.app_bundle_id[1] = 0x12;
    *keybuf = ui->AppKey->text().toLocal8Bit();
    user_act_data.app_key = keybuf->data();
    Save_Setting();
    ui->label_Activation_Status->setText("Activating,Waiting...");
    DJI_Pro_Activate_API(&user_act_data,MainWindow::MainWindow_Activate_Callback);
}

void MainWindow::on_btn_open_serialport_clicked()
{
    DJI_Pro_Hw *hw_instance = DJI_Pro_Hw::Pro_Hw_Get_Instance();
    if(OPEN_SERIAL == ui->btn_open_serialport->text())
    {

        if(true == hw_instance->Pro_Hw_Setup(ui->serial_comboBox->currentText(),\
                                              ui->baud_comboBox->currentText().toInt()))
         {
             hw_instance->start();
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
        hw_instance->Pro_Hw_Close();
        ui->btn_open_serialport->setText(OPEN_SERIAL);
        ui->btn_open_serialport->setIcon(QIcon(":/icon/Images/err.png"));
    }

}

void MainWindow::on_btn_nav_open_close_clicked()
{
    if(NAV_OPEN == ui->btn_nav_open_close->text())
    {
        DJI_Pro_Control_Management(1,NULL);
        ui->btn_nav_open_close->setText(NAV_CLOSE);
    }
    else
    {
       DJI_Pro_Control_Management(0,NULL);
       ui->btn_nav_open_close->setText(NAV_OPEN);
    }
}

void MainWindow::on_btn_Takeoff_clicked()
{
    DJI_Pro_Status_Ctrl(4,0);
    ui->btn_Takeoff->setEnabled(false);
    ui->btn_Landing->setEnabled(false);
    TakeoffDelay->start(10*1000);
}

void MainWindow::on_btn_Landing_clicked()
{
    DJI_Pro_Status_Ctrl(6,0);
    ui->btn_Takeoff->setEnabled(false);
    ui->btn_Landing->setEnabled(false);
    TakeoffDelay->start(10*1000);
}

void MainWindow::on_btn_GoHome_clicked()
{
    DJI_Pro_Status_Ctrl(1,0);
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

void MainWindow::Timeout_handle()
{
    static int count0 = 0;
    static int count1 = 0;
    unsigned char bat = 0;
    api_ctrl_info_data_t ctrl_info;

    char info[][32]={{"Remote controller"},
                     {"Mobile device"},
                     {"Onboard device"},
                    };

    statusBar()->showMessage(
                tr("RX: %1").arg(DJI_Pro_Hw::Pro_Hw_Get_Instance()->load_con));

    /* refresh battery capacity per second */
    if(++ count0 == 20)
    {
        QString str;
        DJI_Pro_Get_Bat_Capacity(&bat);
        str.setNum((uint)bat);
        str += '%';
        ui->label_Battery_Capacity->setText(str);
        count0 = 0;
    }

    /* refresh ctrl info per 500ms */
    if(++ count1 == 10)
    {
        DJI_Pro_Get_CtrlInfo(&ctrl_info);
        ui->label_Control_Device->setText(*(info + ctrl_info.cur_ctrl_dev_in_navi_mode));
        count1 = 0;
    }
}

void MainWindow::TakeoffDelay_handle()
{
    TakeoffDelay->stop();
    ui->btn_Takeoff->setEnabled(true);
    ui->btn_Landing->setEnabled(true);
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

    configIni->beginGroup("user");
    configIni->setValue("AppID", user_act_data.app_id);
    configIni->setValue("Key",user_act_data.app_key);
    configIni->setValue("ApiLevel", user_act_data.app_api_level);
    configIni->endGroup();

    configIni->setValue("Check",Get_Check(configIni));

    delete configIni;
}

void MainWindow::on_btn_gimbal_ctrl_clicked()
{
    if(DJI_Sample_Gimbal_Ctrl()< 0)
    {
        QMessageBox::warning(this,tr("Warning"),tr("Please waiting current sample finish"),QMessageBox::Ok);
    }
}

void MainWindow::on_btn_atti_ctrl_clicked()
{
    int ret = QMessageBox::warning(this, tr("Warning"),
                                   tr("Attention for flight safety.\n"
                                      "Make sure you are in a wide area if you are not using simulator."),
                                   QMessageBox::Ok | QMessageBox::Cancel);
    if(QMessageBox::Ok == ret)
    {
        if(DJI_Sample_Atti_Ctrl() < 0)
        {
            QMessageBox::warning(this,tr("Warning"),tr("Please waiting current sample finish"),QMessageBox::Ok);
        }
    }
    if(QMessageBox::Cancel == ret)
    {
    }
}

void MainWindow::on_btn_capture_clicked()
{
    DJI_Sample_Camera_Shot();
    printf("Take a picture\n");
}

void MainWindow::on_btn_start_video_clicked()
{
    DJI_Sample_Camera_Video_Start();
    ui->btn_start_video->setDown(true);
    printf("Start video\n");
}

void MainWindow::on_btn_stop_video_clicked()
{
    DJI_Sample_Camera_Video_Stop();
    ui->btn_start_video->setDown(false);
    printf("Stop video\n");
}

void MainWindow::on_btn_draw_circle_clicked()
{
    if(DJI_Sample_Funny_Ctrl(DRAW_CIRCLE_SAMPLE) < 0)
    {
        QMessageBox::warning(this,tr("Warning"),tr("Please waiting current sample finish"),QMessageBox::Ok);
    }
    else
    {
        printf("Start to draw circle\n");
    }
}

void MainWindow::on_btn_draw_square_clicked()
{
    if(DJI_Sample_Funny_Ctrl(DRAW_SQUARE_SAMPLE) < 0 )
    {
        QMessageBox::warning(this,tr("Warning"),tr("Please waiting current sample finish"),QMessageBox::Ok);
    }
    else
    {
        printf("Start to draw square\n");
    }
}

void MainWindow::on_btn_way_point_clicked()
{
    if(DJI_Sample_Funny_Ctrl(WAY_POINT_SAMPLE) < 0 )
    {
        QMessageBox::warning(this,tr("Warning"),tr("Please waiting current sample finish"),QMessageBox::Ok);
    }
    else
    {
        printf("Start to way point sample\n");
    }
}


void MainWindow::recv_data(QByteArray data)
{
    ui->TB_Recv->insertPlainText(data);
}

void MainWindow::MainWindow_Transparent_Transmission_Callback(unsigned char *buf,unsigned char len)
{
    QByteArray byte_array;
    byte_array.append((const char*)buf,len);
    emit MainWindow::Get_Instance()->recv_data_signal(byte_array);
}

void MainWindow::on_btn_Send_clicked()
{
    QString string =  ui->plainTextEdit_Send->toPlainText();

    unsigned char *data;
    QByteArray byte_array;

    byte_array = string.toLatin1();

    data = (unsigned char*)byte_array.data();

    DJI_Pro_Send_To_Mobile_Device(data,byte_array.length(),0);
}

void MainWindow::on_btn_Clear_clicked()
{
    ui->TB_Recv->clear();
    ui->plainTextEdit_Send->clear();
}


