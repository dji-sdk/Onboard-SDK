#include "djionboardsdk.h"
#include "ui_djionboardsdk.h"

#include <QFile>

DJIonboardSDK::DJIonboardSDK(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::DJIonboardSDK)
{

    ui->setupUi(this);

    port = new QSerialPort();
    driver = new QHardDriver(port);
    api = new CoreAPI(driver);

    send = new APIThread(api, 1);
    read = new APIThread(api, 2);

    key = new QByteArray;

    refreshPort();
    setPort();
    setBaudrate();
    openPort();

    send->start();
    read->start();

    QFile f("settings.ini");
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
        qDebug() << "fail to open";
    else
    {
        while (!f.atEnd())
        {
            QByteArray line = f.readLine();
            if (line.startsWith("ID:"))
                ui->lineEdit_ID->setText(line.remove(0, 3));
            else if (line.startsWith("KEY:"))
                ui->lineEdit_Key->setText(line.remove(0, 4));
        }
        f.close();
    }
}

DJIonboardSDK::~DJIonboardSDK() { delete ui; }

void DJIonboardSDK::refreshPort()
{
    ui->comboBox_portName->clear();
    auto ports = QSerialPortInfo::availablePorts();
    QStringList list;
    for (int i = 0; i < ports.length(); ++i)
    {
        list.append(ports[i].portName());
    }
    ui->comboBox_portName->addItems(list);
}

void DJIonboardSDK::closeEvent(QCloseEvent *)
{
    qDebug() << "close";
    QFile f("settings.ini");
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text))
        qDebug() << "fail to open";
    else
    {
        f.write(QString("ID:")
                    .append(ui->lineEdit_ID->text())
                    .append("\r\n")
                    .toUtf8());
        f.write(QString("KEY:")
                    .append(ui->lineEdit_Key->text())
                    .append("\r\n")
                    .toUtf8());
        f.close();
    }
}

void DJIonboardSDK::on_btn_portRefresh_clicked() { refreshPort(); }

void DJIonboardSDK::setBaudrate()
{
    int baudrate = ui->lineEdit_portBaudrate->text().toInt();
    driver->setBaudrate(baudrate);
}

void DJIonboardSDK::setPort()
{
    port->setPortName(ui->comboBox_portName->currentText());
}

void DJIonboardSDK::openPort()
{
    driver->init();
    if (port->isOpen())
        ui->btn_portOpen->setText(port->portName().append(" is open"));
    else
        ui->btn_portOpen->setText(port->portName().append(" not exit"));
}

void DJIonboardSDK::closePort()
{
    port->close();
    if (!port->isOpen())
    {
        ui->btn_portOpen->setText(port->portName().append(" closed"));
    }
}

void DJIonboardSDK::on_btn_portOpen_clicked()
{
    if (port == 0)
        ;
    else
    {
        if (port->isOpen())
            closePort();
        else
        {
            setPort();
            setBaudrate();
            openPort();
        }
    }
}

void DJIonboardSDK::on_comboBox_portName_currentIndexChanged(int index)
{
    if (index != -1)
    {
        closePort();
        setPort();
        closePort();
        setBaudrate();
        openPort();
    }
}

void DJIonboardSDK::on_btn_coreActive_clicked()
{
    ActivateData data;
    data.app_api_level = 2;
    data.app_ver = SDK_VERSION;
    data.app_id = ui->lineEdit_ID->text().toInt();
    data.app_bundle_id[0] = data.app_bundle_id[1] = 0x12; // for ios
                                                          // verification
    *key = ui->lineEdit_Key->text().toLocal8Bit();
    data.app_key = key->data();
    api->activate(&data, 0);
}

void DJIonboardSDK::on_btn_coreVersion_clicked() { api->getVersion(0); }

void DJIonboardSDK::on_btn_coreSetControl_clicked() { api->setControl(1, 0); }

void DJIonboardSDK::on_btn_VRC_resetAll_clicked()
{
    on_btn_VRC_resetRight_clicked();
    on_btn_VRC_resetLeft_clicked();
}

void DJIonboardSDK::on_btn_VRC_resetLeft_clicked()
{
    ui->slider_VRC_LH->setValue(1024);
    ui->slider_VRC_LV->setValue(1024);
}

void DJIonboardSDK::on_btn_VRC_resetRight_clicked()
{

    ui->slider_VRC_RH->setValue(1024);
    ui->slider_VRC_RV->setValue(1024);
}
