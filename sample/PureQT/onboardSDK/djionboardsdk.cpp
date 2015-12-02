#include "djionboardsdk.h"
#include "ui_djionboardsdk.h"

#include <QFile>

void DJIonboardSDK::resetFlightData()
{
    flightx = 0;
    flighty = 0;
    flightz = 0;
    flightyaw = 0;
    updateFlightX();
    updateFlightY();
    updateFlightZ();
    updateFlightYaw();
}

DJIonboardSDK::DJIonboardSDK(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::DJIonboardSDK)
{
    sdk = this;

    ui->setupUi(this);

    //! @code init DJISDK
    port = new QSerialPort(this);
    driver = new QHardDriver(port);
    api = new CoreAPI(driver);

    send = new APIThread(api, 1, this);
    read = new APIThread(api, 2, this);

    key = new QByteArray;

    flight = new Flight(api);
    vrc = new VirtualRC(api);

    refreshPort();
    setPort();
    setBaudrate();
    openPort();

    send->start();
    read->start();
    //! @endcode init DJISDK

    //! @code init flight
    connect(ui->btg_flightHL, SIGNAL(buttonClicked(QAbstractButton *)), this,
            SLOT(on_btg_flight_HL(QAbstractButton *)));
    connect(ui->btg_flightVL, SIGNAL(buttonClicked(QAbstractButton *)), this,
            SLOT(on_btg_flight_VL(QAbstractButton *)));
    connect(ui->btg_flightYL, SIGNAL(buttonClicked(QAbstractButton *)), this,
            SLOT(on_btg_flight_YL(QAbstractButton *)));
    connect(ui->btg_flightCL, SIGNAL(buttonClicked(QAbstractButton *)), this,
            SLOT(on_btg_flight_CL(QAbstractButton *)));
    connect(ui->btg_flightSM, SIGNAL(buttonClicked(QAbstractButton *)), this,
            SLOT(on_btg_flight_SM(QAbstractButton *)));

    on_btg_flight_HL(ui->btg_flightHL->checkedButton());
    on_btg_flight_VL(ui->btg_flightVL->checkedButton());
    on_btg_flight_YL(ui->btg_flightYL->checkedButton());
    on_btg_flight_CL(ui->btg_flightCL->checkedButton());
    on_btg_flight_SM(ui->btg_flightSM->checkedButton());

    resetFlightData();

    autoSend = new QTimer();
    autoSend->setInterval(50);
    connect(autoSend, SIGNAL(timeout()), this, SLOT(autosend()));
    //! @endcode init flight

    //! @code init virtual RC
    vrcSend = new QTimer();
    vrcSend->setInterval(10);
    connect(vrcSend, SIGNAL(timeout()), this,
            SLOT(on_tmr_virtualRC_autosend()));
    //! @endcode init virtual RC

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

void DJIonboardSDK::setControlCallback(CoreAPI *This, Header *header)
{
    sdk->ui->btn_coreSetControl->setText("Release Control");
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
    data.app_key = key->data(); //! @todo memory leak fixme
    api->activate(&data);
}

void DJIonboardSDK::on_btn_coreVersion_clicked() { api->getVersion(); }

void DJIonboardSDK::on_btn_coreSetControl_clicked()
{
    if (ui->btn_coreSetControl->text() == "Obtain Contorl")
        api->setControl(true);
    else
        api->setControl(false);
}
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

void DJIonboardSDK::on_tmr_virtualRC_autosend() { vrc->sendData(); }

void DJIonboardSDK::updateFlightFlag()
{
    ui->lineEdit_flightFlag->clear();
    ui->lineEdit_flightFlag->setText(QString::number(flightFlag, 16));
}

void DJIonboardSDK::on_btg_flight_HL(QAbstractButton *button)
{
    QString name = button->text();
    flightFlag &= 0x3F;
    if (name == "Angle")
        flightFlag |= 0x00;
    else if (name == "Velocity")
        flightFlag |= 0x40;
    else if (name == "Possition")
        flightFlag |= 0x80;
    updateFlightFlag();
}

void DJIonboardSDK::on_btg_flight_VL(QAbstractButton *button)
{
    QString name = button->text();
    flightFlag &= 0xCF;
    if (name == "Thrust")
        flightFlag |= 0x20;
    else if (name == "Velocity")
        flightFlag |= 0x00;
    else if (name == "Possition")
        flightFlag |= 0x10;
    updateFlightFlag();
}

void DJIonboardSDK::on_btg_flight_YL(QAbstractButton *button)
{
    QString name = button->text();
    flightFlag &= 0xF7;
    if (name == "Angle")
        flightFlag |= 0x00;
    else
        flightFlag |= 0x08;
    updateFlightFlag();
}

void DJIonboardSDK::on_btg_flight_CL(QAbstractButton *button)
{
    QString name = button->text();
    flightFlag &= 0xF9;
    if (name == "Ground")
        flightFlag |= 0x00;
    else
        flightFlag |= 0x02;
    updateFlightFlag();
}
void DJIonboardSDK::on_btg_flight_SM(QAbstractButton *button)
{

    QString name = button->text();
    flightFlag &= 0xFE;
    if (name == "Disable")
        flightFlag |= 0x00;
    else
        flightFlag |= 0x01;
    updateFlightFlag();
}

void DJIonboardSDK::updateFlightX()
{
    ui->lineEdit_flight_X->setText(QString::number(flightx));
}

void DJIonboardSDK::updateFlightY()
{
    ui->lineEdit_flight_Y->setText(QString::number(flighty));
}

void DJIonboardSDK::updateFlightZ()
{
    ui->lineEdit_flight_Z->setText(QString::number(flightz));
}

void DJIonboardSDK::updateFlightYaw()
{
    ui->lineEdit_flight_Yaw->setText(QString::number(flightyaw));
}

void DJIonboardSDK::on_btn_camera_up_clicked()
{
    //! @todo
    qDebug() << "!";
}

void DJIonboardSDK::on_btn_flight_frount_pressed()
{
    flightx += 0.1f;
    updateFlightX();
}

void DJIonboardSDK::on_btn_flight_back_pressed()
{
    flightx -= 0.1f;
    updateFlightX();
}

void DJIonboardSDK::flightSend()
{
    FlightData data;
    data.ctrl_flag = flightFlag;
    data.roll_or_x = flightx;
    data.pitch_or_y = flighty;
    data.thr_z = flightz;
    data.yaw = flightyaw;
    flight->setFlight(&data);
}

void DJIonboardSDK::on_btn_flight_send_clicked() { flightSend(); }
void DJIonboardSDK::autosend() { flightSend(); }

void DJIonboardSDK::on_btn_flight_runTask_clicked()
{
    QString name = ui->btg_flightTask->checkedButton()->text();
    TASK type = TASK_GOHOME;
    if (name == "Take off")
        type = TASK_TAKEOFF;
    else if (name == "Landing")
        type = TASK_LANDING;

    flight->task(type);
}

void DJIonboardSDK::on_btn_flight_arm_clicked(bool checked)
{
    flight->setArm(checked);
}

void DJIonboardSDK::on_btn_flight_left_pressed()
{
    flighty += 0.1f;
    updateFlightY();
}

void DJIonboardSDK::on_btn_flight_right_pressed()
{
    flighty -= 0.1f;
    updateFlightY();
}

void DJIonboardSDK::on_pushButton_down_pressed()
{
    flightz -= 0.1f;
    updateFlightZ();
}

void DJIonboardSDK::on_btn_flight_up_pressed()
{
    flightz += 0.1f;
    updateFlightZ();
}

void DJIonboardSDK::on_btn_flight_leftYaw_pressed()
{
    flightyaw += 1.0f;
    updateFlightYaw();
}

void DJIonboardSDK::on_btn_flight_rightYaw_pressed()
{
    flightyaw -= 1.0f;
    updateFlightYaw();
}

void DJIonboardSDK::on_btn_flight_dataReset_clicked() { resetFlightData(); }

void DJIonboardSDK::on_lineEdit_flight_X_returnPressed()
{
    flightx = ui->lineEdit_flight_X->text().toFloat();
}

void DJIonboardSDK::on_lineEdit_flight_Y_returnPressed()
{
    flighty = ui->lineEdit_flight_Y->text().toFloat();
}

void DJIonboardSDK::on_lineEdit_flight_Z_returnPressed()
{
    flightz = ui->lineEdit_flight_Z->text().toFloat();
}

void DJIonboardSDK::on_lineEdit_flight_Yaw_returnPressed()
{
    flightyaw = ui->lineEdit_flight_Yaw->text().toFloat();
}

void DJIonboardSDK::on_cb_flight_autoSend_clicked(bool checked)
{
    if (checked)
        autoSend->start();
    else
        autoSend->stop();
}

void DJIonboardSDK::on_btn_virtualRC_send_clicked()
{
    vrcSend->start();
    vrc->sendData();
}

void DJIonboardSDK::on_btn_virtualRC_init_clicked()
{
    vrc->sentContorl(true, VirtualRC::CutOff_ToRealRC);
}

void DJIonboardSDK::on_btn_coreSet_clicked()
{
    uint8_t data[16];
    data[0] = ui->comboBox_1->currentIndex();
    data[1] = ui->comboBox_2->currentIndex();
    data[2] = ui->comboBox_3->currentIndex();
    data[3] = ui->comboBox_4->currentIndex();
    data[4] = ui->comboBox_5->currentIndex();
    data[5] = ui->comboBox_6->currentIndex();
    data[6] = ui->comboBox_7->currentIndex();
    data[7] = ui->comboBox_8->currentIndex();
    data[8] = ui->comboBox_9->currentIndex();
    data[9] = ui->comboBox_10->currentIndex();
    data[10] = ui->comboBox_11->currentIndex();
    data[11] = ui->comboBox_12->currentIndex();
    data[12] = 0;
    data[13] = 0;
    data[14] = 0;
    data[15] = 0;
    api->setBroadcastFeq(data);
}
