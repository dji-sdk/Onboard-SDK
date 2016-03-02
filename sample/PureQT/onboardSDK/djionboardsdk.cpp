#include "djionboardsdk.h"
#include "ui_djionboardsdk.h"

#include <QFile>
#include <QFileDialog>
#include <iostream>
#include <fstream>
#include <string>

void DJIonboardSDK::initSDK()
{
    port = new QSerialPort(this);
    driver = new QHardDriver(port);
    driver->setDisplay(ui->tb_display);
    api = new CoreAPI(driver);

    send = new APIThread(api, 1, port);
    read = new APIThread(api, 2, port);

    key = new QByteArray;

    flight = new Flight(api);
    follow = new Follow(api);
    vrc = new VirtualRC(api);
    cam = new Camera(api);
    hp = new HotPoint(api);
    wp = new WayPoint(api);

    refreshPort();
    setPort();
    setBaudrate();
    openPort();

    send->start();
    read->start();
}

void DJIonboardSDK::initFlight()
{
    connect(ui->btg_flightHL, SIGNAL(buttonClicked(QAbstractButton *)), this,
            SLOT(on_btg_flight_HL(QAbstractButton *)));
    connect(ui->btg_flightVL, SIGNAL(buttonClicked(QAbstractButton *)), this,
            SLOT(on_btg_flight_VL(QAbstractButton *)));
    connect(ui->btg_flightYL, SIGNAL(buttonClicked(QAbstractButton *)), this,
            SLOT(on_btg_flight_YL(QAbstractButton *)));
    connect(ui->btg_flightCL, SIGNAL(buttonClicked(QAbstractButton *)), this,
            SLOT(on_btg_flight_CL(QAbstractButton *)));
   // connect(ui->btg_flightSM, SIGNAL(buttonClicked(QAbstractButton *)), this,
    //        SLOT(on_btg_flight_SM(QAbstractButton *)));

    on_btg_flight_HL(ui->btg_flightHL->checkedButton());
    on_btg_flight_VL(ui->btg_flightVL->checkedButton());
    on_btg_flight_YL(ui->btg_flightYL->checkedButton());
    on_btg_flight_CL(ui->btg_flightCL->checkedButton());

    resetFlightData();

    autoSend = new QTimer();
    autoSend->setInterval(50); // 20Hz
    connect(autoSend, SIGNAL(timeout()), this, SLOT(filght_autosend()));
}

void DJIonboardSDK::initCamera()
{
    connect(ui->btg_cameraAngle, SIGNAL(buttonClicked(QAbstractButton *)), this,
            SLOT(on_btg_cameraAngle(QAbstractButton *)));
    connect(ui->btg_cameraYaw, SIGNAL(buttonClicked(QAbstractButton *)), this,
            SLOT(on_btg_cameraYaw(QAbstractButton *)));
    connect(ui->btg_cameraRoll, SIGNAL(buttonClicked(QAbstractButton *)), this,
            SLOT(on_btg_cameraRoll(QAbstractButton *)));
    connect(ui->btg_cameraPitch, SIGNAL(buttonClicked(QAbstractButton *)), this,
            SLOT(on_btg_cameraPitch(QAbstractButton *)));

    camFlag = 0;

    on_btg_cameraAngle(ui->btg_cameraAngle->checkedButton());
    on_btg_cameraYaw(ui->btg_cameraYaw->checkedButton());
    on_btg_cameraRoll(ui->btg_cameraRoll->checkedButton());
    on_btg_cameraPitch(ui->btg_cameraPitch->checkedButton());
    updateCameraFlag();

    cameraSend = new QTimer();
    cameraSend->setInterval(100); // 10Hz
    connect(cameraSend, SIGNAL(timeout()), this, SLOT(on_tmr_Camera_autosend()));
}

void DJIonboardSDK::initVirtualRC()
{
    vrcSend = new QTimer();
    vrcSend->setInterval(200); // 5Hz
    connect(vrcSend, SIGNAL(timeout()), this, SLOT(on_tmr_VRC_autosend()));
}

void DJIonboardSDK::initFollow()
{
    FollowTarget targetBase;
    targetBase.latitude = 0;
    targetBase.longitude = 0;
    targetBase.angle = 0;
    targetBase.height = 0;
    follow->setTarget(targetBase);
    followSend = new QTimer();
    followSend->setInterval(20); // 50Hz
    connect(followSend, SIGNAL(timeout()), this, SLOT(on_tmr_follow_send()));
}

void DJIonboardSDK::initWayPoint()
{
    waypointData = new QStandardItemModel();
    waypointData->setHorizontalHeaderItem(0, new QStandardItem(QObject::tr("No.")));
    waypointData->setHorizontalHeaderItem(1, new QStandardItem(QObject::tr("Latitude")));
    waypointData->setHorizontalHeaderItem(2, new QStandardItem(QObject::tr("Longitude")));
    waypointData->setHorizontalHeaderItem(3, new QStandardItem(QObject::tr("Altitude")));
    waypointData->setHorizontalHeaderItem(4, new QStandardItem(QObject::tr("Damping")));
    waypointData->setHorizontalHeaderItem(5, new QStandardItem(QObject::tr("Yaw")));
    waypointData->setHorizontalHeaderItem(6, new QStandardItem(QObject::tr("Pitch")));
    waypointData->setHorizontalHeaderItem(7, new QStandardItem(QObject::tr("TurnMode")));
    waypointData->setHorizontalHeaderItem(8, new QStandardItem(QObject::tr("Anction Num")));
    waypointData->setHorizontalHeaderItem(9, new QStandardItem(QObject::tr("Time limit")));
    waypointData->setHorizontalHeaderItem(10, new QStandardItem(QObject::tr("Repeat times")));

    nullAction = initAction();
    currentAction = nullAction;



    connect(waypointData, SIGNAL(dataChanged(QModelIndex, QModelIndex, QVector<int>)), this,
            SLOT(on_waypoint_data_changed(QModelIndex, QModelIndex, QVector<int>)));


}

void DJIonboardSDK::initScript()
{
    scriptSDK = new ConboardSDKScript(api);
    ScriptThread *st = new ScriptThread(scriptSDK);
    st->start();
    scriptlog = new std::fstream("script.log");
    std::cout.rdbuf(scriptlog->rdbuf());
}

DJIonboardSDK::DJIonboardSDK(QWidget *parent) : QMainWindow(parent), ui(new Ui::DJIonboardSDK)
{
    ui->setupUi(this);
    on_tabWidget_currentChanged(ui->tabWidget->currentIndex());

    //! @code mission webview
    webView = new QWebView(this); // new QWebEngineView(this);
    QHBoxLayout *weblayout = new QHBoxLayout();

    weblayout->addWidget(webView);
    //! @endcode mission webview

    initSDK();
    initFlight();
    initCamera();
    initFollow();
    initWayPoint();
    initVirtualRC();

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

    //! @code version control
    timerBroadcast = new QTimer();
    on_cb_core_mechine_activated(ui->cb_core_mechine->currentIndex());
#ifdef SDK_VERSION_2_3

#endif // SDK_VERSION_2_3
    //! @endcode

    connect(timerBroadcast, SIGNAL(timeout()), this, SLOT(on_tmr_Broadcast()));
    timerBroadcast->start(300);
    initScript();
}

DJIonboardSDK::~DJIonboardSDK()
{
    delete ui;
    scriptlog->close();
}

void DJIonboardSDK::refreshPort()
{
    ui->comboBox_portName->clear();
    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
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
        f.write(QString("ID:").append(ui->lineEdit_ID->text()).append("\r\n").toUtf8());
        f.write(QString("KEY:").append(ui->lineEdit_Key->text()).append("\r\n").toUtf8());
        f.close();
    }
}

void DJIonboardSDK::setControlCallback(CoreAPI *This, Header *header, UserData userData)
{
    DJIonboardSDK *sdk = (DJIonboardSDK *)userData;
    unsigned short ack_data = AC_COMMON_NO_RESPONSE;
    unsigned char data = 0x1;

    if (header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data, ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
    }
    else
    {
        API_LOG(sdk->driver, ERROR_LOG, "ACK is exception,seesion id %d,sequence %d\n",
                header->sessionID, header->sequenceNumber);
    }

    switch (ack_data)
    {
        case ACK_SETCONTROL_NEED_MODE_F:
            if (sdk)
                sdk->ui->btn_coreSetControl->setText("Swtich to mod F");
            else
                API_LOG(sdk->driver, ERROR_LOG, "known SDK pointer 0.");
            break;
        case ACK_SETCONTROL_RELEASE_SUCCESS:
            if (sdk)
                sdk->ui->btn_coreSetControl->setText("Obtain Control");
            else
                API_LOG(sdk->driver, ERROR_LOG, "known SDK pointer 0.");
            break;
        case ACK_SETCONTROL_OBTAIN_SUCCESS:
            if (sdk)
                sdk->ui->btn_coreSetControl->setText("Release Control");
            else
                API_LOG(sdk->driver, ERROR_LOG, "known SDK pointer 0.");
            break;
        case ACK_SETCONTROL_OBTAIN_RUNNING:
            This->send(2, encrypt, SET_CONTROL, CODE_SETCONTROL, &data, 1, 500, 2,
                       DJIonboardSDK::setControlCallback, userData);
            break;
        case ACK_SETCONTROL_RELEASE_RUNNING:
            data = 0;
            This->send(2, encrypt, SET_CONTROL, CODE_SETCONTROL, &data, 1, 500, 2,
                       DJIonboardSDK::setControlCallback, userData);
            break;
    }
    //! @note For debug, all functional print is moving to this function,
    //! default API callback is not necessary.
    CoreAPI::setControlCallback(This, header);
}

void DJIonboardSDK::activationCallback(CoreAPI *This, Header *header, UserData userData)
{
    DJIonboardSDK *sdk = (DJIonboardSDK *)userData;
    volatile unsigned short ack_data;
    if (header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data, ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
        if (ack_data == ACK_ACTIVE_NEW_DEVICE)
        {
            sdk->ui->btn_coreActive->setText("New Device");
        }
        else
        {
            if (ack_data == ACK_ACTIVE_SUCCESS)
            {
                sdk->ui->btn_coreActive->setText("Success");
            }
            else
            {
                sdk->ui->btn_coreActive->setText("Error");
            }
        }
    }
    else
    {
        sdk->ui->btn_coreActive->setText("Decode Error");
    }
    This->activateCallback(This, header);
}

void DJIonboardSDK::hotpintReadCallback(CoreAPI *This, Header *header, UserData userData)
{
    DJIonboardSDK *sdk = (DJIonboardSDK *)userData;
    HotPoint::readCallback(This, header, sdk->hp);
    API_LOG(This->getDriver(), STATUS_LOG, "Refreshing data");

}

void DJIonboardSDK::on_btn_portRefresh_clicked() { refreshPort(); }

void DJIonboardSDK::setBaudrate()
{
    int baudrate = ui->lineEdit_portBaudrate->text().toInt();
    driver->setBaudrate(baudrate);
}

void DJIonboardSDK::setPort() { port->setPortName(ui->comboBox_portName->currentText()); }

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
    data.reserved = 2;
    data.ID = ui->lineEdit_ID->text().toInt();
    *key = ui->lineEdit_Key->text().toLocal8Bit();
    data.encKey = key->data(); //! @warning memory leak fixme
    api->activate(&data, DJIonboardSDK::activationCallback, this);
}

void DJIonboardSDK::on_btn_coreVersion_clicked() { api->getVersion(); }

void DJIonboardSDK::on_btn_coreSetControl_clicked()
{
    if (ui->btn_coreSetControl->text() == "Release Control")
        api->setControl(false, DJIonboardSDK::setControlCallback, this);
    else
        api->setControl(true, DJIonboardSDK::setControlCallback, this);
}
void DJIonboardSDK::on_btn_VRC_resetAll_clicked()
{
    on_btn_VRC_resetRight_clicked();
    on_btn_VRC_resetLeft_clicked();
}

void DJIonboardSDK::on_btn_VRC_resetLeft_clicked()
{

}

void DJIonboardSDK::on_btn_VRC_resetRight_clicked()
{

}

void DJIonboardSDK::on_tmr_VRC_autosend()
{
    VirtualRCData data;

}

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
        flightFlag |= Flight::HORIZONTAL_ANGLE;
    else if (name == "Velocity")
        flightFlag |= Flight::HORIZONTAL_VELOCITY;
    else if (name == "Possition")
        flightFlag |= Flight::HORIZONTAL_POSSITION;
    updateFlightFlag();
}

void DJIonboardSDK::on_btg_flight_VL(QAbstractButton *button)
{
    QString name = button->text();
    flightFlag &= 0xCF;
    if (name == "Thrust")
        flightFlag |= Flight::VERTICAL_THRUST;
    else if (name == "Velocity")
        flightFlag |= Flight::VERTICAL_VELOCITY;
    else if (name == "Possition")
        flightFlag |= Flight::VERTICAL_POSSITION;
    updateFlightFlag();
}

void DJIonboardSDK::on_btg_flight_YL(QAbstractButton *button)
{
    QString name = button->text();
    flightFlag &= 0xF7;
    if (name == "Angle")
        flightFlag |= Flight::YAW_ANGLE;
    else
        flightFlag |= Flight::YAW_PALSTANCE;
    updateFlightFlag();
}

void DJIonboardSDK::on_btg_flight_CL(QAbstractButton *button)
{
    QString name = button->text();
    flightFlag &= 0xF9;
    if (name == "Ground")
        flightFlag |= Flight::HORIZONTAL_GROUND;
    else
        flightFlag |= Flight::HORIZONTAL_BODY;
    updateFlightFlag();
}


void DJIonboardSDK::updateFlightX()
{
}

void DJIonboardSDK::updateFlightY()
{
}

void DJIonboardSDK::updateFlightZ()
{
}

void DJIonboardSDK::updateFlightYaw()
{
}

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
    data.flag = flightFlag;
    data.x = flightx;
    data.y = flighty;
    data.z = flightz;
    data.yaw = flightyaw;
    flight->setFlight(&data);
}

void DJIonboardSDK::on_btn_flight_send_clicked() { flightSend(); }
void DJIonboardSDK::filght_autosend() { flightSend(); }

void DJIonboardSDK::on_btn_flight_runTask_clicked()
{
    QString name = ui->btg_flightTask->checkedButton()->text();
    Flight::TASK type = Flight::TASK_GOHOME;
    if (name == "Take off")
        type = Flight::TASK_TAKEOFF;
    else if (name == "Landing")
        type = Flight::TASK_LANDING;

    flight->task(type);
}

void DJIonboardSDK::on_btn_flight_arm_clicked(bool checked) { flight->setArm(checked); }

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

void DJIonboardSDK::on_btn_flight_down_pressed()
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
}

void DJIonboardSDK::on_lineEdit_flight_Y_returnPressed()
{
}

void DJIonboardSDK::on_lineEdit_flight_Z_returnPressed()
{
}

void DJIonboardSDK::on_lineEdit_flight_Yaw_returnPressed()
{
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
    vrc->setControl(true, VirtualRC::CutOff_ToRealRC);
    vrcSend->start();
    vrc->sendData();
}

void DJIonboardSDK::on_btn_virtualRC_init_clicked()
{
    vrc->setControl(true, VirtualRC::CutOff_ToRealRC);
}

void DJIonboardSDK::on_btn_coreSet_clicked()
{
    uint8_t data[16];

    data[12] = 0;
    data[13] = 0;
    data[14] = 0;
    data[15] = 0;
    api->setBroadcastFreq(data);
}

void DJIonboardSDK::on_btn_vrc_down_pressed()
{

}

void DJIonboardSDK::on_btn_vrc_up_pressed()
{

}

void DJIonboardSDK::on_btn_vrc_left_pressed()
{

}

void DJIonboardSDK::on_btn_vrc_right_pressed()
{
}

void DJIonboardSDK::on_btn_vrc_w_pressed()
{
}

void DJIonboardSDK::on_btn_vrc_S_pressed()
{
}

void DJIonboardSDK::on_btn_vrc_A_pressed()
{
}

void DJIonboardSDK::on_btn_vrc_D_pressed()
{
}

void DJIonboardSDK::on_btr_camera_speed_clicked()
{
    ui->hs_camera_yaw->setMinimum(-1800);
    ui->hs_camera_yaw->setMaximum(1800);
    ui->hs_camera_roll->setMinimum(-1800);
    ui->hs_camera_roll->setMaximum(1800);
    ui->hs_camera_pitch->setMinimum(-1800);
    ui->hs_camera_pitch->setMaximum(1800);
    resetCameraAngle();
    ui->gb_cameraFlag->setEnabled(false);
}

void DJIonboardSDK::updateCameraFlag()
{
    ui->lineEdit_cameraFlag->clear();
    ui->lineEdit_cameraFlag->setText(QString::number(camFlag, 16));
}

void DJIonboardSDK::on_btg_cameraAngle(QAbstractButton *button)
{
    camFlag &= 0xFE;
    if (button->text() == "Absolute")
        camFlag |= 0x01;
    updateCameraFlag();
}

void DJIonboardSDK::on_btg_cameraYaw(QAbstractButton *button)
{
    camFlag &= 0xFD;
    if (button->text() == "Disable")
        camFlag |= 0x02;
    updateCameraFlag();
}

void DJIonboardSDK::on_btg_cameraRoll(QAbstractButton *button)
{
    camFlag &= 0xFB;
    if (button->text() == "Disable")
        camFlag |= 0x04;
    updateCameraFlag();
}

void DJIonboardSDK::on_btg_cameraPitch(QAbstractButton *button)
{
    camFlag &= 0xF7;
    if (button->text() == "Disable")
        camFlag |= 0x08;
    updateCameraFlag();
}

void DJIonboardSDK::resetCameraAngle()
{
    ui->hs_camera_yaw->setValue(0);
    ui->hs_camera_roll->setValue(0);
    ui->hs_camera_pitch->setValue(0);
}

void DJIonboardSDK::on_btr_camera_angle_clicked()
{
    ui->hs_camera_yaw->setMinimum(-3200);
    ui->hs_camera_yaw->setMaximum(3200);
    ui->hs_camera_roll->setMinimum(-350);
    ui->hs_camera_roll->setMaximum(350);
    ui->hs_camera_pitch->setMinimum(-900);
    ui->hs_camera_pitch->setMaximum(300);
    ui->gb_cameraFlag->setEnabled(true);
    resetCameraAngle();
}

void DJIonboardSDK::on_btn_cameraRecord_clicked(bool checked)
{
    if (checked)
    {
        ui->btn_cameraRecord->setText("Stop");
        cam->setCamera(Camera::CODE_CAMERA_VIDEO_START);
    }
    else
    {
        ui->btn_cameraRecord->setText("Record");
        cam->setCamera(Camera::CODE_CAMERA_VIDEO_STOP);
    }
}

void DJIonboardSDK::on_btn_cameraShoot_clicked() { cam->setCamera(Camera::CODE_CAMERA_SHOT); }

void DJIonboardSDK::on_btn_camera_send_clicked()
{
    GimbalSpeedData speedData;
    GimbalAngleData angleData;
    if (ui->btg_cameraMode->checkedButton()->text() == "Speed")
    {
        speedData.yaw = ui->hs_camera_yaw->value();
        speedData.roll = ui->hs_camera_roll->value();
        speedData.pitch = ui->hs_camera_pitch->value();
        cam->setGimbalSpeed(&speedData);
    }
    else
    {
        angleData.yaw = ui->hs_camera_yaw->value();
        angleData.roll = ui->hs_camera_roll->value();
        angleData.pitch = ui->hs_camera_pitch->value();
        angleData.mode = camFlag;
        angleData.duration = ui->lineEdit_cameraTime->text().toInt();
        cam->setGimbalAngle(&angleData);
    }
}

void DJIonboardSDK::on_btn_camera_0_clicked() { resetCameraAngle(); }

void DJIonboardSDK::on_btn_camera_8_pressed()
{
    ui->hs_camera_pitch->setValue(ui->hs_camera_pitch->value() + 5);
}

void DJIonboardSDK::on_btn_camera_2_pressed()
{
    ui->hs_camera_pitch->setValue(ui->hs_camera_pitch->value() - 5);
}

void DJIonboardSDK::on_btn_camera_7_pressed()
{
    ui->hs_camera_roll->setValue(ui->hs_camera_roll->value() - 5);
}

void DJIonboardSDK::on_btn_camera_9_pressed()
{
    ui->hs_camera_roll->setValue(ui->hs_camera_roll->value() + 5);
}

void DJIonboardSDK::on_btn_camera_4_pressed()
{
    ui->hs_camera_yaw->setValue(ui->hs_camera_yaw->value() - 5);
}

void DJIonboardSDK::on_btn_camera_6_pressed()
{
    ui->hs_camera_yaw->setValue(ui->hs_camera_yaw->value() + 5);
}

void DJIonboardSDK::on_tmr_Camera_autosend() { on_btn_camera_send_clicked(); }

void DJIonboardSDK::on_cb_camera_send_clicked(bool checked)
{
    if (checked)
        cameraSend->start();
    else
        cameraSend->stop();
}

void DJIonboardSDK::on_btn_webLoad_clicked() { webView->load(QUrl("")); }

void DJIonboardSDK::upDateTime()
{
#ifdef SDK_VERSION_2_3
    ui->le_coreTimeStamp->setText(QString::number(api->getTime()));
#else
    ui->le_coreTimeStamp->setText(QString::number(api->getTime().time));
    ui->le_coreNanoStamp->setText(QString::number(api->getTime().nanoTime));
    ui->le_coreSyncFlag->setText(QString::number(api->getTime().syncFlag));
#endif
}

void DJIonboardSDK::upDateCapacity()
{
    ui->le_coreCapacity->setText(QString::number(api->getBatteryCapacity()));
}

void DJIonboardSDK::on_btn_coreRead_clicked()
{
    upDateTime();
    upDateCapacity();
    upDateFlightStatus();
    updateControlDevice();
}

void DJIonboardSDK::upDateFlightStatus()
{
    ui->le_coreFlightStatus->setText(QString::number((api->getFlightStatus())));
}

void DJIonboardSDK::updateControlDevice()
{
    ui->le_coreControlDevice->setText(QString::number((api->getCtrlInfo().device)));
}

void DJIonboardSDK::on_tmr_Broadcast()
{
    //! @note this function cost too much time to run.
    //! it is better run outside the broadcastCallback.
    if (ui->cb_coreTimeStamp->isChecked())
        upDateTime();
    if (ui->cb_coreCapacity->isChecked())
        upDateCapacity();
    if (ui->cb_coreFlightStatus->isChecked())
        upDateFlightStatus();
    if (ui->cb_coreControlDevice->isChecked())
        updateControlDevice();
    if (ui->cb_cameraYaw->isChecked())
        updateCameraYaw();
    if (ui->cb_cameraRoll->isChecked())
        updateCameraRoll();
    if (ui->cb_cameraPitch->isChecked())
        updateCameraPitch();
    if (ui->cb_FlightA->isChecked())
        updateFlightAcc();
    if (ui->cb_FlightP->isChecked())
        updateFlightPal();
    if (ui->cb_FlightM->isChecked())
        updateFlightMagnet();
    if (ui->cb_FlightQ->isChecked())
        updateFlightQuaternion();
    if (ui->cb_FlightV->isChecked())
        updateFlightVelocity();
    if (ui->cb_FlightPos->isChecked())
        updateFlightPossition();
}

void DJIonboardSDK::updateCameraYaw()
{
    ui->le_cameraYaw->setText(QString::number(cam->getYaw()));
    if (cam->isYawLimit())
        ui->cb_cameraYawLimit->setChecked(true);
    else
        ui->cb_cameraYawLimit->setChecked(false);
}

void DJIonboardSDK::updateCameraRoll()
{
    ui->le_cameraRoll->setText(QString::number(cam->getRoll()));
    if (cam->isRollLimit())
        ui->cb_cameraRollLimit->setChecked(true);
    else
        ui->cb_cameraRollLimit->setChecked(false);
}

void DJIonboardSDK::updateCameraPitch()
{
    ui->le_cameraPitch->setText(QString::number(cam->getPitch()));
    if (cam->isPitchLimit())
        ui->cb_cameraPitchLimit->setChecked(true);
    else
        ui->cb_cameraPitchLimit->setChecked(false);
}

void DJIonboardSDK::updateVirturalRCData()
{
}

void DJIonboardSDK::updateFlightAcc()
{
    ui->le_Flight_accx->setText(QString::number(flight->getAcceleration().x));
    ui->le_Flight_accy->setText(QString::number(flight->getAcceleration().y));
    ui->le_Flight_accz->setText(QString::number(flight->getAcceleration().z));
}

void DJIonboardSDK::updateFlightPal()
{
    ui->le_Flight_palx->setText(QString::number(flight->getPalstance().x));
    ui->le_Flight_paly->setText(QString::number(flight->getPalstance().y));
    ui->le_Flight_palz->setText(QString::number(flight->getPalstance().z));
}

void DJIonboardSDK::updateFlightMagnet()
{
    ui->le_Flight_magx->setText(QString::number(flight->getMagnet().x));
    ui->le_Flight_magy->setText(QString::number(flight->getMagnet().y));
    ui->le_Flight_magz->setText(QString::number(flight->getMagnet().z));
}

void DJIonboardSDK::updateFlightQuaternion()
{
    ui->le_Flight_Q0->setText(QString::number(flight->getQuaternion().q0));
    ui->le_Flight_Q1->setText(QString::number(flight->getQuaternion().q1));
    ui->le_Flight_Q2->setText(QString::number(flight->getQuaternion().q2));
    ui->le_Flight_Q3->setText(QString::number(flight->getQuaternion().q3));
}

void DJIonboardSDK::updateFlightVelocity()
{
    ui->le_Flight_Vx->setText(QString::number(flight->getVelocity().x));
    ui->le_Flight_Vy->setText(QString::number(flight->getVelocity().y));
    ui->le_Flight_Vz->setText(QString::number(flight->getVelocity().z));
    ui->le_Flight_VS->setText(QString::number(flight->getVelocity().sensorID));
    ui->le_Flight_VH->setText(QString::number(flight->getVelocity().health));
}

void DJIonboardSDK::updateFlightPossition()
{
    ui->le_Flight_PosH->setText(QString::number(flight->getPossition().height));
    ui->le_Flight_PosLa->setText(QString::number(flight->getPossition().latitude));
    ui->le_Flight_PosLo->setText(QString::number(flight->getPossition().longitude));
    ui->le_Flight_PosAl->setText(QString::number(flight->getPossition().altitude));
    ui->le_Flight_PosHealth->setText(QString::number(flight->getPossition().health));
}

QStandardItemModel *DJIonboardSDK::initAction()
{
    QStandardItemModel *action = new QStandardItemModel();
    action->setHorizontalHeaderItem(0, new QStandardItem(QObject::tr("No.")));
    action->setHorizontalHeaderItem(1, new QStandardItem(QObject::tr("Index")));
    action->setHorizontalHeaderItem(2, new QStandardItem(QObject::tr("Type")));
    action->setHorizontalHeaderItem(3, new QStandardItem(QObject::tr("Data")));
    return action;
}

void DJIonboardSDK::on_btn_vrcRead_clicked() { updateVirturalRCData(); }
void DJIonboardSDK::on_btn_FlightAcc_clicked() { updateFlightAcc(); }
void DJIonboardSDK::on_btn_FlightPal_clicked() { updateFlightPal(); }
void DJIonboardSDK::on_btn_FlightMag_clicked() { updateFlightMagnet(); }
void DJIonboardSDK::on_btn_FlightVel_clicked() { updateFlightVelocity(); }
void DJIonboardSDK::on_btn_FlightQua_clicked() { updateFlightQuaternion(); }
void DJIonboardSDK::on_btn_FlightPos_clicked() { updateFlightPossition(); }

void DJIonboardSDK::on_btn_cameraRead_clicked()
{
    updateCameraYaw();
    updateCameraRoll();
    updateCameraPitch();
}

void DJIonboardSDK::on_btn_hotPoint_start_clicked()
{
}

void DJIonboardSDK::on_btn_hp_setPal_clicked()
{
}

void DJIonboardSDK::on_btn_hotPoint_current_clicked()
{
}

void DJIonboardSDK::on_cb_mission_hp_clicked(bool checked) { api->setHotPointData(checked); }

void DJIonboardSDK::on_cb_mission_wp_clicked(bool checked) { api->setWayPointData(checked); }

void DJIonboardSDK::on_cb_mission_follow_clicked(bool checked) { api->setFollowData(checked); }

void DJIonboardSDK::on_btn_hotPoint_stop_clicked() { hp->stop(); }

void DJIonboardSDK::on_btn_hp_setRadius_clicked()
{
}

void DJIonboardSDK::on_btn_hp_setYaw_clicked()
{
}

void DJIonboardSDK::on_btn_hp_data_clicked() { hp->readData(hotpintReadCallback, this); }

void DJIonboardSDK::on_btn_follow_current_clicked()
{
}

void DJIonboardSDK::on_hs_follow_la_valueChanged(int value)
{
}

void DJIonboardSDK::on_hs_follow_lo_valueChanged(int value)
{
}

void DJIonboardSDK::on_hs_follow_al_valueChanged(int value)
{
}

void DJIonboardSDK::on_hs_follow_an_valueChanged(int value)
{
}

void DJIonboardSDK::on_btn_follow_update_clicked() { on_tmr_follow_send(); }

void DJIonboardSDK::on_btn_follow_stop_clicked()
{
    follow->stop();
    followSend->stop();
}

void DJIonboardSDK::on_btn_follow_pause_clicked(bool checked)
{
}

void DJIonboardSDK::on_btn_follow_start_clicked()
{
    follow->start();
    followSend->start();
}

void DJIonboardSDK::on_tmr_follow_send()
{
}

void DJIonboardSDK::wpAddPoint()
{
    int number = waypointData->rowCount();
    waypointData->setItem(number, 0, new QStandardItem(QString::number(number)));
    waypointData->setItem(number, 1,
                          new QStandardItem(QString::number(flight->getPossition().latitude)));
    waypointData->setItem(number, 2,
                          new QStandardItem(QString::number(flight->getPossition().longitude)));
    waypointData->setItem(number, 3,
                          new QStandardItem(QString::number(flight->getPossition().altitude)));
    waypointData->setItem(number, 4, new QStandardItem("not available now"));
    waypointData->setItem(number, 5, new QStandardItem("0"));
    waypointData->setItem(number, 6, new QStandardItem("0"));
    waypointData->setItem(number, 7, new QStandardItem("Clockwise"));
    waypointData->setItem(number, 8, new QStandardItem("0"));

    actionData.append(initAction());
}

void DJIonboardSDK::on_btn_waypoint_init_clicked()
{
    WayPointInitData data = wp->getInfo();
    //! @note these are two different way to offer a same init.
    // wp->setInfo(data);
    // wp->init();
    wp->init(&data);
}

void DJIonboardSDK::on_waypoint_data_changed(const QModelIndex &topLeft __UNUSED,
                                             const QModelIndex &bottomRight __UNUSED,
                                             const QVector<int> &roles __UNUSED)
{
    //! @todo waypoint data modify
    //    API_LOG(api->getDriver(), STATUS_LOG, "c: %d r: %d %s", bottomRight.column(),
    //          bottomRight.row(), topLeft.data().toByteArray().data());
    //    API_LOG(api->getDriver(), STATUS_LOG, "c: %d r: %d %lf", bottomRight.column(),
    //        bottomRight.row(), topLeft.data().toDouble());
}

void DJIonboardSDK::wpRemovePoint()
{
}

void DJIonboardSDK::on_cb_waypoint_point_currentIndexChanged(int index)
{
    if (index != 0)
    {
        if (actionData.length() >= (index - 1))
        {
            currentAction = actionData[index - 1];
        }
    }
    else
    {
        currentAction = nullAction;
    }
}

void DJIonboardSDK::on_btn_log_clean_clicked() { ui->tb_display->clear(); }

void DJIonboardSDK::on_btn_log_save_clicked()
{
    QString data = QFileDialog::getSaveFileName(this, "Save", QString(), "Text (*.txt)");
    API_LOG(api->getDriver(), STATUS_LOG, "%s", data.toLatin1().data());
}

void DJIonboardSDK::on_btn_waypoint_action_clicked()
{
    if (currentAction != nullAction)
    {
        int number = currentAction->rowCount();
    }
}

void DJIonboardSDK::on_btn_waypoint_reset_clicked()
{
    while (actionData.length() != 0)
    {
        on_btn_waypoint_remove_clicked();
    }
}

void DJIonboardSDK::on_btn_waypoint_removeAction_clicked()
{
    //! @todo implement removeAction
}

void DJIonboardSDK::on_btn_core_setSync_clicked()
{
}

void DJIonboardSDK::on_le_waypoint_number_editingFinished()
{
}

void DJIonboardSDK::on_btn_waypoint_viewPoint_clicked()
{
}

void DJIonboardSDK::on_btn_wp_ivset_clicked()
{
}

void DJIonboardSDK::on_btn_wp_ivRead_clicked() { wp->readIdleVelocity(); }

void DJIonboardSDK::on_btn_waypoint_add_clicked()
{
    wpAddPoint();
}

void DJIonboardSDK::on_btn_waypoint_remove_clicked()
{
    wpRemovePoint();
}

void DJIonboardSDK::on_btn_wp_pr_clicked(bool checked)
{
}

void DJIonboardSDK::on_le_wp_exec_editingFinished()
{
    //! @note the range of exectimes is 1-255, 0xFF means infinite loop
}

void DJIonboardSDK::on_btn_wp_start_stop_clicked(bool checked)
{
    if (checked)
    {
    }
    else
    {
    }
}

void DJIonboardSDK::on_btn_wp_loadOne_clicked()
{
}
void DJIonboardSDK::on_btn_wp_loadAll_clicked()
{
}

void DJIonboardSDK::on_btn_hp_pause_clicked(bool checked)
{
}

void DJIonboardSDK::on_cb_core_mechine_activated(int index)
{
#ifndef SDK_VERSION_2_3
    if (!index)
        api->setVersion(versionM100_31);
    else
        api->setVersion(versionA3_31);
#else
    ui->cb_core_mechine->setEnabled(false);
#endif
}

void DJIonboardSDK::on_btn_script_run_clicked()
{
    //! @todo remap stdin and stdout
    //!

}

void DJIonboardSDK::on_tabWidget_currentChanged(int index)
{

}

void DJIonboardSDK::on_actionOpen_O_triggered()
{
    qDebug()<<"hello";
}
