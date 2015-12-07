#include "djionboardsdk.h"
#include "ui_djionboardsdk.h"

#include <QFile>

DJIonboardSDK *DJIonboardSDK::sdk = 0;

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
    cam = new Camera(api);

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
    autoSend->setInterval(50); // 20Hz
    connect(autoSend, SIGNAL(timeout()), this, SLOT(autosend()));
    //! @endcode init flight

    //! @code init camera
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
    //! @endcode

    //! @code init virtual RC
    vrcSend = new QTimer();
    vrcSend->setInterval(400); // 2.5Hz
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
    unsigned short ack_data = 0xFFFF;
    unsigned char data = 0x1;

    if (header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,
               ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
    }
    else
        API_ERROR("ACK is exception,seesion id %d,sequence %d\n",
                  header->sessionID, header->sequence_number);

    switch (ack_data)
    {
        case 0x0000:
            if (sdk)
                sdk->ui->btn_coreSetControl->setText("Swtich to mod F");
            else
                API_ERROR("known SDK pointer 0.");
            break;
        case 0x0001:
            if (sdk)
                sdk->ui->btn_coreSetControl->setText("Obtain Control");
            else
                API_ERROR("known SDK pointer 0.");
            break;
        case 0x0002:
            if (sdk)
                sdk->ui->btn_coreSetControl->setText("Release Control");
            else
                API_ERROR("known SDK pointer 0.");
            break;
        case 0x0003:
            This->send(2, 1, SET_CONTROL, API_CTRL_MANAGEMENT, &data, 1,
                       DJIonboardSDK::setControlCallback, 500, 2);
            break;
        case 0x0004:
            data = 0;
            This->send(2, 1, SET_CONTROL, API_CTRL_MANAGEMENT, &data, 1,
                       DJIonboardSDK::setControlCallback, 500, 2);
            break;
    }
    //! @note For debug, all functional print is moving to this function,
    //! default API callback is not necessary.
    //    CoreAPI::setControlCallback(This, header);
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
    data.app_bundle_id[0] = data.app_bundle_id[1] =
        0x12; //! @note for ios verification
    *key = ui->lineEdit_Key->text().toLocal8Bit();
    data.app_key = key->data(); //! @warning memory leak fixme
    api->activate(&data);
}

void DJIonboardSDK::on_btn_coreVersion_clicked() { api->getVersion(); }

void DJIonboardSDK::on_btn_coreSetControl_clicked()
{
    if (ui->btn_coreSetControl->text() == "Release Control")
        api->setControl(false, DJIonboardSDK::setControlCallback);
    else
        api->setControl(true, DJIonboardSDK::setControlCallback);
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

void DJIonboardSDK::on_tmr_virtualRC_autosend()
{
    VirtualRCData data;
    data.throttle = ui->slider_VRC_LV->value();
    data.roll = ui->slider_VRC_RH->value();
    data.pitch = ui->slider_VRC_RV->value();
    data.yaw = ui->slider_VRC_LH->value();

    if (ui->btg_vrcMode->checkedButton()->text() == "F")
        data.mode = 496;
    else if (ui->btg_vrcMode->checkedButton()->text() == "A")
        data.mode = 1024;
    else
        data.mode = 1552;

    if (ui->btg_vrcGear->checkedButton()->text() == "Up")
        data.gear = 1684;
    else
        data.gear = 1324;

    vrc->sendData(data);
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
    //! @todo write camera operation
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
    vrc->sentContorl(true, VirtualRC::CutOff_ToRealRC);
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

void DJIonboardSDK::on_btn_vrc_down_pressed()
{
    ui->slider_VRC_RV->setValue(ui->slider_VRC_RV->value() - 10);
}

void DJIonboardSDK::on_btn_vrc_up_pressed()
{
    ui->slider_VRC_RV->setValue(ui->slider_VRC_RV->value() + 10);
}

void DJIonboardSDK::on_btn_vrc_left_pressed()
{
    ui->slider_VRC_RH->setValue(ui->slider_VRC_RH->value() - 10);
}

void DJIonboardSDK::on_btn_vrc_right_pressed()
{
    ui->slider_VRC_RH->setValue(ui->slider_VRC_RH->value() + 10);
}

void DJIonboardSDK::on_btn_vrc_w_pressed()
{
    ui->slider_VRC_LV->setValue(ui->slider_VRC_LV->value() + 10);
}

void DJIonboardSDK::on_btn_vrc_S_pressed()
{
    ui->slider_VRC_LV->setValue(ui->slider_VRC_LV->value() - 10);
}

void DJIonboardSDK::on_btn_vrc_A_pressed()
{
    ui->slider_VRC_LH->setValue(ui->slider_VRC_LH->value() - 10);
}

void DJIonboardSDK::on_btn_vrc_D_pressed()
{
    ui->slider_VRC_LH->setValue(ui->slider_VRC_LH->value() + 10);
}

void DJIonboardSDK::on_btr_camera_speed_clicked()
{
    ui->hs_camera_yaw->setValue(0);
    ui->hs_camera_yaw->setMinimum(-1800);
    ui->hs_camera_yaw->setMaximum(1800);
    ui->hs_camera_roll->setValue(0);
    ui->hs_camera_roll->setMinimum(-1800);
    ui->hs_camera_roll->setMaximum(1800);
    ui->hs_camera_pitch->setValue(0);
    ui->hs_camera_pitch->setMinimum(-1800);
    ui->hs_camera_pitch->setMaximum(1800);
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

void DJIonboardSDK::on_btr_camera_angle_clicked()
{
    ui->hs_camera_yaw->setValue(0);
    ui->hs_camera_yaw->setMinimum(-3200);
    ui->hs_camera_yaw->setMaximum(3200);
    ui->hs_camera_roll->setValue(0);
    ui->hs_camera_roll->setMinimum(-350);
    ui->hs_camera_roll->setMaximum(350);
    ui->hs_camera_pitch->setValue(0);
    ui->hs_camera_pitch->setMinimum(-900);
    ui->hs_camera_pitch->setMaximum(300);
    ui->gb_cameraFlag->setEnabled(true);
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

void DJIonboardSDK::on_btn_cameraShoot_clicked()
{
    cam->setCamera(Camera::CODE_CAMERA_SHOT);
}

void DJIonboardSDK::on_btn_camera_send_clicked()
{
    GimbalSpeedData speedData;
    GimbalAngleData angleData;
    if (ui->btg_cameraMode->checkedButton()->text() == "Speed")
    {
        qDebug() << "speed";
        speedData.yaw_angle_rate = ui->hs_camera_yaw->value();
        speedData.roll_angle_rate = ui->hs_camera_roll->value();
        speedData.pitch_angle_rate = ui->hs_camera_pitch->value();
        qDebug() << speedData.yaw_angle_rate << speedData.roll_angle_rate
                 << speedData.pitch_angle_rate;
        cam->setGimbalSpeed(&speedData);
    }
    else
    {
        qDebug() << "angle";
        angleData.yaw_angle = ui->hs_camera_yaw->value();
        angleData.roll_angle = ui->hs_camera_roll->value();
        angleData.pitch_angle = ui->hs_camera_pitch->value();
        angleData.ctrl_byte = camFlag;
        angleData.duration = 10;
        cam->setGimbalAngle(&angleData);
    }
}
