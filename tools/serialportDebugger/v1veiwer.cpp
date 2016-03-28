#include "v1veiwer.h"
#include "ui_v1veiwer.h"

#include <QSerialPort>
#include <QSerialPortInfo>
#include <Qstringlist>
#include <QStandardItem>

V1Veiwer::V1Veiwer(QWidget* parent) : QMainWindow(parent), ui(new Ui::V1Veiwer)
{
    ui->setupUi(this);
    setupSerialPort();
}

V1Veiwer::~V1Veiwer() { delete ui; }

void V1Veiwer::setupSerialPort()
{
    port = new QSerialPort(this);
    driver = new QHardDriver(port);

    refreshPort();
    setPort();
    setBaudrate();
    openPort();

    startTimer(5);
}

void V1Veiwer::timerEvent(QTimerEvent* event)
{
    //! @note read function
    const size_t bufSize = 2048;
    size_t readLen = 0;
    uint8_t buf[bufSize];
    readLen = driver->readall(buf, bufSize);

    QString str;
    for (int i = 0; i < readLen; ++i)
    {
        str.append(" 0x");
        str.append(QString::number(buf[i], 16));
    }
    if (!str.isEmpty())
        ui->tb_recv->append(str);
}

void V1Veiwer::setBaudrate()
{
    int baudrate = ui->le_portBaudrate->text().toInt();
    driver->setBaudrate(baudrate);
}

void V1Veiwer::setPort() { port->setPortName(ui->cb_portName->currentText()); }

void V1Veiwer::openPort()
{
    driver->init();
    if (port->isOpen())
        ui->btn_portOpen->setText(port->portName().append(" is open"));
    else
        ui->btn_portOpen->setText(port->portName().append(" not exit"));
}

void V1Veiwer::closePort()
{
    port->close();
    if (!port->isOpen())
    {
        ui->btn_portOpen->setText(port->portName().append(" closed"));
    }
}

void V1Veiwer::refreshPort()
{

    ui->cb_portName->clear();
    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
    QStringList list;
    for (int i = 0; i < ports.length(); ++i)
    {
        list.append(ports[i].portName());
    }
    ui->cb_portName->addItems(list);
}
void V1Veiwer::on_btn_portOpen_clicked()
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

void V1Veiwer::on_btn_portRefresh_clicked() { refreshPort(); }

void V1Veiwer::on_cb_portName_currentIndexChanged(int index)
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

void V1Veiwer::on_btn_cvtRR_clicked() { ui->tb_recv->clear(); }

void V1Veiwer::on_btn_cvtRS_clicked() { ui->tb_send->clear(); }

void V1Veiwer::on_btn_send_clicked()
{
    QString data = ui->le_data->text();
    QByteArray sendData;
    if (ui->cb_hex->isChecked())
    {
        QStringList list = data.split(" ");
        bool ok;
        for (int i = 0; i < list.length(); ++i)
        {
            uint8_t cdata = list[i].toInt(&ok, 16);
            sendData.append(cdata);
            ui->tb_send->append(QString("0X").append(QString::number(cdata, 16)));
        }
    }
    else
    {
        sendData = data.toLatin1();
        ui->tb_send->append(sendData);
    }
    driver->send(sendData.data(), sendData.length());
}
