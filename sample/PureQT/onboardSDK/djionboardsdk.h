#ifndef DJIONBOARDSDK_H
#define DJIONBOARDSDK_H

#include <QMainWindow>
#include <QAbstractButton>
#include "QonboardSDK.h"

using namespace DJI::onboardSDK;

namespace Ui
{
class DJIonboardSDK;
}

class DJIonboardSDK : public QMainWindow
{
    Q_OBJECT

  public:
    explicit DJIonboardSDK(QWidget *parent = 0);
    ~DJIonboardSDK();

    void updateFlightFlag();
private:
    void setBaudrate();
    void setPort();
    void openPort();
    void closePort();
    void refreshPort();

  protected:
    void closeEvent(QCloseEvent *);
  private slots:
    void on_btn_portRefresh_clicked();
    void on_btn_portOpen_clicked();
    void on_comboBox_portName_currentIndexChanged(int index);

    void on_btn_coreActive_clicked();
    void on_btn_coreVersion_clicked();
    void on_btn_coreSetControl_clicked();

    void on_btn_VRC_resetAll_clicked();
    void on_btn_VRC_resetLeft_clicked();
    void on_btn_VRC_resetRight_clicked();

    void on_btg_flight_HL(QAbstractButton* button);
    void on_btg_flight_VL(QAbstractButton* button);
    void on_btg_flight_YL(QAbstractButton* button);
    void on_btg_flight_CL(QAbstractButton* button);
    void on_btg_flight_SM(QAbstractButton* button);

    void on_btn_camera_up_clicked();

private:
    Ui::DJIonboardSDK *ui;
    CoreAPI *api;
    QHardDriver *driver;
    QSerialPort *port;

    APIThread *send;
    APIThread *read;

    QByteArray *key;

    uint8_t flightFlag;
};

#endif // DJIONBOARDSDK_H
