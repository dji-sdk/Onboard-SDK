/*! @file qtosdk.hpp
 *  @version 3.4
 *  @date Dec 2017
 *
 *
 *  @Copyright (c) 2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef QTOSDK_H
#define QTOSDK_H

#include <QFile>
#include <QByteArray>
#include <QMainWindow>
#include <QSerialPortInfo>
#include <broadcast.hpp>
#include <camera_gimbal_control_panel.hpp>
#include <dji_vehicle.hpp>
#include <flight_control_panel.hpp>
#include <hotpoint_panel.hpp>
#include <mfio_panel.hpp>
#include <qwaypoints.hpp>
#include <subscribe_panel.hpp>

namespace Ui
{
class qtOsdk;
}

class qtOsdk : public QMainWindow
{
  Q_OBJECT

public:
  explicit qtOsdk(QWidget* parent = 0);
  void setVehicle(DJI::OSDK::Vehicle* vehiclePtr)
  {
    this->vehicle = vehiclePtr;
  }
  ~qtOsdk();

  void readAppIDKey();

private:
  void refreshPort();
  void initComponents();

private:
  Ui::qtOsdk*          ui;
  DJI::OSDK::Vehicle*  vehicle;
  FlightControlPanel*  flightControl;
  CameraGimbalControl* cameraGimbalControl;
  Broadcast*           broadcast;
  SubscribePanel*      subscribe;
  QWaypoints*          waypoints;
  HotpointPanel*       hotpointPanel;
  MFIOPanel*           mfioPanel;

  // Callbacks
public:
  static void activateCallback(DJI::OSDK::Vehicle*      vehicle,
                               DJI::OSDK::RecvContainer recvFrame,
                               DJI::OSDK::UserData      userData);
  static void setControlCallback(DJI::OSDK::Vehicle*      vehicle,
                                 DJI::OSDK::RecvContainer recvFrame,
                                 DJI::OSDK::UserData      userData);

private slots:

  void on_initVehicle_clicked();
  void on_activateButton_clicked();
  void on_obtainCtrlButton_clicked();
  void ctrlStatusChanged(QString);
  void initFinished(QString initStatus, bool initResult);
  void activateFinished(QString activateStatus, bool activateResult);

  void on_componentTabs_currentChanged(int index);

  void on_portSelection_activated(const QString &arg1);

signals:
  void changeControlAuthorityStatus(QString textToDisplay);
  void changeInitButton(QString textToDisplay, bool success);
  void changeActivateButton(QString textToDisplay, bool success);
};

#endif // QTOSDK_H
