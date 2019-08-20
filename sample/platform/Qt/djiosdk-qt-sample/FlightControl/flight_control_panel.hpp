/*! @file flight_control_panel.hpp
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

#ifndef CONTROLPANNEL_H
#define CONTROLPANNEL_H

#include "dji_vehicle.hpp"
#include <QAbstractButton>
#include <QWidget>

namespace Ui
{
class FlightControlPanel;
}

class FlightControlPanel : public QWidget
{
  Q_OBJECT

public:
  explicit FlightControlPanel(QWidget*            parent  = 0,
                              DJI::OSDK::Vehicle* vehicle = 0);
  ~FlightControlPanel();

  static void actionCallback(DJI::OSDK::Vehicle*      vehiclePtr,
                             DJI::OSDK::RecvContainer recvFrame,
                             DJI::OSDK::UserData      userData);

private:
  void handleXYRanges(double minXY, double maxXY);
  void handleZRanges(double minZ, double maxZ);
  void handleYawRanges(double minYaw, double maxYaw);

private slots:
  void on_btn_flight_runCommand_clicked();

  void on_buttonGroupHorizontal_buttonClicked(QAbstractButton* button);
  void on_buttonGroupVertical_buttonClicked(QAbstractButton* button);
  void on_buttonGroupYaw_buttonClicked(QAbstractButton* button);
  void on_buttonGroupFrame_buttonClicked(QAbstractButton* button);
  void on_buttonGroupStable_buttonClicked(QAbstractButton* button);

  void on_xControlSpinBox_valueChanged(double arg1);
  void on_xControlSlider_valueChanged(int value);

  void on_yControlSpinBox_valueChanged(double arg1);
  void on_yControlSlider_valueChanged(int value);

  void on_zControlSpinBox_valueChanged(double arg1);
  void on_zControlSlider_valueChanged(int value);

  void on_yawControlSpinBox_valueChanged(double arg1);
  void on_yawControlSlider_valueChanged(int value);

  void on_btn_flight_send_clicked();

  void commandStatusChanged(QString);

  void on_cb_command_activated(const QString& arg1);

signals:
  void changeCommandStatus(QString textToDisplay);

private:
  Ui::FlightControlPanel*      ui;
  DJI::OSDK::Vehicle*          vehicle;
  DJI::OSDK::Control::CtrlData command;
};

#endif // CONTROLPANNEL_H
