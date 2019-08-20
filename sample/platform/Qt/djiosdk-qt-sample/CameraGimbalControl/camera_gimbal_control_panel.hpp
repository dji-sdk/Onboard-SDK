/*! @file camera_gimbal_control_panel.hpp
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

#ifndef QGIMBAL_H
#define QGIMBAL_H

#include <QAbstractButton>
#include <QWidget>
#include <dji_vehicle.hpp>

namespace Ui
{
class CameraGimbalControl;
}

class CameraGimbalControl : public QWidget
{
  Q_OBJECT

public:
  explicit CameraGimbalControl(QWidget*            parent  = 0,
                               DJI::OSDK::Vehicle* vehicle = 0);
  ~CameraGimbalControl();

private:
  void handleRPYRanges(QString modeString);

private slots:
  void on_rollCheckBox_stateChanged(int arg1);
  void on_pitchCheckBox_stateChanged(int arg1);
  void on_yawCheckBox_stateChanged(int arg1);
  void on_controlModeButtonGroup_buttonClicked(QAbstractButton*);
  void on_angleControlButtonGroup_buttonClicked(QAbstractButton*);

  void on_pitchControlSpinBox_valueChanged(double arg1);
  void on_pitchControlSlider_valueChanged(int value);
  void on_rollControlSpinBox_valueChanged(double arg1);
  void on_rollControlSlider_valueChanged(int value);
  void on_yawControlSpinBox_valueChanged(double arg1);
  void on_yawControlSlider_valueChanged(int value);

  void on_sendGimbalCommand_clicked();

  void on_photoButton_clicked();

  void on_videoButton_clicked();

private:
  Ui::CameraGimbalControl*     ui;
  DJI::OSDK::Vehicle*          vehicle;
  DJI::OSDK::Gimbal::AngleData angleData;
  DJI::OSDK::Gimbal::SpeedData speedData;
};

#endif // QGIMBAL_H
