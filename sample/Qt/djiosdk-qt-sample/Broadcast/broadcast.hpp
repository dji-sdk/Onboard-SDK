/*! @file broadcast.hpp
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

#ifndef BROADCAST_H
#define BROADCAST_H

#include <QTimer>
#include <QWidget>
#include <dji_vehicle.hpp>

namespace Ui
{
class Broadcast;
}

class Broadcast : public QWidget
{
  Q_OBJECT

public:
  explicit Broadcast(QWidget* parent = 0, DJI::OSDK::Vehicle* vehicle = 0);
  ~Broadcast();

private:
  void updateFlightAcc();
  void updateFlightPal();
  void updateFlightMagnet();
  void updateFlightVelocity();
  void updateFlightPosition();
  void updateFlightQuaternion();
  void updateVirturalRCData();
  void updateCameraPitch();
  void updateCameraRoll();
  void updateCameraYaw();
  void updateGPS();
  void updateRTK();
  void updateTime();
  void updateCapacity();
  void updateFlightStatus();
  void updateControlDevice();
  void manageDisplayEnabling(uint8_t* curFreqSettings);

private slots:
  void updateDisplay();

  void on_pushButtonBroadcastFreqSet_clicked();

private:
  Ui::Broadcast*      ui;
  DJI::OSDK::Vehicle* vehicle;
  QTimer*             broadcastTimer;
};

#endif // BROADCAST_H
