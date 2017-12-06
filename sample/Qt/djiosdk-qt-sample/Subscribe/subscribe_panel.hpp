/*! @file subscribe_panel.hpp
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

#ifndef SUBSCRIBEPANNEL_H
#define SUBSCRIBEPANNEL_H

#include <QTableWidgetItem>
#include <QWidget>
#include <dji_vehicle.hpp>

namespace Ui
{
class SubscribePanel;
}

class SubscribePanel : public QWidget
{
  Q_OBJECT

public:
  int freqEnum[7] = { 0, 1, 10, 50, 100, 200, 400 };

public:
  explicit SubscribePanel(QWidget*            parent     = 0,
                          DJI::OSDK::Vehicle* vehiclePtr = 0);
  ~SubscribePanel();

  void display(DJI::OSDK::Telemetry::TopicName topicName, uint32_t id);

private slots:
  void on_btn_match_clicked();
  //  void on_btn_reset_clicked();
  //  void on_btn_subscribe_clicked();
  //  void on_btn_remove_clicked();
  //  void on_btn_pause_clicked();
  //  void on_btn_resume_clicked();
  void on_tableWidget_itemChanged(QTableWidgetItem* item);

  void on_startPkg0_clicked();

  void on_stopPkg0_clicked();

  void on_startPkg1_clicked();

  void on_stopPkg1_clicked();

  void on_startPkg2_clicked();

  void on_stopPkg2_clicked();

  void on_startPkg3_clicked();

  void on_stopPkg3_clicked();

  void on_stopPkg4_clicked();

  void on_startPkg4_clicked();

public:
  static void pkg0UnpackCallback(DJI::OSDK::Vehicle*      vehiclePtr,
                                 DJI::OSDK::RecvContainer rcvContainer,
                                 DJI::OSDK::UserData      userData);
  static void pkg1UnpackCallback(DJI::OSDK::Vehicle*      vehiclePtr,
                                 DJI::OSDK::RecvContainer rcvContainer,
                                 DJI::OSDK::UserData      userData);
  static void pkg2UnpackCallback(DJI::OSDK::Vehicle*      vehiclePtr,
                                 DJI::OSDK::RecvContainer rcvContainer,
                                 DJI::OSDK::UserData      userData);
  static void pkg3UnpackCallback(DJI::OSDK::Vehicle*      vehiclePtr,
                                 DJI::OSDK::RecvContainer rcvContainer,
                                 DJI::OSDK::UserData      userData);
  static void pkg4UnpackCallback(DJI::OSDK::Vehicle*      vehiclePtr,
                                 DJI::OSDK::RecvContainer rcvContainer,
                                 DJI::OSDK::UserData      userData);

private:
  Ui::SubscribePanel* ui;
  QVector<int>        pkg0Indices;
  QVector<int>        pkg1Indices;
  QVector<int>        pkg2Indices;
  QVector<int>        pkg3Indices;
  QVector<int>        pkg4Indices;

  DJI::OSDK::Vehicle* vehicle;
};

#endif // SUBSCRIBEPANNEL_H
