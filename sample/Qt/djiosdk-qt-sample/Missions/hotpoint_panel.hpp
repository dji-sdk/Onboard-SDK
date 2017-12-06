/*! @file hotpoint_panel.hpp
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

#ifndef HOTPOINT_PANEL_H
#define HOTPOINT_PANEL_H

#include <QFrame>
#include <dji_vehicle.hpp>

#define DEG2RAD 0.01745329252

namespace Ui
{
class HotpointPanel;
}

class HotpointPanel : public QFrame
{
  Q_OBJECT

public:
  explicit HotpointPanel(QWidget* parent = 0, DJI::OSDK::Vehicle* vehicle = 0);
  ~HotpointPanel();

public:
  static void hotpointReadCallback(DJI::OSDK::Vehicle*      This,
                                   DJI::OSDK::RecvContainer recvFrame,
                                   DJI::OSDK::UserData      userData);

private slots:
  void on_btn_hotPoint_start_clicked();
  void on_btn_hotPoint_stop_clicked();
  void on_btn_hotPoint_current_clicked();
  void on_btn_hp_pause_clicked(bool checked);
  void on_btn_hp_setPal_clicked();
  void on_btn_hp_setRadius_clicked();
  void on_btn_hp_setYaw_clicked();
  void on_btn_hp_data_clicked();

private:
  DJI::OSDK::Vehicle* vehicle;
  Ui::HotpointPanel*  ui;
};

#endif // HOTPOINT_PANEL_H
