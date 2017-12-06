/*! @file mfio_panel.hpp
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

#ifndef MFIOPANNEL_H
#define MFIOPANNEL_H

#include "dji_vehicle.hpp"
#include <QWidget>

namespace Ui
{
class MFIOPanel;
}

class MFIOPanel : public QWidget
{
  Q_OBJECT

public:
  explicit MFIOPanel(QWidget* parent = 0, DJI::OSDK::Vehicle* vehicle = 0);
  ~MFIOPanel();

  static void getValueCallback(DJI::OSDK::Vehicle*      vehicle,
                               DJI::OSDK::RecvContainer recvFrame,
                               DJI::OSDK::UserData      data);

private slots:
  void on_btn_init_clicked();

  void on_cb_mode_currentIndexChanged(const QString& arg1);

  void on_btn_get_clicked();

  void on_btn_set_clicked();

private:
  Ui::MFIOPanel*      ui;
  DJI::OSDK::Vehicle* vehicle;
  QVector<int>        outputMap;
  QVector<int>        inputMap;
};

#endif // MFIOPANNEL_H
