/*! @file main.cpp
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

#include "flight_control_panel.hpp"
#include "qtosdk.hpp"
#include <QApplication>
#include <QDebug>
#include <QFile>
#include <QMetaType>
#include <stdio.h>

// Declarations
void initVehicleConnections(qtOsdk* sdk);
QByteArray readTextFile(const QString& file_path);

int
main(int argc, char* argv[])
{

  QString      style_sheet = readTextFile(":/stylesheets/material-blue.qss");
  QApplication a(argc, argv);
  a.setStyleSheet(style_sheet);

  qRegisterMetaType<QVector<int> >("QVector<int>");
  qtOsdk w;
  w.setWindowTitle("DJI Onboard SDK");
  w.setWindowIcon(QIcon(":/images/dji_logo_gray.png"));
  initVehicleConnections(&w);
  w.show();
  return a.exec();
}

void
initVehicleConnections(qtOsdk* sdk)
{
  QObject::connect(sdk, SIGNAL(changeControlAuthorityStatus(QString)), sdk,
                   SLOT(ctrlStatusChanged(QString)));
  QObject::connect(sdk, SIGNAL(changeInitButton(QString, bool)), sdk,
                   SLOT(initFinished(QString, bool)));
  QObject::connect(sdk, SIGNAL(changeActivateButton(QString, bool)), sdk,
                   SLOT(activateFinished(QString, bool)));
}

QByteArray
readTextFile(const QString& file_path)
{
  QFile      input_file(file_path);
  QByteArray input_data;

  if (input_file.open(QIODevice::Text | QIODevice::Unbuffered |
                      QIODevice::ReadOnly))
  {
    input_data = input_file.readAll();
    input_file.close();
    return input_data;
  }
  else
  {
    return QByteArray();
  }
}
