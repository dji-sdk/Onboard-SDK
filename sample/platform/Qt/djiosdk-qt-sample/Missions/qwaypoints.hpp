/*! @file qwaypoints.hpp
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

#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <QStandardItemModel>

#include "dji_vehicle.hpp"
#include "sdk_widgets.hpp"
#include <QWidget>

namespace Ui
{
class MissionWidget;
}

class QWaypoints : public QWidget
{
  Q_OBJECT

public:
  explicit QWaypoints(QWidget* parent = 0, DJI::OSDK::Vehicle* vehicle = 0);
  ~QWaypoints();

  void initWayPoint();
  void wpAddPoint();
  void wpRemovePoint();

  Ui::MissionWidget* getMissionUi()
  {
    return this->ui;
  }
private slots:
  void on_btn_waypoint_init_clicked();

  void on_cb_waypoint_point_currentIndexChanged(int index);
  void on_le_waypoint_number_editingFinished();
  void on_btn_waypoint_action_clicked();
  void on_btn_waypoint_reset_clicked();
  void on_btn_waypoint_removeAction_clicked();
  void on_btn_waypoint_viewPoint_clicked();
  void on_btn_wp_ivset_clicked();
  void on_btn_wp_ivRead_clicked();
  void on_btn_waypoint_add_clicked();
  void on_btn_waypoint_remove_clicked();

  void on_btn_wp_pr_clicked(bool checked);
  void on_le_wp_exec_editingFinished();
  void on_btn_wp_loadAll_clicked();
  void on_btn_wp_start_stop_clicked(bool checked);
  void on_btn_wp_loadOne_clicked();
  void wpDataChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight,
                     const QVector<int>& roles);

  //  void on_btn_webTool_clicked(bool checked);

  void on_btn_AbortWaypoint_clicked();

private:
  QStandardItemModel* initAction();

  Ui::MissionWidget* ui;

  DJI::OSDK::WayPointSettings      wayPointDataTmp;
  DJI::OSDK::WayPointInitSettings* wpInitSettings;

  DJI::OSDK::Vehicle*         vehicle;
  QStandardItemModel*         waypointData;
  QStandardItemModel*         currentAction;
  QStandardItemModel*         nullAction;
  QStandardItemModel*         persistentActionPtr;
  QList<QStandardItemModel*>* actionData;
};

#endif // WAYPOINTS_H
