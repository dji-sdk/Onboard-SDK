#ifndef DJIONBOARDSDK_H
#define DJIONBOARDSDK_H

#include <QMainWindow>
#include <QAbstractButton>
#include <QTimer>
#include <QTextStream>
#ifdef GROUNDSTATION
#include <QWebEngineView>
#endif
#include <QHBoxLayout>
#include <QStandardItemModel>
#include <QList>
#include <QFile>
#include <QComboBox>

#include <fstream>
#ifdef SDK_DEV
#include "uidev.h"
#endif
#include "QonboardSDK.h"

#define   DEG2RAD 0.01745329252

using namespace DJI;
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

  private:
    void setBaudrate();
    void setPort();
    void openPort();
    void closePort();
    void refreshPort();

  protected:
    void closeEvent(QCloseEvent *);
    void timerEvent(QTimerEvent *);

    //! @note callback static class pointer and callback functions
  public:
    static void setControlCallback(CoreAPI *This, Header *header, UserData userData);
    static void activationCallback(CoreAPI *This, Header *header, UserData userData);
    static void hotpintReadCallback(CoreAPI *This, Header *header, UserData userData);

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

    QStandardItemModel *initAction();
    void wpAddPoint();
    void wpRemovePoint();

    void initSDK();
    void initFlight();
    void initCamera();
    void initFollow();
    void initDisplay();
    void initWayPoint();
    void initVirtualRC();

  private slots:
    //! @note too much slots, tired to rename.
    void on_btn_portRefresh_clicked();
    void on_btn_portOpen_clicked();
    void on_comboBox_portName_currentIndexChanged(int index);

    void on_btn_coreSet_clicked();
    void on_btn_coreActive_clicked();
    void on_btn_coreVersion_clicked();
    void on_btn_coreSetControl_clicked();
    void on_btn_core_setSync_clicked();
    void on_btn_coreRead_clicked();
    void on_btn_mobile_clicked();
    void on_tmr_Broadcast();
    void on_cb_core_mechine_activated(int index);

    void upDateTime();
    void upDateCapacity();
    void upDateFlightStatus();
    void updateControlDevice();

    void on_btn_VRC_resetAll_clicked();
    void on_btn_VRC_resetLeft_clicked();
    void on_btn_VRC_resetRight_clicked();
    void on_btn_virtualRC_send_clicked();
    void on_btn_virtualRC_init_clicked();
    void on_btn_vrc_down_pressed();
    void on_btn_vrc_up_pressed();
    void on_tmr_VRC_autosend();
    void on_btn_vrc_w_pressed();
    void on_btn_vrc_A_pressed();
    void on_btn_vrc_S_pressed();
    void on_btn_vrc_D_pressed();
    void on_btn_vrcRead_clicked();
    void on_btn_vrc_left_pressed();
    void on_btn_vrc_right_pressed();

    void on_btn_cameraRead_clicked();
    void on_btn_camera_0_clicked();
    void on_btn_camera_8_pressed();
    void on_btn_camera_2_pressed();
    void on_btn_camera_7_pressed();
    void on_btn_camera_9_pressed();
    void on_btn_camera_4_pressed();
    void on_btn_camera_6_pressed();
    void on_tmr_Camera_autosend();
    void on_cb_camera_send_clicked(bool checked);

    void on_btn_cameraRecord_clicked(bool checked);
    void on_btn_cameraShoot_clicked();
    void on_btn_camera_send_clicked();

    void on_btr_camera_angle_clicked();
    void on_btr_camera_speed_clicked();
    void on_btg_cameraAngle(QAbstractButton *button);
    void on_btg_cameraYaw(QAbstractButton *button);
    void on_btg_cameraRoll(QAbstractButton *button);
    void on_btg_cameraPitch(QAbstractButton *button);
    void resetCameraAngle();
    void updateCameraFlag();

    void on_btn_flight_frount_pressed();
    void on_btn_flight_back_pressed();
    void on_btn_flight_send_clicked();
    void on_btn_flight_runTask_clicked();
    void on_btn_flight_arm_clicked(bool checked);
    void on_btn_flight_up_pressed();
    void on_btn_flight_left_pressed();
    void on_btn_flight_right_pressed();
    void on_btn_flight_down_pressed();
    void on_btn_flight_leftYaw_pressed();
    void on_btn_flight_rightYaw_pressed();
    void on_btg_flight_HL(QAbstractButton *button);
    void on_btg_flight_VL(QAbstractButton *button);
    void on_btg_flight_YL(QAbstractButton *button);
    void on_btg_flight_CL(QAbstractButton *button);
    void on_btg_flight_SM(QAbstractButton *button);

    void updateFlightFlag();
    void updateFlightX();
    void updateFlightY();
    void updateFlightZ();
    void updateFlightYaw();
    void resetFlightData();
    void flightSend();
    void on_btn_FlightAcc_clicked();
    void on_btn_FlightPal_clicked();
    void on_btn_FlightMag_clicked();
    void on_btn_FlightQua_clicked();
    void on_btn_FlightVel_clicked();
    void on_btn_FlightPos_clicked();

    void on_lineEdit_flight_X_editingFinished();
    void on_lineEdit_flight_Y_editingFinished();
    void on_lineEdit_flight_Z_editingFinished();
    void on_lineEdit_flight_Yaw_editingFinished();
    void on_lineEdit_flight_X_returnPressed();
    void on_lineEdit_flight_Y_returnPressed();
    void on_lineEdit_flight_Z_returnPressed();
    void on_lineEdit_flight_Yaw_returnPressed();
    void on_cb_flight_autoSend_clicked(bool checked);
    void on_btn_flight_dataReset_clicked();
    void filght_autosend();

    void on_btn_log_clean_clicked();
    void on_btn_log_save_clicked();

    void on_cb_mission_hp_clicked(bool checked);
    void on_cb_mission_wp_clicked(bool checked);
    void on_cb_mission_follow_clicked(bool checked);

    void on_btn_hotPoint_start_clicked();
    void on_btn_hotPoint_stop_clicked();
    void on_btn_hotPoint_current_clicked();
    void on_btn_hp_pause_clicked(bool checked);
    void on_btn_hp_setPal_clicked();
    void on_btn_hp_setRadius_clicked();
    void on_btn_hp_setYaw_clicked();
    void on_btn_hp_data_clicked();

    void on_btn_follow_current_clicked();
    void on_hs_follow_la_valueChanged(int value);
    void on_hs_follow_lo_valueChanged(int value);
    void on_hs_follow_al_valueChanged(int value);
    void on_hs_follow_an_valueChanged(int value);
    void on_btn_follow_update_clicked();
    void on_btn_follow_start_clicked();
    void on_btn_follow_pause_clicked(bool checked);
    void on_btn_follow_stop_clicked();
    void on_tmr_follow_send();

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
    void on_btn_waypoint_init_clicked();
    void on_btn_wp_pr_clicked(bool checked);
    void on_le_wp_exec_editingFinished();
    void on_btn_wp_loadAll_clicked();
    void on_btn_wp_start_stop_clicked(bool checked);
    void on_btn_wp_loadOne_clicked();
    void on_waypoint_data_changed(const QModelIndex &topLeft, const QModelIndex &bottomRight,
                                  const QVector<int> &roles __UNUSED);

    void on_btn_gps_read_clicked();
    void on_btn_rtk_read_clicked();

    //! @todo sort

    void on_btn_webTool_clicked(bool checked);

    void on_btn_AbortWaypoint_clicked();

private:
#ifdef GROUNDSTATION
  private:
    void initGroundStation();
    void initMap();

  private slots:
    void on_btn_webLoad_clicked();
    void on_btn_webRefresh_clicked();

  private:
    QWebEngineView *webView;
#endif // GROUNDSTATION

  private:
    Ui::DJIonboardSDK *ui;

    CoreAPI *api;
    QtOnboardsdkPortDriver *driver;
    QSerialPort *port;
    QByteArray *key;

    Flight *flight;
    uint8_t flightFlag;
    float32_t flightX;
    float32_t flightY;
    float32_t flightZ;
    float32_t flightYaw;
    QTimer *autoSend;

    VirtualRC *vrc;
    QTimer *vrcSend;

    HotPoint *hp;

    Camera *cam;
    uint8_t camFlag;
    QTimer *cameraSend;

    Follow *follow;
    QTimer *followSend;

    WayPoint *wp;
    QStandardItemModel *waypointData;
    QStandardItemModel *currentAction;
    QStandardItemModel *nullAction;
    QList<QStandardItemModel *> actionData;

    QTimer *timerBroadcast;

    WayPointData wayPointDataTmp;


#ifdef SDK_DEV
    UIDev *dev;
#endif // SDK_DEV
    void functionAlloc();

    unsigned int versionIndex;

};

#endif // DJIONBOARDSDK_H
