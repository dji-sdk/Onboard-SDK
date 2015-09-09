#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "win_qextserialport.h"
#include <QTimer>
#include <QByteArray>
#include <QLabel>
#include <QSettings>
#include "DJI_Pro_Sample.h"

#define TAKEOFF    4
#define LANDING    6

#define OPEN_SERIAL     "Open"
#define CLOSE_SERIAL    "Close"

#define NAV_OPEN        "Obtain Control"
#define NAV_CLOSE       "Release Control"
#define SETTING_FILENAMES  "Setting.ini"

#define DEFAULT_KEY     "0123012301230123012301230123012301230123012301230123012301230123"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_BtnActivation_clicked();
    void on_btn_open_serialport_clicked();

    void on_btn_nav_open_close_clicked();

    void on_btn_Takeoff_clicked();

    void on_btn_update_com_clicked();

    void Timeout_handle();

    void TakeoffDelay_handle();

    void btn_About();

    void on_btn_Landing_clicked();

    void on_btn_GoHome_clicked();

    void on_btn_gimbal_ctrl_clicked();

    void on_btn_atti_ctrl_clicked();

private:
    Ui::MainWindow *ui;
    QTimer *time;
    QTimer *TakeoffDelay;
    QByteArray *keybuf;
    activate_data_t user_act_data;
    void Set_Default_Ini();
    int Get_Check(QSettings *set);
    void Read_Setting();
    void Save_Setting();
    static void MainWindow_Activate_Callback(unsigned short res);
public:
    static MainWindow *mainwindow_object;
    static MainWindow* Create_Instance(void);
    static MainWindow* Get_Instance(void);
};

#endif // MAINWINDOW_H
