#ifndef V1VEIWER_H
#define V1VEIWER_H

#include <QMainWindow>
#include <QStandardItemModel>
#include <QTimerEvent>
#include <QTimer>
#include "QHardDriver.h"

namespace Ui
{
class V1Veiwer;
}

class V1Veiwer : public QMainWindow
{
    Q_OBJECT

  public:
    explicit V1Veiwer(QWidget *parent = 0);
    ~V1Veiwer();

    void setupSerialPort();

    /*! @note serial port management*/
  protected:
    void timerEvent(QTimerEvent *event);

  private slots:
    void on_cb_portName_currentIndexChanged(int index);
    void on_btn_portOpen_clicked();
    void on_btn_portRefresh_clicked();
    void on_btn_cvtRR_clicked();
    void on_btn_cvtRS_clicked();
    void on_btn_send_clicked();

private:
    void setBaudrate();
    void setPort();
    void openPort();
    void closePort();
    void refreshPort();


  private:
    Ui::V1Veiwer *ui;
    QSerialPort *port;
    QHardDriver *driver;

    QStandardItemModel *sendTable;
    QStandardItemModel *recvTable;
};

#endif // V1VEIWER_H
