/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.5.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionAbout;
    QWidget *centralWidget;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout;
    QPushButton *BtnActivation;
    QPushButton *btn_nav_open_close;
    QPushButton *btn_Takeoff;
    QPushButton *btn_Landing;
    QPushButton *btn_GoHome;
    QPushButton *btn_atti_ctrl;
    QPushButton *btn_gimbal_ctrl;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout;
    QPushButton *btn_open_serialport;
    QComboBox *serial_comboBox;
    QPushButton *btn_update_com;
    QComboBox *baud_comboBox;
    QGroupBox *groupBox_3;
    QVBoxLayout *verticalLayout_4;
    QLabel *label;
    QLineEdit *AppID;
    QLabel *label_2;
    QComboBox *AppLevel;
    QLabel *label_3;
    QLineEdit *AppKey;
    QGroupBox *groupBox_4;
    QWidget *layoutWidget1;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_6;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_Activation_Status;
    QLabel *label_Battery_Capacity;
    QLabel *label_Control_Device;
    QMenuBar *menuBar;
    QMenu *menuHelp;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(372, 389);
        MainWindow->setLayoutDirection(Qt::LeftToRight);
        actionAbout = new QAction(MainWindow);
        actionAbout->setObjectName(QStringLiteral("actionAbout"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(190, 0, 161, 271));
        verticalLayout = new QVBoxLayout(groupBox);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        BtnActivation = new QPushButton(groupBox);
        BtnActivation->setObjectName(QStringLiteral("BtnActivation"));

        verticalLayout->addWidget(BtnActivation);

        btn_nav_open_close = new QPushButton(groupBox);
        btn_nav_open_close->setObjectName(QStringLiteral("btn_nav_open_close"));

        verticalLayout->addWidget(btn_nav_open_close);

        btn_Takeoff = new QPushButton(groupBox);
        btn_Takeoff->setObjectName(QStringLiteral("btn_Takeoff"));

        verticalLayout->addWidget(btn_Takeoff);

        btn_Landing = new QPushButton(groupBox);
        btn_Landing->setObjectName(QStringLiteral("btn_Landing"));

        verticalLayout->addWidget(btn_Landing);

        btn_GoHome = new QPushButton(groupBox);
        btn_GoHome->setObjectName(QStringLiteral("btn_GoHome"));

        verticalLayout->addWidget(btn_GoHome);

        btn_atti_ctrl = new QPushButton(groupBox);
        btn_atti_ctrl->setObjectName(QStringLiteral("btn_atti_ctrl"));

        verticalLayout->addWidget(btn_atti_ctrl);

        btn_gimbal_ctrl = new QPushButton(groupBox);
        btn_gimbal_ctrl->setObjectName(QStringLiteral("btn_gimbal_ctrl"));

        verticalLayout->addWidget(btn_gimbal_ctrl);

        groupBox_2 = new QGroupBox(centralWidget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(10, 0, 171, 111));
        gridLayout = new QGridLayout(groupBox_2);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        btn_open_serialport = new QPushButton(groupBox_2);
        btn_open_serialport->setObjectName(QStringLiteral("btn_open_serialport"));
        QIcon icon;
        icon.addFile(QStringLiteral(":/icon/Images/err.png"), QSize(), QIcon::Normal, QIcon::Off);
        btn_open_serialport->setIcon(icon);

        gridLayout->addWidget(btn_open_serialport, 0, 0, 1, 2);

        serial_comboBox = new QComboBox(groupBox_2);
        serial_comboBox->setObjectName(QStringLiteral("serial_comboBox"));

        gridLayout->addWidget(serial_comboBox, 1, 0, 1, 1);

        btn_update_com = new QPushButton(groupBox_2);
        btn_update_com->setObjectName(QStringLiteral("btn_update_com"));

        gridLayout->addWidget(btn_update_com, 1, 1, 1, 1);

        baud_comboBox = new QComboBox(groupBox_2);
        baud_comboBox->setObjectName(QStringLiteral("baud_comboBox"));
        baud_comboBox->setEditable(false);
        baud_comboBox->setModelColumn(0);

        gridLayout->addWidget(baud_comboBox, 2, 0, 1, 2);

        groupBox_3 = new QGroupBox(centralWidget);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        groupBox_3->setGeometry(QRect(10, 117, 171, 151));
        verticalLayout_4 = new QVBoxLayout(groupBox_3);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        label = new QLabel(groupBox_3);
        label->setObjectName(QStringLiteral("label"));

        verticalLayout_4->addWidget(label);

        AppID = new QLineEdit(groupBox_3);
        AppID->setObjectName(QStringLiteral("AppID"));

        verticalLayout_4->addWidget(AppID);

        label_2 = new QLabel(groupBox_3);
        label_2->setObjectName(QStringLiteral("label_2"));

        verticalLayout_4->addWidget(label_2);

        AppLevel = new QComboBox(groupBox_3);
        AppLevel->setObjectName(QStringLiteral("AppLevel"));

        verticalLayout_4->addWidget(AppLevel);

        label_3 = new QLabel(groupBox_3);
        label_3->setObjectName(QStringLiteral("label_3"));

        verticalLayout_4->addWidget(label_3);

        AppKey = new QLineEdit(groupBox_3);
        AppKey->setObjectName(QStringLiteral("AppKey"));

        verticalLayout_4->addWidget(AppKey);

        groupBox_4 = new QGroupBox(centralWidget);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        groupBox_4->setGeometry(QRect(10, 270, 341, 61));
        layoutWidget1 = new QWidget(groupBox_4);
        layoutWidget1->setObjectName(QStringLiteral("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(10, 10, 110, 50));
        verticalLayout_2 = new QVBoxLayout(layoutWidget1);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        label_4 = new QLabel(layoutWidget1);
        label_4->setObjectName(QStringLiteral("label_4"));

        verticalLayout_2->addWidget(label_4);

        label_5 = new QLabel(layoutWidget1);
        label_5->setObjectName(QStringLiteral("label_5"));

        verticalLayout_2->addWidget(label_5);

        label_6 = new QLabel(layoutWidget1);
        label_6->setObjectName(QStringLiteral("label_6"));

        verticalLayout_2->addWidget(label_6);

        layoutWidget = new QWidget(groupBox_4);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(120, 10, 211, 50));
        verticalLayout_3 = new QVBoxLayout(layoutWidget);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        label_Activation_Status = new QLabel(layoutWidget);
        label_Activation_Status->setObjectName(QStringLiteral("label_Activation_Status"));

        verticalLayout_3->addWidget(label_Activation_Status);

        label_Battery_Capacity = new QLabel(layoutWidget);
        label_Battery_Capacity->setObjectName(QStringLiteral("label_Battery_Capacity"));

        verticalLayout_3->addWidget(label_Battery_Capacity);

        label_Control_Device = new QLabel(layoutWidget);
        label_Control_Device->setObjectName(QStringLiteral("label_Control_Device"));

        verticalLayout_3->addWidget(label_Control_Device);

        MainWindow->setCentralWidget(centralWidget);
        groupBox_3->raise();
        groupBox_2->raise();
        groupBox->raise();
        groupBox_4->raise();
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 372, 23));
        menuHelp = new QMenu(menuBar);
        menuHelp->setObjectName(QStringLiteral("menuHelp"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuHelp->menuAction());
        menuHelp->addAction(actionAbout);

        retranslateUi(MainWindow);

        baud_comboBox->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "DJI_Onboard_API_Test", 0));
        actionAbout->setText(QApplication::translate("MainWindow", "About", 0));
#ifndef QT_NO_STATUSTIP
        actionAbout->setStatusTip(QString());
#endif // QT_NO_STATUSTIP
        actionAbout->setShortcut(QApplication::translate("MainWindow", "Ctrl+A", 0));
        groupBox->setTitle(QApplication::translate("MainWindow", "Commands", 0));
        BtnActivation->setText(QApplication::translate("MainWindow", "Activation", 0));
        btn_nav_open_close->setText(QApplication::translate("MainWindow", "Obtain Control", 0));
        btn_Takeoff->setText(QApplication::translate("MainWindow", "Takeoff", 0));
        btn_Landing->setText(QApplication::translate("MainWindow", "Landing", 0));
        btn_GoHome->setText(QApplication::translate("MainWindow", "Go Home", 0));
        btn_atti_ctrl->setText(QApplication::translate("MainWindow", "Attitude Ctrl Sample", 0));
        btn_gimbal_ctrl->setText(QApplication::translate("MainWindow", "Gimbal Ctrl Sample", 0));
        groupBox_2->setTitle(QApplication::translate("MainWindow", "Serial Port", 0));
        btn_open_serialport->setText(QApplication::translate("MainWindow", "Open", 0));
        btn_update_com->setText(QApplication::translate("MainWindow", "Refresh", 0));
        baud_comboBox->clear();
        baud_comboBox->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "230400", 0)
         << QApplication::translate("MainWindow", "115200", 0)
         << QApplication::translate("MainWindow", "54600", 0)
         << QApplication::translate("MainWindow", "38400", 0)
         << QApplication::translate("MainWindow", "19200", 0)
         << QApplication::translate("MainWindow", "9600", 0)
         << QApplication::translate("MainWindow", "4800", 0)
        );
        groupBox_3->setTitle(QApplication::translate("MainWindow", "User Setting", 0));
        label->setText(QApplication::translate("MainWindow", "APP id", 0));
        AppID->setText(QString());
        label_2->setText(QApplication::translate("MainWindow", "API level", 0));
        AppLevel->clear();
        AppLevel->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "1", 0)
         << QApplication::translate("MainWindow", "2", 0)
        );
        label_3->setText(QApplication::translate("MainWindow", "Key", 0));
        AppKey->setText(QString());
        groupBox_4->setTitle(QApplication::translate("MainWindow", "UAV Info", 0));
        label_4->setText(QApplication::translate("MainWindow", "Activation status:", 0));
        label_5->setText(QApplication::translate("MainWindow", "Battery capacity:", 0));
        label_6->setText(QApplication::translate("MainWindow", "Control device:", 0));
        label_Activation_Status->setText(QApplication::translate("MainWindow", "unknown", 0));
        label_Battery_Capacity->setText(QApplication::translate("MainWindow", "unknown", 0));
        label_Control_Device->setText(QApplication::translate("MainWindow", "unknown", 0));
        menuHelp->setTitle(QApplication::translate("MainWindow", "Help", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
