/********************************************************************************
** Form generated from reading UI file 'about.ui'
**
** Created by: Qt User Interface Compiler version 5.5.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ABOUT_H
#define UI_ABOUT_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_About
{
public:
    QPushButton *btn_close;
    QLabel *label;
    QLabel *label_2;

    void setupUi(QDialog *About)
    {
        if (About->objectName().isEmpty())
            About->setObjectName(QStringLiteral("About"));
        About->resize(414, 176);
        btn_close = new QPushButton(About);
        btn_close->setObjectName(QStringLiteral("btn_close"));
        btn_close->setGeometry(QRect(320, 140, 75, 23));
        label = new QLabel(About);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(30, 40, 111, 91));
        label->setPixmap(QPixmap(QString::fromUtf8(":/icon/Images/DJI.png")));
        label->setScaledContents(true);
        label_2 = new QLabel(About);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(160, 50, 261, 41));
        label_2->setAcceptDrops(false);
        label_2->setTextFormat(Qt::AutoText);

        retranslateUi(About);

        QMetaObject::connectSlotsByName(About);
    } // setupUi

    void retranslateUi(QDialog *About)
    {
        About->setWindowTitle(QApplication::translate("About", "About", 0));
        btn_close->setText(QApplication::translate("About", "close", 0));
        label->setText(QString());
        label_2->setText(QApplication::translate("About", "Copyright 2015-2016 \n"
"The DJI Company Ltd.All rights reserved.\n"
"Version:1.1.0", 0));
    } // retranslateUi

};

namespace Ui {
    class About: public Ui_About {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ABOUT_H
