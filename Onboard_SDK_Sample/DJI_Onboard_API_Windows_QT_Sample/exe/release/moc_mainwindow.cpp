/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[14];
    char stringdata0[289];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 24), // "on_BtnActivation_clicked"
QT_MOC_LITERAL(2, 36, 0), // ""
QT_MOC_LITERAL(3, 37, 30), // "on_btn_open_serialport_clicked"
QT_MOC_LITERAL(4, 68, 29), // "on_btn_nav_open_close_clicked"
QT_MOC_LITERAL(5, 98, 22), // "on_btn_Takeoff_clicked"
QT_MOC_LITERAL(6, 121, 25), // "on_btn_update_com_clicked"
QT_MOC_LITERAL(7, 147, 14), // "Timeout_handle"
QT_MOC_LITERAL(8, 162, 19), // "TakeoffDelay_handle"
QT_MOC_LITERAL(9, 182, 9), // "btn_About"
QT_MOC_LITERAL(10, 192, 22), // "on_btn_Landing_clicked"
QT_MOC_LITERAL(11, 215, 21), // "on_btn_GoHome_clicked"
QT_MOC_LITERAL(12, 237, 26), // "on_btn_gimbal_ctrl_clicked"
QT_MOC_LITERAL(13, 264, 24) // "on_btn_atti_ctrl_clicked"

    },
    "MainWindow\0on_BtnActivation_clicked\0"
    "\0on_btn_open_serialport_clicked\0"
    "on_btn_nav_open_close_clicked\0"
    "on_btn_Takeoff_clicked\0on_btn_update_com_clicked\0"
    "Timeout_handle\0TakeoffDelay_handle\0"
    "btn_About\0on_btn_Landing_clicked\0"
    "on_btn_GoHome_clicked\0on_btn_gimbal_ctrl_clicked\0"
    "on_btn_atti_ctrl_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   74,    2, 0x08 /* Private */,
       3,    0,   75,    2, 0x08 /* Private */,
       4,    0,   76,    2, 0x08 /* Private */,
       5,    0,   77,    2, 0x08 /* Private */,
       6,    0,   78,    2, 0x08 /* Private */,
       7,    0,   79,    2, 0x08 /* Private */,
       8,    0,   80,    2, 0x08 /* Private */,
       9,    0,   81,    2, 0x08 /* Private */,
      10,    0,   82,    2, 0x08 /* Private */,
      11,    0,   83,    2, 0x08 /* Private */,
      12,    0,   84,    2, 0x08 /* Private */,
      13,    0,   85,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_BtnActivation_clicked(); break;
        case 1: _t->on_btn_open_serialport_clicked(); break;
        case 2: _t->on_btn_nav_open_close_clicked(); break;
        case 3: _t->on_btn_Takeoff_clicked(); break;
        case 4: _t->on_btn_update_com_clicked(); break;
        case 5: _t->Timeout_handle(); break;
        case 6: _t->TakeoffDelay_handle(); break;
        case 7: _t->btn_About(); break;
        case 8: _t->on_btn_Landing_clicked(); break;
        case 9: _t->on_btn_GoHome_clicked(); break;
        case 10: _t->on_btn_gimbal_ctrl_clicked(); break;
        case 11: _t->on_btn_atti_ctrl_clicked(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow.data,
      qt_meta_data_MainWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
