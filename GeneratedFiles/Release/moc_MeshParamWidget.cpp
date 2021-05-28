/****************************************************************************
** Meta object code from reading C++ file 'MeshParamWidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.14.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../MeshParamWidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'MeshParamWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.14.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MeshParamWidget_t {
    QByteArrayData data[15];
    char stringdata0[274];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MeshParamWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MeshParamWidget_t qt_meta_stringdata_MeshParamWidget = {
    {
QT_MOC_LITERAL(0, 0, 15), // "MeshParamWidget"
QT_MOC_LITERAL(1, 16, 15), // "PrintInfoSignal"
QT_MOC_LITERAL(2, 32, 0), // ""
QT_MOC_LITERAL(3, 33, 9), // "RunSignal"
QT_MOC_LITERAL(4, 43, 11), // "ResetSignal"
QT_MOC_LITERAL(5, 55, 15), // "ParamInitSignal"
QT_MOC_LITERAL(6, 71, 15), // "ParamIterSignal"
QT_MOC_LITERAL(7, 87, 14), // "ParamITDSignal"
QT_MOC_LITERAL(8, 102, 16), // "ParamResetSignal"
QT_MOC_LITERAL(9, 119, 26), // "ParamViewModeChangedSignal"
QT_MOC_LITERAL(10, 146, 28), // "ParamInitMethodChangedSignal"
QT_MOC_LITERAL(11, 175, 30), // "ParamPlanarMethodChangedSignal"
QT_MOC_LITERAL(12, 206, 33), // "ParamSphericalMethodChangedSi..."
QT_MOC_LITERAL(13, 240, 24), // "ParamWeightChangedSignal"
QT_MOC_LITERAL(14, 265, 8) // "CloseTab"

    },
    "MeshParamWidget\0PrintInfoSignal\0\0"
    "RunSignal\0ResetSignal\0ParamInitSignal\0"
    "ParamIterSignal\0ParamITDSignal\0"
    "ParamResetSignal\0ParamViewModeChangedSignal\0"
    "ParamInitMethodChangedSignal\0"
    "ParamPlanarMethodChangedSignal\0"
    "ParamSphericalMethodChangedSignal\0"
    "ParamWeightChangedSignal\0CloseTab"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MeshParamWidget[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      12,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   79,    2, 0x06 /* Public */,
       3,    0,   80,    2, 0x06 /* Public */,
       4,    0,   81,    2, 0x06 /* Public */,
       5,    0,   82,    2, 0x06 /* Public */,
       6,    0,   83,    2, 0x06 /* Public */,
       7,    0,   84,    2, 0x06 /* Public */,
       8,    0,   85,    2, 0x06 /* Public */,
       9,    1,   86,    2, 0x06 /* Public */,
      10,    1,   89,    2, 0x06 /* Public */,
      11,    1,   92,    2, 0x06 /* Public */,
      12,    1,   95,    2, 0x06 /* Public */,
      13,    1,   98,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      14,    1,  101,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString,    2,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    2,

       0        // eod
};

void MeshParamWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MeshParamWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->PrintInfoSignal(); break;
        case 1: _t->RunSignal(); break;
        case 2: _t->ResetSignal(); break;
        case 3: _t->ParamInitSignal(); break;
        case 4: _t->ParamIterSignal(); break;
        case 5: _t->ParamITDSignal(); break;
        case 6: _t->ParamResetSignal(); break;
        case 7: _t->ParamViewModeChangedSignal((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 8: _t->ParamInitMethodChangedSignal((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 9: _t->ParamPlanarMethodChangedSignal((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 10: _t->ParamSphericalMethodChangedSignal((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 11: _t->ParamWeightChangedSignal((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 12: _t->CloseTab((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (MeshParamWidget::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MeshParamWidget::PrintInfoSignal)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (MeshParamWidget::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MeshParamWidget::RunSignal)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (MeshParamWidget::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MeshParamWidget::ResetSignal)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (MeshParamWidget::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MeshParamWidget::ParamInitSignal)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (MeshParamWidget::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MeshParamWidget::ParamIterSignal)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (MeshParamWidget::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MeshParamWidget::ParamITDSignal)) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (MeshParamWidget::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MeshParamWidget::ParamResetSignal)) {
                *result = 6;
                return;
            }
        }
        {
            using _t = void (MeshParamWidget::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MeshParamWidget::ParamViewModeChangedSignal)) {
                *result = 7;
                return;
            }
        }
        {
            using _t = void (MeshParamWidget::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MeshParamWidget::ParamInitMethodChangedSignal)) {
                *result = 8;
                return;
            }
        }
        {
            using _t = void (MeshParamWidget::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MeshParamWidget::ParamPlanarMethodChangedSignal)) {
                *result = 9;
                return;
            }
        }
        {
            using _t = void (MeshParamWidget::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MeshParamWidget::ParamSphericalMethodChangedSignal)) {
                *result = 10;
                return;
            }
        }
        {
            using _t = void (MeshParamWidget::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MeshParamWidget::ParamWeightChangedSignal)) {
                *result = 11;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject MeshParamWidget::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_MeshParamWidget.data,
    qt_meta_data_MeshParamWidget,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *MeshParamWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MeshParamWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MeshParamWidget.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int MeshParamWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 13)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 13;
    }
    return _id;
}

// SIGNAL 0
void MeshParamWidget::PrintInfoSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void MeshParamWidget::RunSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void MeshParamWidget::ResetSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void MeshParamWidget::ParamInitSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}

// SIGNAL 4
void MeshParamWidget::ParamIterSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 4, nullptr);
}

// SIGNAL 5
void MeshParamWidget::ParamITDSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 5, nullptr);
}

// SIGNAL 6
void MeshParamWidget::ParamResetSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 6, nullptr);
}

// SIGNAL 7
void MeshParamWidget::ParamViewModeChangedSignal(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void MeshParamWidget::ParamInitMethodChangedSignal(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void MeshParamWidget::ParamPlanarMethodChangedSignal(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void MeshParamWidget::ParamSphericalMethodChangedSignal(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 10, _a);
}

// SIGNAL 11
void MeshParamWidget::ParamWeightChangedSignal(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 11, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
