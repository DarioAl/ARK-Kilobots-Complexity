#ifndef KILOBOTENVIRONMENT_H
#define KILOBOTENVIRONMENT_H

#include <QObject>
#include <QVector>
#include "kilobot.h"

class KilobotEnvironment : public QObject
{
    Q_OBJECT
public:
    typedef enum {
        OUTSIDE_AREA=255,
        INSIDE_AREA_0=0,
        INSIDE_AREA_1=1,
        INSIDE_AREA_2=2,
        INSIDE_AREA_01=3,
        INSIDE_AREA_02=6,
        INSIDE_AREA_12=7,
        INSIDE_AREA_012=9,
    } kilobot_arena_state;

    explicit KilobotEnvironment(QObject *) {}
    KilobotEnvironment() {}

signals:
    void transmitKiloState(kilobot_message);
    //void broadcastMessage(kilobot_broadcast);

public slots:
    virtual void update() {}
    virtual void updateVirtualSensor(Kilobot) {} // Call this updateVirtualSensor(...)


};

#endif // KILOBOTENVIRONMENT_H
