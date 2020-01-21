#ifndef COMPLEXITYENVIRONMENT_H
#define COMPLEXITYENVIRONMENT_H

/**
 * Author: Dario Albani
 *
 * This is the code that specifies the specific environment used for the complexity experiment.
 * The environment is composed by empty spaces and up to three resources that have to be exploited by the kilobots.
 * Colours for the resources are green, blue and red (not be confused with led colours for the kb having different meanings).
 */

#include <QObject>
#include <QPointF>
#include <QVector>
#include <QVector3D>
#include <QTime>
#include <QMatrix>
#include <QList>
#include <QColor>
#include <QElapsedTimer>

#include <limits>

#include <kilobotenvironment.h>
#include "resources.h"
#include "area.h"

class mykilobotenvironment : public KilobotEnvironment {
 Q_OBJECT
public:
    explicit mykilobotenvironment(QObject *parent=0);
    void reset();

    QVector<Resource> resources; // list of all resources present in the experiment
    QVector<uint8_t> kilobots_states;  // list of all kilobots locations meaning 255 for empty spaces and 1, 2, 3 for resources
    QVector<QPointF> kilobots_positions;    // list of all kilobots positions
    QVector<QColor> kilobots_colours;  // list of all kilobots led colours, the led indicate committed (red), working (green) and uncommiited (blue)
    QVector<float> lastSent;    // when the last message was sent to the kb at given position

    float minTimeBetweenTwoMessages;    // minimum time between two messages
    double time;
    int ArenaX, ArenaY;
    bool ongoingRuntimeIdentification;



// signals and slots are used by qt to signal state changes to objects
signals:
    void errorMessage(QString);

public slots:
    void update();
    void updateVirtualSensor(Kilobot kilobot);

private:

};

#endif // COMPLEXITYENVIRONMENT_H
