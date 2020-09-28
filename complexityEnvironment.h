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


#define ARENA_CENTER 750
#define ARENA_SIZE 746

#define EXPLORATION_TIME 5
#define COMMUNICATION_TIME 2

class mykilobotenvironment : public KilobotEnvironment {
 Q_OBJECT
public:
    explicit mykilobotenvironment(QObject *parent=0);
    void reset();

    QVector<kilobot_arena_state> kilobots_states;  // list of all kilobots locations meaning 255 for empty spaces and 1, 2, 3 for resources

    QVector<Resource*> resources; // list of all resources present in the experiment
    QVector<QPointF> kilobots_positions;    // list of all kilobots positions
    QVector<QColor> kilobots_colours;  // list of all kilobots led colours, the led indicate committed (red), working (green) and uncommiited (blue)
    QVector<float> lastSent;    // when the last message was sent to the kb at given position

    float minTimeBetweenTwoMessages;    // minimum time between two messages
    double time;

    int ArenaX, ArenaY;
    bool ongoingRuntimeIdentification;

    // used to implement the mechanism that switch between exploration and communication (i.e., robots not moving, experiments frozen)
    double lastTransitionTime; // used to switch between exploration time and communication time
    double lastCommunication; // used to transmit either the "communicate" or "stop communication" message three times per second
    bool isCommunicationTime; // determine if the robots are communicating or explorations

// signals and slots are used by qt to signal state changes to objects
signals:
    void errorMessage(QString);

public slots:
    void update();
    void updateVirtualSensor(Kilobot kilobot);

private:

};

#endif // COMPLEXITYENVIRONMENT_H
