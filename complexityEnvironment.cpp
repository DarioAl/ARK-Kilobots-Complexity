#ifndef COMPLEXITYENVIRONMENT_CPP
#define COMPLEXITYENVIRONMENT_CPP

#include "complexityEnvironment.h"
#include "resources.h"
#include "area.h"

#include "kilobot.h"

#include <QVector>
#include <QVector2D>
#include <QLineF>
#include <QDebug>
#include <QtMath>
#include <QColor>

mykilobotenvironment::mykilobotenvironment(QObject *parent) : KilobotEnvironment(parent) {
    // environment specifications
    this->ArenaX = 1;
    this->ArenaY = 1;
    this->ongoingRuntimeIdentification = false;

    // define environment:
    // call any functions to setup features in the environment
    reset();
}

void mykilobotenvironment::reset() {
    qDebug() << QString("in reset");

    this->time = 0;
    this->minTimeBetweenTwoMessages = 0;
    this->ongoingRuntimeIdentification = false;

    // measurements expressed in mm
    resources.clear();
    kilobots_states.clear();
    kilobots_positions.clear();

    QVector<Area> oth_areas;
    double area_radius = 70;
    Resource a(0, 1000, area_radius, 0.1, oth_areas);
    std::cout << oth_areas.size() << std::endl;
    Resource b(1, 1000, area_radius, 0.0, oth_areas);
        std::cout << oth_areas.size() << std::endl;
    Resource c(2, 1000, area_radius, 0.0, oth_areas);
        std::cout << oth_areas.size() << std::endl;

    resources.push_back(a);
    resources.push_back(b);
    resources.push_back(c);
}

void mykilobotenvironment::update() {
    //qDebug() << QString("in update");

    // get all areas in a single array to avoid placing duplicate
    QVector<Area> allAreas;
    for(Resource r : resources) {
        for(Area a : r.areas) {
            allAreas.append(a);
        }
    }

    // update resources and areas
    for(Resource& r : resources) {
        r.doStep(kilobots_positions, kilobots_states, kilobots_colours, allAreas, 1000);
    }
}

// generate virtual sensors reading and send it to the kbs (same as for ARGOS)
void mykilobotenvironment::updateVirtualSensor(Kilobot kilobot_entity) {
    qDebug() << QString("in update virtual sensors");

    // update local arrays
    // update kilobot position
    kilobot_id k_id = kilobot_entity.getID();
    this->kilobots_positions[k_id] = kilobot_entity.getPosition();

    // update kilobot led colour (indicates the internal decision state of the kb)
    lightColour kb_colour = kilobot_entity.getLedColour();
    if(kb_colour == lightColour::RED)
        this->kilobots_colours[k_id] = Qt::red;     // committed not working
    else if(kb_colour == lightColour::GREEN)
        this->kilobots_colours[k_id] = Qt::green;   // committed and working
    else if(kb_colour == lightColour::BLUE)
        this->kilobots_colours[k_id] = Qt::blue;    // uncommitted
    else
        qDebug() << "ERROR: kilobot " << k_id << " at position " << kilobot_entity.getPosition() << " has its light off";

    // update kilobots status (where in the arena it is)
    bool overAnArea = false;
    // paint the resources
    this->kilobots_states[k_id] = 255; // start as uncommitted
    for(int i=0; i<resources.size(); i++) {
        Resource r = resources.at(i);
        for(const Area& a : r.areas) {
            double squared_xs = pow(kilobot_entity.getPosition().x() - a.position.x(),2);
            double squared_ys = pow(kilobot_entity.getPosition().y() - a.position.y(),2);
            if(squared_xs + squared_ys < pow(a.radius,2)) {
                this->kilobots_states[k_id] = r.type;
                overAnArea = true;
                // avoid the double loop if found
                break;
            }
        }
        // avoid the double loop if found
        if(overAnArea)
            break;
    }

    // now we have everything up to date and everything we need
    // then if it is time to send the message to the kilobot send info to the kb
    if(this->time - this->lastSent[k_id] > minTimeBetweenTwoMessages && !ongoingRuntimeIdentification){
        qDebug() << "SENDING OUT MESSAGE";
        lastSent[k_id] = this->time;

        // create and fill the message
        kilobot_message message; // this is a 24 bits field not the original kb message
        // make sure to start clean
        message.type = 0;
        message.id = 0;
        message.data = 0;

        /* Prepare the individual kilobot message     */
        /* xxxx xxxx xyyz zzzw wwww wwww              */
        /* x bits used for bilobot id                 */
        /* y bits used for kilobot arena state        */
        /* z bits used for resource umin              */
        /* w bits used for kilobot rotation to center */

        // store kb id
        message.type = (k_id >> 5);
        message.id = k_id << 5;

        // store kb arena state
        // for the kb the state is 0 if empty
        // 1 2 3 over area 1 area 2 or area 3
        uint8_t kb_arena_state = kilobots_states.at(k_id) + 1; // add +1 to convert to kb form
        message.id = message.id | (kb_arena_state << 3);

        // store resource umin (if arena state not on empty space)
        if(kb_arena_state != 0) { // i.e. 255 in local kilobots_state list
            uint8_t rumin = resources.at(kb_arena_state-1).umin;
            message.id = message.id | (rumin >> 1);
            message.data = rumin << 9;
        }

        // store kb rotation toward the center if the kb is too close to the border
        // this is used to avoid that the kb gets stuck in the wall

        QVector2D kb_position(this->kilobots_positions[k_id].x(), this->kilobots_positions[k_id].y());
        kb_position.normalize();
        double distance_from_centre = sqrt(pow(kb_position.x(),2)+pow(kb_position.y(),2));

        if (distance_from_centre > 0.92) {
            double orient_degrees = qRadiansToDegrees(qAtan2(-kilobot_entity.getVelocity().y(), kilobot_entity.getVelocity().x()));
            QVector2D kb_orientation(1, orient_degrees);
            kb_orientation.normalize();
            double turning_angle = M_PI	* QVector2D::dotProduct(kb_orientation,kb_position);

            uint8_t angle_sign = turning_angle > 0? 1 : -1;
            uint8_t degree_angle = turning_angle * 180 / M_PI;
            message.data = message.data | angle_sign << 8 | degree_angle;
        }

        // send it
        emit transmitKiloState(message);
    }
}

#endif // COMPLEXITYENVIRONMENT_CPP
