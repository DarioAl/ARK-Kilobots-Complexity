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
    this->ArenaX = 0.5;
    this->ArenaY = 0.5;
    this->ongoingRuntimeIdentification = false;

    // define environment:
    // call any functions to setup features in the environment
    reset();
}

void mykilobotenvironment::reset() {
    this->time = 0;
    this->minTimeBetweenTwoMessages = 0;
    this->ongoingRuntimeIdentification = false;

    resources.clear();
    kilobots_states.clear();
    kilobots_positions.clear();
    kilobots_colours.clear();
    kilobots_quorum.clear();

    QVector<Area> oth_areas;
    double area_radius = 146;
    Resource* a = new Resource(0, ARENA_CENTER, area_radius, 1, oth_areas);
    Resource* b = new Resource(1, ARENA_CENTER, area_radius, 1, oth_areas);
    Resource* c = new Resource(2, ARENA_CENTER, area_radius, 1, oth_areas);
    resources.push_back(a);
    resources.push_back(b);
    resources.push_back(c);

    isCommunicationTime = false;
    lastTransitionTime = this->time;
}

void mykilobotenvironment::update() {
    // if in communication time the enironment is frozen
    if(!this->isCommunicationTime) {
        // update resources and areas
        for(Resource* r : resources) {
            r->doStep();
        }
    }
}

// generate virtual sensors reading and send it to the kbs (same as for ARGOS)
void mykilobotenvironment::updateVirtualSensor(Kilobot kilobot_entity) {
    // update local arrays
    // update kilobot position
    kilobot_id k_id = kilobot_entity.getID();
    this->kilobots_positions[k_id] = kilobot_entity.getPosition();

    // update kilobot led colour (indicates the internal decision state of the kb)
    lightColour kb_colour = kilobot_entity.getLedColour();
    if(kb_colour == lightColour::RED)
        this->kilobots_colours[k_id] = Qt::red;     // committed resource 0
    else if(kb_colour == lightColour::GREEN)
        this->kilobots_colours[k_id] = Qt::green;   // committed resource 1
    else if(kb_colour == lightColour::BLUE)
        this->kilobots_colours[k_id] = Qt::blue;    // committed resource 2
    else
        this->kilobots_colours[k_id] = Qt::black;   // uncommitted

    // if in communication time only update kilobots but avoid sending information to them
    if(this->isCommunicationTime) {
#ifdef GLOBAL_QUORUM
        // store for quorum
        if(kilobots_quorum.empty()) { // only first time, list is cleared in complexityExperiment
            QVector<uint8_t> colors_hits;
            kilobots_colours[k_id]==Qt::red?colors_hits.append(1):colors_hits.append(0);
            kilobots_colours[k_id]==Qt::green?colors_hits.append(1):colors_hits.append(0);
            kilobots_colours[k_id]==Qt::blue?colors_hits.append(1):colors_hits.append(0);
            kilobots_quorum.append(colors_hits);
        } else if(kilobots_colours[k_id]==Qt::red) {
            kilobots_quorum[k_id][0] = kilobots_quorum[k_id][0]+1;
        } else if(kilobots_colours[k_id]==Qt::green) {
            kilobots_quorum[k_id][1] = kilobots_quorum[k_id][1]+1;
        } else if(kilobots_colours[k_id]==Qt::blue) {
            kilobots_quorum[k_id][2] = kilobots_quorum[k_id][2]+1;
        }
#endif
        return;
    }

    // used to update working kilbots
    Qt::GlobalColor areaColors[3] = {Qt::red, Qt::green, Qt::blue};
#ifndef REAL_UTILITY
    // used later for sending the utility
    double areasUt[3] = {0,0,0};
#endif
    // initialize as on white space
    this->kilobots_states[k_id] = (KilobotEnvironment::kilobot_arena_state)255; // start as over no area
    // cycle over the resources
    for(Resource *r : resources) {
        // and areas
        for(Area *a : r->areas) {
            // if inside
            if(a->isInside(kilobot_entity.getPosition())) {
                // check the color and update the area at the same time
                if(this->kilobots_colours[k_id] == areaColors[r->type])  {
                    a->kilobots_in_area++;
                }
                // update kilobot state
                if(this->kilobots_states[k_id] != OUTSIDE_AREA)
                    this->kilobots_states[k_id] = (KilobotEnvironment::kilobot_arena_state)(this->kilobots_states[k_id]+r->type*3);
                else
                    this->kilobots_states[k_id] = (KilobotEnvironment::kilobot_arena_state)r->type;
#ifndef REAL_UTILITY
                // update kb perception of utility (see below)
                areasUt[r->type] = a->population;
#endif
                // break and go to the next resource
                break;
            }
        }
    }

    // now we have everything up to date and everything we need
    // then if it is time to send the message to the kilobot send info to the kb
    if(this->time - this->lastSent[k_id] > minTimeBetweenTwoMessages && !ongoingRuntimeIdentification){
        lastSent[k_id] = this->time;
        // create and fill the message
        kilobot_message message; // this is a 24 bits field not the original kb message
        // make sure to start clean
        message.id = 0;
        message.type = 0;
        message.data = 0;

        // !!! THE FOLLOWING IS OF EXTREME IMPORTANCE !!!
        // NOTE although the message is defined as type, id and data, in ARK the fields type and id are swapped
        // resulting in a mixed message. If you are not using the whole field this could lead to problems.
        // To avoid it, consider to concatenate the message as ID, type and data.

        /* Prepare the inividual kilobot's message         */
        /* see README.md to understand about ARK messaging */
        /* data has 3x24 bits divided as                   */
        /*   ID 10b    type 4b  data 10b     <- ARK msg    */
        /*  data[0]   data[1]   data[2]      <- kb msg     */
        /* xxxx xxxy yyyy zzzz zwww wwtt     <- complexity */
        /* x bits used for kilobot id                      */
        /* y bits used for resource 0 population           */
        /* z bits used for resource 1 population           */
        /* w bits used for resource 2 population           */
        /* t bits used for rotation toward the center      */

        // 7 bits used for the id of the kilobot store in the first 10 bits of the message and shift left
        message.id = k_id;
        message.id = message.id << 3;
        // 5 bits to signal are utility and proximity (i.e. 0 ut means over no area)
        // get the state
        KilobotEnvironment::kilobot_arena_state kst = this->kilobots_states[k_id];
        if(kst == INSIDE_AREA_0 || kst == INSIDE_AREA_01 || kst == INSIDE_AREA_02 || kst == INSIDE_AREA_012) {
#ifdef REAL_UTILITY
            uint8_t ut = ceil(resources.at(0)->population/resources.at(0)->areas.size()*31);
#else
            uint8_t ut = ceil(areasUt[0]*31);
#endif
            message.id = message.id | (ut >> 2);
            message.type = ut << 2;
        }
        if(kst == INSIDE_AREA_1 || kst == INSIDE_AREA_01 || kst == INSIDE_AREA_12 || kst == INSIDE_AREA_012) {
#ifdef REAL_UTILITY
            uint8_t ut = ceil(resources.at(1)->population/resources.at(1)->areas.size()*31);
#else
            uint8_t ut = ceil(areasUt[1]*31);
#endif
            message.type = message.type | (ut>>3);
            message.data = ut;
            message.data = message.data << 7;
        }
        if(kst == INSIDE_AREA_2 || kst == INSIDE_AREA_02 || kst == INSIDE_AREA_12 || kst == INSIDE_AREA_012) {
#ifdef REAL_UTILITY
            uint8_t ut = ceil(resources.at(2)->population/resources.at(2)->areas.size()*31);
#else
            uint8_t ut = ceil(areasUt[2]*31);
#endif
            message.data = message.data | (ut << 2);
        }

        // store kb rotation toward the center if the kb is too close to the border
        // this is used to avoid that the kb gets stuck in the wall
        uint8_t turning_in_msg = 0;  // 0 no turn, 1 pi/2, 2 pi, 3 3pi/2
        double distance_from_centre = sqrt(pow(this->kilobots_positions[k_id].x()-ARENA_CENTER,2)+pow(this->kilobots_positions[k_id].y()-ARENA_CENTER,2));
        if(distance_from_centre/ARENA_SIZE > 0.9) {

            // get position translated w.r.t. center of arena
            QVector2D pos = QVector2D(this->kilobots_positions[k_id]);
            pos.setX(ARENA_CENTER - pos.x());
            pos.setY(ARENA_CENTER - pos.y());
            // get orientation (from velocity)
            QVector2D ori = QVector2D(kilobot_entity.getVelocity());
            ori.setX(ori.x()*10);
            ori.setY(ori.y()*10);
            // use atan2 to get angle between two vectors
            double angle = qAtan2(ori.y(), ori.x()) - qAtan2(pos.y(), pos.x());

            if(angle > M_PI*3/4 || angle < -M_PI*3/4) {
                 turning_in_msg = 2;
            } else if(angle < -M_PI/2) {
                turning_in_msg = 3;
            } else if(angle > M_PI/2){
                turning_in_msg = 1;
            }
            // store angle (no need if 0)
            message.data = message.data | turning_in_msg;
        }

        // send it
        emit transmitKiloState(message);
    }
}

#endif // COMPLEXITYENVIRONMENT_CPP
