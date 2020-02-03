/**
 * Custom definition of an area
 * With the term area we refer to the single area (e.g. the single coloured circle)
 * Areas disasppears when the population reaches 0
 *
 * @author Dario Albani
 * @email dario.albani@istc.cnr.it
 */

#ifndef AREA_H
#define AREA_H

#include <math.h>
#include <stdlib.h>

#include <QPointF>
#include <QColor>

// base population for every area
#define BASE_POP 50

class Area {
private:
    // enum for resource color, at max 3 different resource admitted
    QColor enumColor[3] = {
        Qt::green,
        Qt::red,
        Qt::yellow,
    };


public:
    uint type; // resource type
    uint id; // area id

    QPointF position; /* Center of the resource */
    double radius; /* Radius of the circle to plot */
    double population; /* Population in the current area */
    QColor color; /* Color used to represent the area */
    uint kilobots_in_area; /* keep counts of how many kbs are in the area*/
    std::string exploitation_type; /* determine the exploitation on the area by different kbs */



    /* constructor */
    Area() : type(0), id(0), position(QPointF(0,0)), radius(0), exploitation_type("linear") {}

    Area(uint type, uint id, QPointF position, double radius, std::string exploitation_type) :
        type(type), id(id), position(position), radius(radius), exploitation_type(exploitation_type) {
        this->kilobots_in_area = 0;
        this->population = BASE_POP;
        this->color = enumColor[type];
    }

    /* destructor */
    ~Area(){}

    /*
   * do one simulation step during which:
   * - the population is decreased according to the number of agents
   *
   * @return true if a specific the population value reaches 0
   */
    bool doStep() {
        if(this->exploitation_type == "quadratic") {
            this->population -= pow(kilobots_in_area,3);
        } else if(this->exploitation_type == "cubic") {
            this->population -= pow(kilobots_in_area,2);
        } else {
            // it is linear
            this->population -= kilobots_in_area;
        }

        // reset kbs in the area
        this->kilobots_in_area = 0;
        return this->population ==0;
    }
};

#endif // AREA_H
