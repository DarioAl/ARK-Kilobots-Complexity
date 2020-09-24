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
#include <iostream>
class Area {
public:
    uint type; // resource type
    uint id; // area id

    QPointF position; /* Center of the resource */
    double radius; /* Radius of the circle to plot */
    double population; /* Population in the current area */
    QColor color; /* Color used to represent the area */
    uint kilobots_in_area; /* keep counts of how many kbs are in the area*/
    std::string exploitation_type; /* determine the exploitation on the area by different kbs */
    double lambda; /* epxloitation coefficient */
    double eta; /* area growth */

    /* constructor */
    Area() : type(0), id(0), position(QPointF(0,0)), radius(0), exploitation_type("quadratic") {}

    Area(uint type, uint id, QPointF position, double radius, std::string exploitation_type) :
        type(type), id(id), position(position), radius(radius), exploitation_type(exploitation_type) {
        this->kilobots_in_area = 0;
        this->population = 1;
        this->lambda = 0.005;
        this->eta = 0.008424878;

        if(type == 0)
            this->color = Qt::red;
        else if(type == 1)
            this->color = Qt::green;
        else
            this->color = Qt::blue;
    }

    /* destructor */
    ~Area(){}

    /* check if the point is inside the area */
    bool isInside(QPointF point) {
       return pow(point.x()-position.x(),2)+pow(point.y()-position.y(),2) <= pow(radius,2);
    }

    /*
   * do one simulation step during which:
   * - the population is decreased according to the number of agents
   *
   * @return exploitation
   */
    double doStep() {
        // compute exploitation
        double exploitation = 0;
        if(this->exploitation_type == "cubic") {
            exploitation = this->population*lambda*pow(kilobots_in_area,3);
        } else if(this->exploitation_type == "quadratic") {
            exploitation = this->population*lambda*pow(kilobots_in_area,2);
        } else {
            exploitation = this->population*lambda*kilobots_in_area;
        }

        // compute growth
        double growth = this->population*eta*(1-this->population);

        // sum and subtract
        this->population = this->population - exploitation + growth;

        // trick to restore areas that went to low (usually due to bad led readings)
        if(this->population <= 0.0001) {
            this->population = 0.001;
        }

        // reset kbs in the area
        this->kilobots_in_area = 0;

        //std::cout << "utility is " << population;
        //std::cout << "exploitation is " << exploitation << " growth is " << growth << std::endl;
        // return exploitation
        return exploitation;
    }
};

#endif // AREA_H
