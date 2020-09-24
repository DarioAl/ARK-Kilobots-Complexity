/**
 * A resource is represented by a group of areas.
 * i.e. a group of dots around the environment that shares the same type
 * A resource has a cap and a population (the number of areas) that grows according to
 * a logistic function.
 *
 * @author Dario Albani
 * @email dario.albani@istc.cnr.it
 */

#ifndef RESOURCES_H
#define RESOURCES_H

#include "area.h"
#include "kilobot.h"
#include "kilobotenvironment.h"

#include <math.h>
#include <stdlib.h>
#include <random>
#include <iostream>

#include <QPointF>
#include <QtMath>

class Resource {

public:

    /************************************/
    /* logistic growth                  */
    /************************************/
    double umin; /* population threshold in percentange @see doStep */
    double eta; /* growht factor */
    uint k; /* number of areas in the resource */

    /************************************/
    /* virtual environment visualization*/
    /************************************/
    uint8_t type; // resource type
    QColor colour; // resource colour associated to the type, red green and blue
    double area_radius;   // the radius of the circle
    uint seq_areas_id; // used to sequentially assign ids to areas
    std::vector<Area*> areas; /* areas of the resource */
    double population; /* Total resource population from 0 to 1 */

    /************************************/
    /* area exploitation function       */
    /************************************/
    std::string exploitation; /* single area exploitation */
    double totalExploitation; /* total exploitation over the whole simulation */
    /* constructor */
    Resource() {
       this->type = 0;
        this->colour = QColor(Qt::red);
        this->population = 0.0;
        this->eta = 0.008424878;
        this->k = 10;
        this->umin = 0.6;
        this->area_radius = 150;
        this->seq_areas_id = 0;

        re.seed(qrand());
    }

    Resource(uint type, double arena_radius, double area_radius, double population, QVector<Area>& oth_areas) {
        this->type = type;
        this->population = population;
        this->eta = 0.008424878;
        this->k = 10;
        this->umin = 0.6;
        this->area_radius = area_radius;
        this->seq_areas_id = 0; // not used anywhere (remove?)
        this->exploitation = "quadratic";
        re.seed(qrand());



        if(type==0)
            this->colour = QColor(Qt::red);
        else if(type==1)
            this->colour = QColor(Qt::green);
        else if(type==2)
            this->colour = QColor(Qt::blue);

        this->generate(oth_areas, arena_radius, this->k*this->population);
    }

    /* destructor */
    ~Resource() {}

   /*
   * generate areas for the resource by taking into account all other areas positions
   */
    void generate(QVector<Area>& oth_areas, double arena_radius, uint num_of_areas) {
        uint tries = 0;         // placement tries
        uint maxTries = 9999;   // max placement tries

        for(uint i=0; i<num_of_areas; i++) {
            for(tries=0; tries <= maxTries; tries++) {
                QPointF pos;
                // find a possible placement inside the arena (750 is the center of the image)
                pos.setX(getRandDouble(750-arena_radius,750+arena_radius));
                pos.setY(getRandDouble(750-arena_radius,750+arena_radius));

                // if not within the circle break
                if(pow(pos.x()-750,2)+pow(pos.y()-750,2) < pow(arena_radius-area_radius,2)) {
                    bool overlaps = false;
                    // check if overlaps with an area in the same resource
                    for(Area* my_area : this->areas) {
                        QPointF point = my_area->position;
                        if(pow(point.x()-pos.x(),2)+pow(point.y()-pos.y(),2) <= pow(area_radius*2,2)){
                            overlaps = true;
                            break;
                        }
                    }

                    if(!overlaps){
                        // create the area
                        Area* new_area = new Area(this->type, seq_areas_id, pos, area_radius, exploitation);
                        seq_areas_id++;
                        // save new area for simulation
                        areas.push_back(new_area);
                        // add to oth areas for other areas generation
                        oth_areas.push_back(*new_area);
                        break;
                    }
                }
            }
        }
    }

    /*
   * do one simulation step during which:
   * - the population is increased according to a logistic function
   * - the population is exploited according to the kilobots over it
   *
   * @return TODO
   */
    bool doStep() {
        // reset before update
        this->population = 0;
        // after the update of the areas then apply exploitation
        for(Area* a : areas) {
            this->totalExploitation += a->doStep();
            this->population += a->population;
        }
        // normalize between 0 and 1
        this->population = this->population/this->areas.size();

        return false;
    }

private:
    unsigned int seed;
    std::default_random_engine re;

    int getRandInt(int min, int max) {
        std::uniform_int_distribution<> uid(min, max);
        return uid(re);
    }

    double getRandDouble(double min, double max) {
        std::uniform_real_distribution<double> urd(min, max);
        return urd(re);
    }
};
#endif // RESOURCES_H
