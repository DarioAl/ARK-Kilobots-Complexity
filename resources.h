
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

#include <math.h>
#include <stdlib.h>
#include <random>
#include <iostream>

#include <QPointF>
#include <QtMath>

#define ETA 0.05
#define K 25
#define UMIN 0.25

class Resource {

public:

    /************************************/
    /* logistic growth                  */
    /************************************/
    double umin; /* population threshold in percentange @see doStep */
    double eta; /* growht factor */
    double k; /* maximum population for single area */

    /************************************/
    /* virtual environment visualization*/
    /************************************/
    uint8_t type; // resource type
    QColor colour; // resource colour associated to the type, red green and blue
    double area_radius;   // the radius of the circle
    uint seq_areas_id; // used to sequentially assign ids to areas
    std::vector<Area> areas; /* areas of the resource */
    uint population; /* Total resource population from0 to k*/

    /************************************/
    /* area exploitation function       */
    /************************************/
    std::string exploitation; /* single area exploitation */

    /* constructor */
    inline Resource() {
        this->type = 0;
        this->colour = Qt::red;
        this->population = 0;
        this->eta = 0.05;
        this->k = 25;
        this->umin = 0.25;
        this->area_radius = 0.035;
        this->seq_areas_id = 0;
    }

    Resource(uint type, double area_radius, double population) {
        this->type = type;
        this->population = population;
        this->eta = ETA;
        this->k = K;
        this->umin = UMIN;
        this->area_radius = area_radius;
        this->seq_areas_id = 0;
        if(type==0)
            this->colour = Qt::red;
        else if(type==1)
            this->colour = Qt::green;
        else if(type==2)
            this->colour = Qt::blue;
    }

    /* destructor */
    ~Resource() {}

   /*
   * generate areas for the resource by taking into account all other areas positions
   */
    void generate(const QVector<Area>& oth_areas, double arena_radius, uint num_of_areas) {
        std::vector<Area> all_areas;
        all_areas.insert(all_areas.end(), oth_areas.begin(), oth_areas.end());
        all_areas.insert(all_areas.end(), this->areas.begin(), this->areas.end());

        QPointF pos;
        uint tries = 0;         // placement try
        uint maxTries = 9999;   // max placement tries

        for(uint i=0; i<num_of_areas; i++) {
            for(tries=0; tries <= maxTries; tries++) {
                // find a possible placement inside the arena
                do {
                    double rand_angle = getRandDouble(M_PI, -M_PI);
                    double rand_displacement_x = getRandInt(0, arena_radius-area_radius/2);
                    double rand_displacement_y = getRandInt(0, arena_radius-area_radius/2);

                    pos = QPoint(rand_displacement_x*cos(rand_angle),
                                    rand_displacement_y*sin(rand_angle));
                } while(SquaredDistance(pos.x(),0,pos.y(),0) < pow(arena_radius-area_radius/2,2));

                bool duplicate = false;
                for(const Area& an_area : all_areas) {
                    duplicate = SquaredDistance(an_area.position.x(),pos.x(),an_area.position.y(),pos.y()) <= pow(area_radius*2,2);
                    if(duplicate)
                        break;
                }

                if(!duplicate) {
                    Area new_area(this->type, seq_areas_id, pos, area_radius, exploitation);
                    // add new area to the list of all areas for further comparisons
                    all_areas.push_back(new_area);
                    // save new area for simulation
                    areas.push_back(new_area);
                    break;
                }

                // too many tries, there is no valid spot
                if(tries >= maxTries-1) {
                     std::cout << "ERROR while placing a new area. No valid spot has been found." << std::endl;
                }

            }
        }
    }

    /*
   * do one simulation step during which:
   * - the population is increased according to a logistic function
   * - the population is exploited according to the kilobots over it
   *
   * @return true if a specific umin value is reached
   */
    bool doStep(const QVector<QPointF>& kilobot_positions, const QVector<uint8_t>& kilobot_states,
                const QVector<QColor>& kilobot_colors, const QVector<Area>& oth_areas, double arena_radius) {
        // update kilobots positions in the areas, compute only for those kilobots with the correct state
        for(int i=0; i<kilobot_positions.size(); i++) {
            if(kilobot_states.at(i) == this->type && kilobot_colors.at(i) == lightColour::GREEN) {
                // the kilobot is working on the area
                // find and update it
                for(Area& area : areas) {
                    if(SquaredDistance(kilobot_positions.at(i).x(),area.position.x(),kilobot_positions.at(i).y(),area.position.y()) < pow(area_radius, 2)) {
                        area.kilobots_in_area++;
                        break;
                    }
                }
            }
        }

        // after the update of the areas then apply exploitation
        std::vector<Area>::iterator it = areas.begin();
        while(it != areas.end()) {
            if(it->doStep()) {
                // delete if do step return true
                it = areas.erase(it);
                population = areas.size();
            }
            // else increase iterator
            ++it;
        }

        // apply growth
        population += population*eta*(1-population/k);

        // regenerate areas
        uint diff = floor(population) - areas.size();
        if(diff>0) {
            this->generate(oth_areas, arena_radius, diff);
        }

        return population < population*umin;
    }

private:
    int getRandInt(int min, int max) {
        unsigned int seed = static_cast<unsigned>(qrand());
        std::mt19937 gen(seed);
        std::uniform_int_distribution<> uid(min, max);
        return uid(gen);
    }

    int getRandDouble(double min, double max) {
        unsigned int seed = static_cast<unsigned>(qrand());
        std::mt19937 gen(seed);
        std::uniform_int_distribution<> uid(min, max);
        return uid(gen);
    }

    double SquaredDistance(double x1, double x2, double y1, double y2) {
        return pow(x1-x2,2) + pow(y1-y2,2);
    }
};
#endif // RESOURCES_H
