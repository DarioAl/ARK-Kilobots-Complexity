
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

#define ETA 0.005
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
    double population; /* Total resource population from 0 to 1 */

    /************************************/
    /* area exploitation function       */
    /************************************/
    std::string exploitation; /* single area exploitation */

    /* constructor */
    inline Resource() {
        this->type = 0;
        this->colour = QColor(Qt::red);
        this->population = 0.0;
        this->eta = ETA;
        this->k = K;
        this->umin = UMIN;
        this->area_radius = 35;
        this->seq_areas_id = 0;

        re.seed(qrand());
    }

    Resource(uint type, double arena_radius, double area_radius, double population, QVector<Area>& oth_areas) {
        this->type = 0;
        this->eta = ETA;
        this->k = K;
        this->umin = UMIN;
        this->population = population;
        this->area_radius = area_radius;
        this->seq_areas_id = 0;
        if(type==0)
            this->colour = QColor(Qt::red);
        else if(type==1)
            this->colour = QColor(Qt::green);
        else if(type==2)
            this->colour = QColor(Qt::blue);

        re.seed(qrand());

        generate(oth_areas, arena_radius, k*this->population);
    }

    /* destructor */
    ~Resource() {}

   /*
   * generate areas for the resource by taking into account all other areas positions
   */
    void generate(QVector<Area>& oth_areas, double arena_radius, uint num_of_areas) {
        std::cout << "asking to generate " << num_of_areas << std::endl;
        uint tries = 0;         // placement tries
        uint maxTries = 9999;   // max placement tries

        for(uint i=0; i<num_of_areas; i++) {
            for(tries=0; tries <= maxTries; tries++) {
                QPointF pos;
                // find a possible placement inside the arena
                double rand_angle = getRandDouble(0, 2*M_PI);
                double rand_displacement_x = getRandDouble(0, arena_radius);
                double rand_displacement_y = getRandDouble(0, arena_radius);
                pos.setX(rand_displacement_x*cos(rand_angle));
                pos.setY(rand_displacement_y*sin(rand_angle));
                // translate to center w.r.t. the image
                pos.rx()+=1000;
                pos.ry()+=1000;

                // find a possible placement on empty space
                bool duplicate = false;
                for(const Area& an_area : oth_areas) {
                    // greater than diameter plus thickness of the drawing
                    double distance = sqrt(pow(an_area.position.x()-pos.x(),2)+pow(an_area.position.y()-pos.y(),2));
                    duplicate = (bool) (distance < area_radius*2.2);

                    if(duplicate) {
                        break;
                    }
                }

                if(!duplicate) {
                    Area new_area(this->type, seq_areas_id, pos, area_radius, exploitation);
                    seq_areas_id++;
                    // save new area for simulation
                    areas.push_back(new_area);
                    // add to oth areas for other areas generation
                    oth_areas.push_back(new_area);
                    break;
                }

                // too many tries, there is no valid spot
                if(tries >= maxTries-1) {
                     std::cout << "ERROR while placing a new area. No valid spot has been found." << std::endl;
                     exit(-1);
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
                const QVector<QColor>& kilobot_colors, QVector<Area>& oth_areas, double arena_radius) {
        // update kilobots positions in the areas, compute only for those kilobots with the correct state
        for(int i=0; i<kilobot_positions.size(); i++) {
            if(kilobot_states.at(i) == this->type && kilobot_colors.at(i) == lightColour::GREEN) {
                // the kilobot is working on the area
                // find and update it
                for(Area& area : areas) {
                    QPointF kbp = kilobot_positions.at(i);
                    QPointF ap = area.position;
                    if(pow(kbp.x()-ap.x(),2)+pow(kbp.y()-ap.y(),2) < pow(area_radius, 2)) {
                        area.kilobots_in_area++;
                        std::cout << "there is one kb in the area" << std::endl;
                        // TODO activate the kb and the leds and check if the logic works
                        // if it works fuck this shit its done!
                        break;
                    }
                }
            }
        }

        // check if the value of k has been changed from the GUI and if any
        // check if there are more areas than allowed and remove those in excess
        if(areas.size() > population*k) {
            uint diff = areas.size()-population*k;
            while(diff) {
                areas.pop_back();
                diff--;
            }
        }

        // after the update of the areas then apply exploitation
        std::vector<Area>::iterator it = areas.begin();
        while(it != areas.end()) {
            if(it->doStep()) {
                // delete if do step return true
                it = areas.erase(it);
                population = areas.size()/k;
            }
            // else increase iterator
            ++it;
        }

        // apply growth
        population += population*eta*(1-population);

        // regenerate areas
        uint diff = double(k)*population-areas.size();
        if(diff>0) {
            this->generate(oth_areas, arena_radius, diff);
        }

        return population < umin;
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
