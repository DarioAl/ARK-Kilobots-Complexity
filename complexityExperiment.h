#ifndef COMPLEXITYEXPERIMENT_H
#define COMPLEXITYEXPERIMENT_H

/**
 * Author: Dario Albani
 *
 * This is the main file for the experiment that implements the template functions in the kilobot*.h files
 * and that are need for ARK to correctly execute the experiment.
 *
 * The complexity experiment consist in a swarm of kilobots exploiting several resources (up to three) present
 * in the environment. Each resource consiste of k (k=25) areas that the kilobots can exploit as single or group.
 *
 * A kilobot can assume three states indicated by three different led colours:
 * - uncommitted - red light - during which a kb estimates the resoruces and performs a random walk
 * - committed to a resource but not working on an area - blue light - during which the kb performs a random walk and seeks for an area belongig to the wanted resource
 * - committed to a resource and working on an area - green light - during which the kb stands still over and area and it is considered as working on it (i.e. exploting the area)
 */
#include "global.h"

#include <QObject>
#include <QFile>

// these are the templates that should be used for ARK
#include "kilobot.h"
#include "kilobotexperiment.h"
#include "kilobotenvironment.h"
#include "complexityEnvironment.h"

// there are the file for the complexity experiment
#include "resources.h"
#include "area.h"

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Qt includes
#include <QObject>
#include <QFile>
#include <QList>
#include <QTableWidget>
#include <QSpinBox>
#include <QFormLayout>
#include <QPushButton>
#include <QCheckBox>
#include <QScrollBar>
#include <QSlider>
#include <QGroupBox>
#include <QFormLayout>
#include <QLabel>
#include <QFrame>
#include <QtMath>
#include <QElapsedTimer>

/**
 * @brief Log kilobot related information, Values are defined in the template file kilobot.h
 */
class KiloLog {
public:
    KiloLog() {}
    KiloLog(kilobot_id id, QPointF position, double orientation, kilobot_colour colour) :
        id(id), position(position), orientation(orientation), colour(colour) {}

    kilobot_id id;          // unique kilobot id
    QPointF position;       // kilobot position on the plane
    double orientation;     // kilobot orientation
    kilobot_colour colour;  // kilobot led colour

    /** Update all log values */
    void updateAllValues(kilobot_id id, QPointF position, double orientation, kilobot_colour colour) {
        this->id = id;
        this->position = position;
        this->orientation = orientation;
        this->colour = colour;
    }

    /** Set position */
    void setPosition(QPointF position) {
        this->position = position;
    }

    /** Set orientation */
    void setOrientation(double orientation) {
        this->orientation = orientation;
    }

    void setColour(kilobot_colour colour) {
        this->colour = colour;
    }
}; /* end class kilo_log */

/**
 * @brief mykilobotexperiment is where the complexity experiment is defined and ARK templates area extended
 * This create a separate window in the ARK GUI where one can set up experiments variables.
*/
class COMPLEXITYEXPERIMENTEXPSHARED_EXPORT mykilobotexperiment : public KilobotExperiment {
    Q_OBJECT

public:
    mykilobotexperiment();
    virtual ~mykilobotexperiment() {}

    QWidget *createGUI();
    QVBoxLayout *lay = new QVBoxLayout;

// signals and slots are used by qt to signal state changes to objects
signals:
    void errorMessage(QString);
public slots:
    void initialise(bool);
    void run();
    void stopExperiment();

    void toggleSaveImages(bool toggle) {
        saveImages = toggle;
    }
    void toggleLogExp(bool toggle) {
        logExp = toggle;
    }

    // set resource growth rate
    inline void setResourceAEta(double eta) {
        complexityEnvironment.resources[0].eta = eta;
    }
    inline void setResourceBEta(double eta) {
        complexityEnvironment.resources[1].eta = eta;
    }
    inline void setResourceCEta(double eta) {
        complexityEnvironment.resources[2].eta = eta;
    }

    // set resource C population cap as number of areas
    inline void setResourceAK(int k) {
        complexityEnvironment.resources[0].k = k;
    }
    inline void setResourceBK(int k) {
        complexityEnvironment.resources[1].k = k;
    }
    inline void setResourceCK(int k) {
        complexityEnvironment.resources[2].k = k;
    }

    // set resource minimum population for exploitation as percentage
    inline void setResourceAUmin(double umin) {
        complexityEnvironment.resources[0].umin = umin;
    }
    inline void setResourceBUmin(double umin) {
        complexityEnvironment.resources[1].umin = umin;
    }
    inline void setResourceCUmin(double umin) {
        complexityEnvironment.resources[2].umin = umin;
    }

    QColor GetFloorColor(int x, int y);

private:
    void updateKilobotState(Kilobot kilobotCopy);
    void setupInitialKilobotState(Kilobot kilobot_entity);

    void setupEnvironments();
    void plotEnvironment();

    void printTotalExploitaion();

    mykilobotenvironment complexityEnvironment;

    // loggin variables
    bool saveImages;
    int savedImagesCounter;
    bool logExp;
    QFile log_file;
    QString log_filename_prefix = "log_complexity";
    QTextStream log_stream;

    // GUI objects

    QSpinBox *pop_spina, *pop_spinb, *pop_spinc;
    QDoubleSpinBox *eta_spina, *eta_spinb, *eta_spinc;
    QSpinBox *k_spina, *k_spinb, *k_spinc;
    QDoubleSpinBox *umin_spina, *umin_spinb, *umin_spinc;

    // kilobots objects
    QVector<kilobot_id> kilobots_ids;
    QVector<KiloLog> kilobots;

}; /* end class mykilobotexperiment */

#endif // COMPLEXITYEXPERIMENT_H
