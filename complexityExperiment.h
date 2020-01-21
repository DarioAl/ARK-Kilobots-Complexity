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
    kilobot_id id;          // unique kilobot id
    QPointF position;       // kilobot position on the plane
    double orientation;     // kilobot orientation
    kilobot_colour colour;  // kilobot led colour

    KiloLog() {}
    KiloLog(kilobot_id id, QPointF position, double orientation, kilobot_colour colour) :
        id(id), position(position), orientation(orientation), colour(colour) {}

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

    void setResourceAPopulation(int numOfAreas);  // set resource A population as number of areas
    void setResourceBPopulation(int numOfAreas);  // set resource B population as number of areas
    void setResourceCPopulation(int numOfAreas);  // set resource C population as number of areas
    void setResourceAEta(double eta); // set resource A growth rate
    void setResourceBEta(double eta); // set resource B growth rate
    void setResourceCEta(double eta); // set resource C growth rate
    void setResourceAK(int k);   // set resource A population cap as number of areas
    void setResourceBK(int k);   // set resource B population cap as number of areas
    void setResourceCK(int k);   // set resource C population cap as number of areas
    void setResourceAUmin(double umin);    // set resource A minimum population for exploitation as percentage
    void setResourceBUmin(double umin);    // set resource B minimum population for exploitation as percentage
    void setResourceCUmin(double umin);    // set resource C minimum population for exploitation as percentage

    QColor GetFloorColor(int x, int y);

private:
    void updateKilobotState(Kilobot kilobotCopy);
    void setupInitialKilobotState(Kilobot kilobot_entity);

    void setupEnvironments();
    void plotEnvironments();

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
    QDoubleSpinBox *eta_spina, *eta_spinb, *eta_spinc;
    QDoubleSpinBox *k_spina, *k_spinb, *k_spinc;
    QDoubleSpinBox *umin_spina, *umin_spinb, *umin_spinc;

    // kilobots objects
    QVector<kilobot_id> kilobots_ids;
    QVector<KiloLog> kilobots;

}; /* end class mykilobotexperiment */

#endif // COMPLEXITYEXPERIMENT_H
