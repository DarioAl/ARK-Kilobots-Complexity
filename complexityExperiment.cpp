#include "complexityExperiment.h"
#include "complexityEnvironment.h"

#include <QDebug>
#include <QThread>

// widgets
#include <QPushButton>
#include <QCheckBox>
#include <QScrollBar>
#include <QSlider>
#include <QGroupBox>
#include <QFormLayout>
#include <QLabel>
#include <QFrame>
#include <QtMath>
#include <QRadioButton>
#include <QPainter>
#include <QList>
#include <iterator>
#include <QSignalMapper>
#include <QFile>

#define STOP_AFTER 3600 + 1440
#define SAVE_IMAGE_EVERY 5
#define SAVE_LOG_EVERY 50

// return pointer to interface!
// mykilobotexperiment can and should be completely hidden from the application
extern "C" COMPLEXITYEXPERIMENTEXPSHARED_EXPORT KilobotExperiment *createExpt()
{
    return new mykilobotexperiment();
}

/* setup the environment */
mykilobotexperiment::mykilobotexperiment() {
    // qDebug() << QString("in constructor");

    // Initialize seed
    QDateTime cd = QDateTime::currentDateTime();
    qsrand(cd.toTime_t());

    // setup the environment here
    connect(&complexityEnvironment,SIGNAL(transmitKiloState(kilobot_message)), this, SLOT(signalKilobotExpt(kilobot_message)));
    this->serviceInterval = 100; // timestep expressed in ms
}

/* create the GUI as a separate frame in the main ARK window */
QWidget *mykilobotexperiment::createGUI() {
    // qDebug() << QString("in create gui");

    QFrame *frame = new QFrame;
    frame->setLayout(lay);

    // add check box for saving the images
    QCheckBox *saveImages_ckb = new QCheckBox("Record experiment");
    saveImages_ckb->setChecked(false);  // start as not checked
    lay->addWidget(saveImages_ckb);
    toggleSaveImages(saveImages_ckb->isChecked());

    // add check box for logging the experiment
    QCheckBox *logExp_ckb = new QCheckBox("Log experiment");
    logExp_ckb->setChecked(true);   // start as checked
    lay->addWidget(logExp_ckb);
    toggleLogExp(logExp_ckb->isChecked());

    // create a box for resource parameters as following
    // Resource A:
    //   eta [     ]
    //     k [     ]
//    QFormLayout *layoutResourceA = new QFormLayout;
//    QGroupBox *resourceAParameters = new QGroupBox(tr("Resource A"));
//    resourceAParameters->setLayout(layoutResourceA);

//    // eta
//    eta_spina = new QDoubleSpinBox();
//    eta_spina->setMinimum(0);
//    eta_spina->setMaximum(1);
//    eta_spina->setSingleStep(0.001);
//    eta_spina->setValue(complexityEnvironment.resources.at(0).eta);
//    layoutResourceA->addRow(new QLabel(tr("eta")), eta_spina);

    // k
//    k_spina = new QSpinBox();
//    k_spina->setMinimum(0);
//    k_spina->setMaximum(50);
//    k_spina->setSingleStep(1);
//    k_spina->setValue(complexityEnvironment.resources.at(0).k);
//    layoutResourceA->addRow(new QLabel(tr("k")), k_spina);
//    lay->addWidget(resourceAParameters);

    // create a box for resource parameters as following
    // Resource B:
    //   eta [     ]
    //     k [     ]
//    QFormLayout *layoutResourceB = new QFormLayout;
//    QGroupBox *resourceBParameters = new QGroupBox(tr("Resource B"));
//    resourceBParameters->setLayout(layoutResourceB);

//    // eta
//    eta_spinb = new QDoubleSpinBox();
//    eta_spinb->setMinimum(0);
//    eta_spinb->setMaximum(1);
//    eta_spinb->setSingleStep(0.001);
//    eta_spinb->setValue(complexityEnvironment.resources.at(1).eta);
//    layoutResourceB->addRow(new QLabel(tr("eta")), eta_spinb);
    // k
//    k_spinb = new QSpinBox();
//    k_spinb->setMinimum(0);
//    k_spinb->setMaximum(50);
//    k_spinb->setSingleStep(1);
//    k_spinb->setValue(complexityEnvironment.resources.at(1).k);
//    layoutResourceB->addRow(new QLabel(tr("k")), k_spinb);
//    lay->addWidget(resourceBParameters);

    // create a box for resource parameters as following
    // Resource C:
    //   eta [     ]
    //     k [     ]
//    QFormLayout *layoutResourceC = new QFormLayout;
//    QGroupBox *resourceCParameters = new QGroupBox(tr("Resource C"));
//    resourceCParameters->setLayout(layoutResourceC);

//    // eta
//    eta_spinc = new QDoubleSpinBox();
//    eta_spinc->setMinimum(0);
//    eta_spinc->setMaximum(1);
//    eta_spinc->setSingleStep(0.001);
//    eta_spinc->setValue(complexityEnvironment.resources.at(2).eta);
//    layoutResourceC->addRow(new QLabel(tr("eta")), eta_spinc);
    // k
//    k_spinc = new QSpinBox();
//    k_spinc->setMinimum(0);
//    k_spinc->setMaximum(50);
//    k_spinc->setSingleStep(1);
//    k_spinc->setValue(complexityEnvironment.resources.at(2).k);
//    layoutResourceC->addRow(new QLabel(tr("k")), k_spinc);
//    lay->addWidget(resourceCParameters);

//    connect(eta_spina, SIGNAL(valueChanged(double)),this, SLOT(setResourceAEta(double)));
//    connect(eta_spinb, SIGNAL(valueChanged(double)),this, SLOT(setResourceBEta(double)));
//    connect(eta_spinc, SIGNAL(valueChanged(double)),this, SLOT(setResourceCEta(double)));

//    connect(k_spina, SIGNAL(valueChanged(int)),this, SLOT(setResourceAK(int)));
//    connect(k_spinb, SIGNAL(valueChanged(int)),this, SLOT(setResourceBK(int)));
//    connect(k_spinc, SIGNAL(valueChanged(int)),this, SLOT(setResourceCK(int)));

    connect(saveImages_ckb, SIGNAL(toggled(bool)),this, SLOT(toggleSaveImages(bool)));
    connect(logExp_ckb, SIGNAL(toggled(bool)),this, SLOT(toggleLogExp(bool)));
    connect(this,SIGNAL(destroyed(QObject*)), lay, SLOT(deleteLater()));

    return frame;
}

void mykilobotexperiment::initialise(bool isResume) {
    //qDebug() << QString("in initialise");

    // generate the environments
    setupEnvironments();

    // initialize kilobot states
    if(!isResume) {
        emit getInitialKilobotStates();
    }

    // Pos = position, ROT = orientation, LED = color
    emit setTrackingType(POS | ROT | LED);

    QThread::currentThread()->setPriority(QThread::HighestPriority);

    savedImagesCounter = 0;
    this->time = 0;

    // init log file operations
    // if the log checkmark is marked then save the logs
    if(logExp) {
        // open file
        if(log_file.isOpen()) {
            // if it was open close and re-open again later
            // this erase the old content
            log_file.close();
        }
        // log filename consist of the prefix and current date and time
        QString log_filename = log_filename_prefix + "_" + QDate::currentDate().toString("yyMMdd") + "_" + QTime::currentTime().toString("hhmmss") + ".txt";
        log_file.setFileName(log_filename);
        // open the file
        if(log_file.open(QIODevice::WriteOnly)) {
            qDebug() << "Log file " << log_file.fileName() << " opened";
            log_stream.setDevice(&log_file);
        } else {
            qDebug() << "ERROR opening file "<< log_filename;
        }
    }

    // if the checkbox for saving the images is checked
    if(saveImages) {
        emit saveImage(QString("complexity_%1.jpg").arg(savedImagesCounter++, 5, 10, QChar('0')));
    }

    clearDrawings();
}

void mykilobotexperiment::stopExperiment() {
    // close log file
    if(log_file.isOpen()) {
        qDebug() << "Closing log file " << log_file.fileName();
        log_file.close();
    }
}

void mykilobotexperiment::run() {
    //qDebug() << QString("in run");

    this->time += 0.1; // 10 ms

    // stop after given time
    if(this->time >= STOP_AFTER) {
        // close the experiment
        this->stopExperiment();
        emit(experimentComplete());
    }

    // update environment
    qDebug() << "-----" << time;
    qDebug() << "in update at time " << time;
    qDebug() << "isCom " << complexityEnvironment.isCommunicationTime << " time elapsed " << this->time - complexityEnvironment.lastTransitionTime;
    // switch between communication time and exploration time
    if(!complexityEnvironment.isCommunicationTime && EXPLORATION_TIME <= this->time - complexityEnvironment.lastTransitionTime) {
        complexityEnvironment.isCommunicationTime = true;
        complexityEnvironment.lastTransitionTime = this->time;
        kilobot_broadcast message;
        message.type = 2; // 2 "communicate"
        emit broadcastMessage(message);
    } else if(complexityEnvironment.isCommunicationTime && COMMUNICATION_TIME <= this->time - complexityEnvironment.lastTransitionTime) {
        complexityEnvironment.isCommunicationTime = false;
        complexityEnvironment.lastTransitionTime = this->time;
        kilobot_broadcast message;
        message.type = 3; // 3 "stop communications"
        emit broadcastMessage(message);
    }

    // emit continuosly the "communicate" or "stop communication message"
    // do this only three time per second to avoid interferences
    if(uint(this->time*10)%4) {
        kilobot_broadcast message;
        message.type = complexityEnvironment.isCommunicationTime?2:3; //2 "communicate" 3 "stop communications"
        emit broadcastMessage(message);
    }

    complexityEnvironment.time = (float)time;
    complexityEnvironment.ongoingRuntimeIdentification = this->runtimeIdentificationLock;
    complexityEnvironment.update();

    // update kilobots states
    emit updateKilobotStates();

    // update visualization twice per second
    if(qRound(this->time*10)%5 == 0) {
        // clear current environment
        clearDrawings();
        clearDrawingsOnRecordedImage();

        // plot updated environment
        plotEnvironment();
    }

    // if in communication time do not save image and the log
    if(!complexityEnvironment.isCommunicationTime) {
        // save image and log
        if(qRound(this->time*10)%SAVE_IMAGE_EVERY == 0) {
            if(saveImages) {
                emit saveImage(QString("complexity_%1.jpg").arg(savedImagesCounter++, 5, 10, QChar('0')));
            }
        }
        if(qRound(this->time*10)%SAVE_LOG_EVERY == 0) {
            qDebug() << "LOG: saving at " << this->time*10;
            // log kilobot positions
            if(logExp) {
                // count kilobots
                uint8_t committed0 = 0;
                uint8_t committed1 = 0;
                uint8_t committed2 = 0;
                uint8_t uncommitted = 0;
                for(int i=0; i<kilobots_ids.size(); ++i) {
                    kilobot_id k_id = kilobots_ids.at(i);
                    if(complexityEnvironment.kilobots_colours.at(k_id) == Qt::red) {
                        committed0++;
                    } else if(complexityEnvironment.kilobots_colours.at(k_id) == Qt::blue) {
                        committed1++;
                    } else if(complexityEnvironment.kilobots_colours.at(k_id) == Qt::blue) {
                        committed2++;
                    } else {
                        uncommitted++;
                    }
                    log_stream
                            << complexityEnvironment.resources.at(0)->population << " " << committed0
                            << complexityEnvironment.resources.at(1)->population << " " << committed0
                            << complexityEnvironment.resources.at(2)->population << " " << committed0
                            << uncommitted;
                }
                log_stream << endl;
            }
        }
    }
}

void mykilobotexperiment::setupInitialKilobotState(Kilobot kilobot_entity) {
    //qDebug() << QString("in setup init kilobot state");

    // assign all kilobot to environment complexity
    this->setCurrentKilobotEnvironment(&complexityEnvironment);
    kilobot_id k_id = kilobot_entity.getID();

    // create a necessary list and variable for correct message timing
    if(k_id+1 > kilobots.size()) {
        kilobots.resize(k_id+1);
    }

    // this are used to solve a bug causing the app to crash after resetting
    if(complexityEnvironment.lastSent.size() < k_id+1) {
        complexityEnvironment.lastSent.resize(k_id+1);
    }
    if(complexityEnvironment.kilobots_positions.size() < k_id+1) {
        complexityEnvironment.kilobots_positions.resize(k_id+1);
    }
    if(complexityEnvironment.kilobots_states.size() < k_id+1) {
        complexityEnvironment.kilobots_states.resize(k_id+1);
    }
    if(complexityEnvironment.kilobots_colours.size() < k_id+1) {
        complexityEnvironment.kilobots_colours.resize(k_id+1);
    }


    complexityEnvironment.lastSent[k_id] = complexityEnvironment.minTimeBetweenTwoMessages;

    // TODO initialize kilobots location correctly
    complexityEnvironment.kilobots_positions[k_id] = kilobot_entity.getPosition();
    complexityEnvironment.kilobots_states[k_id] = (KilobotEnvironment::kilobot_arena_state)255;
    complexityEnvironment.kilobots_colours[k_id] = Qt::black;

    KiloLog kLog(k_id, kilobot_entity.getPosition(), 0, kilobot_entity.getLedColour());
    kilobots[k_id] = kLog;
    if(!kilobots_ids.contains(k_id))
        kilobots_ids.append(k_id);

    double timeForAMessage = 0.05; // 50 ms each message
    complexityEnvironment.minTimeBetweenTwoMessages = kilobots_ids.size()*timeForAMessage/2.8;
    complexityEnvironment.lastSent[k_id] = complexityEnvironment.minTimeBetweenTwoMessages;
}

void mykilobotexperiment::updateKilobotState(Kilobot kilobotCopy) {
    //qDebug() << QString("in update kilobot state");

    // update values for logging
    if(logExp && (qRound(time*10)%SAVE_LOG_EVERY == 0)) {
        kilobot_id k_id = kilobotCopy.getID();
        kilobot_colour k_colour = kilobotCopy.getLedColour();
        QPointF k_position = kilobotCopy.getPosition();
        double k_rotation = qRadiansToDegrees(qAtan2(-kilobotCopy.getVelocity().y(), kilobotCopy.getVelocity().x()));
        kilobots[k_id].updateAllValues(k_id, k_position, k_rotation, k_colour);
    }
}

void mykilobotexperiment::setupEnvironments() {
    //qDebug() << QString("in setup env");

    complexityEnvironment.reset();
}

QColor mykilobotexperiment::GetFloorColor(int track_x, int track_y) {
    //qDebug() << QString("in get floor color");

    QColor floorColour = Qt::white; // no resource
    // paint the resources
    for(Resource* r : complexityEnvironment.resources) {
        for(Area* a : r->areas) {
            if(a->isInside(QPointF(track_x,track_y))) {
                floorColour = r->colour;
                break;
            }
        }
        if(floorColour != Qt::white)
            break;
    }
    return floorColour;
}

void mykilobotexperiment::plotEnvironment() {
    // if in communication time do not save the images (clean video)
    if(complexityEnvironment.isCommunicationTime) {
        return;
    }

    // clean image
    clearDrawingsOnRecordedImage();

    // center, radius, color, thikness, text, dunno
    drawCircle(QPointF(750,750), 13, QColor(Qt::yellow), 25, "", true);
    drawCircle(QPointF(750,750), 735, QColor(Qt::yellow), 25, "", true);
    uint8_t size = 10;
    // print areas as circles
    for(const Resource* r : complexityEnvironment.resources) {
        for(const Area* a : r->areas) {
            char apop[3];
            sprintf(apop, "%d", (int)(a->population*100));
            drawCircle(a->position, a->radius, r->colour, 15, apop, true);

            if(this->saveImages) {
                // draw a inner gray circle if below umin
                if(a->population < 0.6) {
                    drawCircleOnRecordedImage(a->position, a->radius-(size*a->population/2), Qt::gray, 5, apop);
                }
                drawCircleOnRecordedImage(a->position, a->radius, r->colour, size*a->population, apop);
            }
        }
    }

    for(uint k_id : this->kilobots_ids) {
        drawCircleOnRecordedImage(complexityEnvironment.kilobots_positions.at(k_id), 5, complexityEnvironment.kilobots_colours.at(k_id), 5, "");
    }
}
