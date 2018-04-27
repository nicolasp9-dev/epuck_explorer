/*
 * File : mod_exploration.c
 * Project : e_puck_project
 * Description : Module in charge of exploration / autonomous displacements algorithms
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */

#include "mod_exploration.h"


// Our headers
#include "mod_mapping.h"
#include "mod_errors.h"
#include "mod_communication.h"
#include "mod_motors.h"
#include "mod_sensors.h"
#include "mod_basicIO.h"
#include <stdio.h>
#include "mod_check.h"

#include <ch.h>
#include "hal.h"

#define ACCELERATION_FACTOR         2
#define DEFAULT_TRANSLATION_SPEED   40 //mm/s
#define DEFAULT_ROTATION_SPEED      0.28 //rad/s

#define ARENA_WALL_DISTANCE                             66  // Between epuck and wall in the arena (in mm)
#define CALIBRATION_REF_TIME                            4000
#define ROTATION_ELMT_TIME                              50
#define TOLERATE_ERROR                                  4   // Error in mm

static thread_t * discoverThread;
static thread_t * explorationThread;
static thread_t * motorsControlThread;
// Semaphores
BSEMAPHORE_DECL (wipEndSignal_sem, true);
BSEMAPHORE_DECL (wipEndMovingSignal_sem, true);
BSEMAPHORE_DECL (isMessage, true);

/********************
 *  Private functions
 */

/**
 * @brief Change motors states and compute the new position of the robot
 *
 * @param[in] wheelSpeed       The wheels speed of the robot
 * @param[in] time             The time the robot was at the indicated speed
 */
void moveAndComputePositionWheelSpeedType(const wheelSpeed_t* wheelSpeed, int deltaTime);

/**
 * @brief Change motors states and compute the new position of the robot
 *
 * @param[in] robotSpeed       The robot style speed of the robot
 * @param[in] time             The time the robot was at the indicated speed
 */
void moveAndComputePositionRobotSpeedType(const robotSpeed_t* robotSpeed, int deltaTime);

/**
 * @brief Change motors states
 *
 * @param[in] wheelSpeed       The wheels speed of the robot
 * @param[in] time             The time the robot was at the indicated speed
 */
void moveWheelSpeedType(const wheelSpeed_t* wheelSpeed, int deltaTime);

/**
 * @brief Change motors state
 *
 * @param[in] robotSpeed       The robot style speed of the robot
 * @param[in] time             The time the robot was at the indicated speed
 */
void moveRobotSpeedType(const robotSpeed_t* robotSpeed, int deltaTime);

/**
 * @brief Store the current value of the front distance sensor with the actual position of the robot
 *
 * @param[in] measurement       A pointer to the storing space
 */
void storeFrontDistanceSensorValue(measurement_t* measurement);


void rotateAndMeasureWallsDistance(measurement_t* measurement, int number);

/**
 * @brief Move in a direction and send a signal when an object was found
 *
 * @param[in] measurement       A pointer to the storing space
 */
void moveInDirection(double theta);

wheelSpeed_t lastOrder;

void goTo(const robotPosition_t * newAbsolutePosition);

void changeAngleRelative(double angle);

/***************/


void signalEndOfWork(void){
    chBSemSignal(&wipEndSignal_sem);
}

void signalEndMovement(void){
    chBSemSignal(&wipEndMovingSignal_sem);
}

static THD_WORKING_AREA(changeMotorsStateAsync_wa, 1024);
static THD_FUNCTION(changeMotorsStateAsync, arg){
    (void) arg;
    static bool isMoving = false;
    static systime_t lastOrderTime;
    while(1){
        thread_t *tp = chMsgWait();
        const wheelSpeed_t *order = (const wheelSpeed_t *)chMsgGet(tp);

        if(isMoving){
            systime_t deltaTime = chVTGetSystemTime() - lastOrderTime;
            lastOrderTime = chVTGetSystemTime();
            mod_motors_changeStateWheelSpeedType(*order);
            mod_mapping_updatePositionWheelSpeedType(&lastOrder, ST2MS(deltaTime));
        }
        else{
            lastOrderTime = chVTGetSystemTime();
            mod_motors_changeStateWheelSpeedType(*order);
        }

        if(order->left == 0 && order->right==0){
            mod_com_writeMessage("Robot is not moving", 0);
            isMoving = false;
            signalEndMovement();
        }
        else{
            mod_com_writeMessage("Robot is moving", 0);
            isMoving = true;
            lastOrder = *order;
        }
        chMsgRelease(tp, MSG_OK);
    }
}

void waitForMovementEnd(void){
    chBSemWait(&wipEndMovingSignal_sem);
}

void changeMotorsState(wheelSpeed_t order){
    (void)chMsgSend(motorsControlThread, (msg_t) &order);
}

void stopMotors(void){
    changeMotorsState((wheelSpeed_t){0,0});
}

void moveAndComputePositionWheelSpeedType(const wheelSpeed_t* wheelSpeed, int deltaTime){
    
    changeMotorsState(*wheelSpeed);
    systime_t time = chVTGetSystemTime();
    chThdSleepUntilWindowed(time, time + MS2ST(deltaTime*1.0200));
    stopMotors();
    waitForMovementEnd();
}

void moveAndComputePositionRobotSpeedType(const robotSpeed_t* robotSpeed, int deltaTime){
    wheelSpeed_t wheelSpeed = mod_motors_convertRobotSpeedToWheelspeed(*robotSpeed);
    moveAndComputePositionWheelSpeedType(&wheelSpeed, deltaTime);
}

void moveAndComputePositionDistanceType(const robotDistance_t* robotDistance){

    changeAngleRelative(robotDistance->rotation[0]);
    moveAndComputePositionRobotSpeedType(&((robotSpeed_t){((robotDistance->translation< 0) ? -1 : 1 )*DEFAULT_TRANSLATION_SPEED, 0}), 1000*fabs(robotDistance->translation/DEFAULT_TRANSLATION_SPEED));
    changeAngleRelative(robotDistance->rotation[1]);
}



void moveWheelSpeedType(const wheelSpeed_t* wheelSpeed, int deltaTime){
    mod_motors_changeStateWheelSpeedType(*wheelSpeed);
    systime_t time = chVTGetSystemTime();
    chThdSleepUntilWindowed(time, time + MS2ST(deltaTime));
    mod_motors_stop();
}

void moveRobotSpeedType(const robotSpeed_t* robotSpeed, int deltaTime){
    wheelSpeed_t wheelSpeed = mod_motors_convertRobotSpeedToWheelspeed(*robotSpeed);
    moveWheelSpeedType(&wheelSpeed, deltaTime);
}


void storeFrontDistanceSensorValue(measurement_t* measurement){
    int value = mod_sensors_getValueTOF();
    *measurement = (measurement_t) {mod_mapping_getActualPosition(), value};
}



void rotateAndMeasureWallsDistance(measurement_t* measurement, int number){
    assert(measurement);
    int i;
    for(i=0; i < number; i++){
        chThdSleepMilliseconds(200);
        storeFrontDistanceSensorValue(&measurement[i]);
        changeAngleRelative(ANGLE_ELEMENT);
    }
}

void moveInDirection(double absoluteAngle){
    double relativeAngle = mod_mapping_getAngleForTranslation(absoluteAngle);
    moveAndComputePositionRobotSpeedType(&((robotSpeed_t){0, ((relativeAngle < 0) ? -1 : 1 ) * DEFAULT_ROTATION_SPEED}), 1000*fabs(relativeAngle)/DEFAULT_ROTATION_SPEED);
    changeMotorsState(mod_motors_convertRobotSpeedToWheelspeed((robotSpeed_t){DEFAULT_TRANSLATION_SPEED, 0}));
    mod_sensors_waitForObstacle();
     mod_sensors_need_objectDetection = false;
    stopMotors();
}

void goTo(const robotPosition_t * newAbsolutePosition){
    robotDistance_t toDo = mod_mapping_getRobotDisplacement(newAbsolutePosition);
    moveAndComputePositionDistanceType(&toDo);
}

void changeAngleRelative(double angle){
    moveAndComputePositionRobotSpeedType(&((robotSpeed_t){0, ((angle< 0) ? -1 : 1 )*DEFAULT_ROTATION_SPEED}), (double)1000*fabs(angle)/DEFAULT_ROTATION_SPEED);
}

actualEnvironement_t environmentObstacles;


void detectEnvironment(measurement_t* measurement, int numberOfMeasurements){

    mod_mapping_checkEnvironment(measurement,numberOfMeasurements, &environmentObstacles);
    

    
}

void scanInFront(void){
    measurement_t *measurement = malloc(NUMBER_OF_STEPS_FRONT*sizeof(measurement_t));
    changeAngleRelative(M_PI/4);
    int i;
    for(i=0; i < NUMBER_OF_STEPS_FRONT; i++){
        chThdSleepMilliseconds(150);
        storeFrontDistanceSensorValue(&measurement[i]);
        changeAngleRelative(ANGLE_ELEMENT_FRONT);
    }
    changeAngleRelative(-M_PI/4);
    detectEnvironment(measurement, NUMBER_OF_STEPS_FRONT);
}

static THD_WORKING_AREA(discover_wa, 1024);
static THD_FUNCTION(discover, arg){
    (void) arg;
    
    changeAngleRelative(2*M_PI);
    chThdSleepMilliseconds(2000);
    mod_mapping_resetCoordinates();
    
    measurement_t * measurement = malloc(NUMBER_OF_STEPS*sizeof(measurement_t));
    assert(measurement);
    rotateAndMeasureWallsDistance(measurement, NUMBER_OF_STEPS);
    mod_mapping_computeWallLocation(measurement,2);
    free(measurement);
    measurement = NULL;

    signalEndOfWork();
}


static THD_WORKING_AREA(exploration_wa, 1024);
static THD_FUNCTION(exploration, arg){
    (void) arg;
    changeAngleRelative(-M_PI/2);
    scanInFront();
    
    
    mod_com_writeMessage("Entering exploration thread", 3);
    
    signalEndOfWork();
}


/********************
 *  Public functions
 */


void mod_explo_initModule(void){
    mod_sensors_initSensors();
    mod_motors_init();
    mod_mapping_init();
    mod_sensors_initObjectDetection();
    motorsControlThread = chThdCreateStatic(changeMotorsStateAsync_wa, sizeof(changeMotorsStateAsync_wa), NORMALPRIO+10, changeMotorsStateAsync, NULL);
    //applyMotorThread = chThdCreateStatic(applyMotorBias_wa, sizeof(applyMotorBias_wa), NORMALPRIO+2, applyMotorBias, NULL);
    //robotDirection = chThdCreateStatic(robotDirectionBias_wa, sizeof(robotDirectionBias_wa), NORMALPRIO+2, robotDirectionBias, NULL);
    
}


void mod_explo_calibration(void){
    
    mod_sensors_calibrateIRSensors();
    /*mod_motors_initCalibration();
    
    // Calibration TOF sensor
    mod_sensors_calibrateFrontSensor(ARENA_WALL_DISTANCE);
    
    // Wait for obstacle insertion
    mod_basicIO_changeRobotState(WAITING);
    int i = 0;
    while(1){
        if(mod_sensors_getValueTOF() < (ARENA_WALL_DISTANCE-TOLERATE_ERROR)){
            if(i>=4){
                break;
            }
            i++;
        }
        else{
            i = 0;
        }
        chThdSleepMilliseconds(1000);
    }
    mod_basicIO_changeRobotState(WIP);
    
    // Calibration rotation constants
    const int toReach = mod_sensors_getValueTOF();
    wheelSpeed_t refRotation = {-200, 200};
    moveWheelSpeedType(&refRotation, CALIBRATION_REF_TIME);
    int totalTime = CALIBRATION_REF_TIME;
    
    while(1){
        moveWheelSpeedType(&refRotation, ROTATION_ELMT_TIME);
        totalTime += ROTATION_ELMT_TIME;
        if(mod_sensors_getValueTOF() < (toReach + TOLERATE_ERROR)){
            break;
        }
    }
    mod_motor_angleCalibration(refRotation, totalTime);
    
    while(1){
        if(mod_sensors_getValueTOF() > (ARENA_WALL_DISTANCE - TOLERATE_ERROR)){
            break;
        }
    };
    
    chThdSleepMilliseconds(300);
    // Callibration proximity sensors and motors constant
    wheelSpeed_t refTranslation = {200, 200};
    
    mod_motor_distanceCalibration(mod_sensors_getValueTOF(), refTranslation, 0);
    mod_sensors_calibrateIRSensors(mod_sensors_getValueTOF());
    moveWheelSpeedType(&refTranslation, CALIBRATION_REF_TIME);
    chThdSleepMilliseconds(300);
    
    mod_motor_distanceCalibration(mod_sensors_getValueTOF(), refTranslation, CALIBRATION_REF_TIME);
    mod_sensors_calibrateIRSensors(mod_sensors_getValueTOF());*/
}

void mod_explo_discoverTheAreaOnThread(void){
    discoverThread = chThdCreateStatic(discover_wa, sizeof(discover_wa), NORMALPRIO+9, discover, NULL);

}

void mod_explo_explorateTheAreaOnThread(){
    explorationThread = chThdCreateStatic(exploration_wa, sizeof(exploration_wa), NORMALPRIO+2, exploration, NULL);
}

void mod_explo_sendTheMap(void){
    mod_com_writeCommand(SEND_MAP);
}

void mod_explo_waitUntilEndOfWork(void){
    chBSemWait(&wipEndSignal_sem);
}


