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
#define DEFAULT_TRANSLATION_SPEED    60 //mm/s
#define DEFAULT_ROTATION_SPEED      1 //rad/s

#define ARENA_WALL_DISTANCE                             66  // Between epuck and wall in the arena (in mm)
#define CALIBRATION_REF_TIME                            4000
#define ROTATION_ELMT_TIME                              50
#define TOLERATE_ERROR                                  4   // Error in mm

static thread_t * discoverThread;
static thread_t * explorationThread;
static thread_t * motorsControlThread;
static thread_t * applyMotorThread;
static thread_t * robotDirection;
// Semaphores
binary_semaphore_t wipEndSignal_sem;
binary_semaphore_t wipEndMovingSignal_sem;
binary_semaphore_t isMessage;

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
void moveInDirection(float theta);

wheelSpeed_t lastOrder;

void goTo(const robotPosition_t * newAbsolutePosition);

/***************/


void signalEndOfWork(void){
    chBSemSignal(&wipEndSignal_sem);
}

void signalEndMovement(void){
    chBSemSignal(&wipEndMovingSignal_sem);
}

static THD_WORKING_AREA(changeMotorsStateAsync_wa, 1024);
static THD_FUNCTION(changeMotorsStateAsync, arg){
    
    static bool isMoving = false;
    static systime_t lastOrderTime;
    while(1){
        thread_t *tp = chMsgWait();
        const wheelSpeed_t *order = (const wheelSpeed_t *)chMsgGet(tp);
        
        mod_com_writeMessage("Will change motor states", 0);

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
    chSemWait(&wipEndMovingSignal_sem);
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
    chThdSleepUntilWindowed(time, time + MS2ST(deltaTime));
    stopMotors();
}

void moveAndComputePositionRobotSpeedType(const robotSpeed_t* robotSpeed, int deltaTime){
    wheelSpeed_t wheelSpeed = mod_motors_convertRobotSpeedToWheelspeed(*robotSpeed);
    moveAndComputePositionWheelSpeedType(&wheelSpeed, deltaTime);
}

void moveAndComputePositionDistanceType(const robotDistance_t* robotDistance){
    char toSend[100];
    sprintf(toSend, "Rot : %f Trans : %d Rot : %f", 1000*robotDistance->rotation[0]/DEFAULT_ROTATION_SPEED, 1000*robotDistance->translation/DEFAULT_TRANSLATION_SPEED,1000*robotDistance->rotation[1]/DEFAULT_ROTATION_SPEED);
    mod_com_writeMessage(toSend, 10);
    
    moveAndComputePositionRobotSpeedType(&((robotSpeed_t){0, DEFAULT_ROTATION_SPEED}), 1000*robotDistance->rotation[0]/DEFAULT_ROTATION_SPEED);
    waitForMovementEnd();
    moveAndComputePositionRobotSpeedType(&((robotSpeed_t){DEFAULT_TRANSLATION_SPEED, 0}), 1000*robotDistance->translation/DEFAULT_TRANSLATION_SPEED);
    waitForMovementEnd();
    moveAndComputePositionRobotSpeedType(&((robotSpeed_t){0, DEFAULT_ROTATION_SPEED}), 1000*robotDistance->rotation[1]/DEFAULT_ROTATION_SPEED);
    waitForMovementEnd();
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
        chThdSleepMilliseconds(250);
        storeFrontDistanceSensorValue(&measurement[i]);
        moveAndComputePositionRobotSpeedType(&( (robotSpeed_t) {0,ANGLE_ELEMENT*ACCELERATION_FACTOR } ), MS_TO_S/ACCELERATION_FACTOR);
    }
}

void moveInDirection(float absoluteAngle){
    float relativeAngle = mod_mapping_getAngleForTranslation(absoluteAngle);
    moveAndComputePositionRobotSpeedType(&((robotSpeed_t){0, DEFAULT_ROTATION_SPEED}), 1000*relativeAngle/DEFAULT_ROTATION_SPEED);
    waitForMovementEnd();
    changeMotorsState(mod_motors_convertRobotSpeedToWheelspeed((robotSpeed_t){DEFAULT_TRANSLATION_SPEED, 0}));
    mod_sensors_waitForObstacle();
    stopMotors();
}

void goTo(const robotPosition_t * newAbsolutePosition){
    robotDistance_t toDo = mod_mapping_getRobotDisplacement(newAbsolutePosition);
    moveAndComputePositionDistanceType(&toDo);
}

static THD_WORKING_AREA(exploration_wa, 1024);
static THD_FUNCTION(exploration, arg){
    (void) arg;
    while(1){
        break;
    }
    signalEndOfWork();
}

static THD_WORKING_AREA(discover_wa, 1024);
static THD_FUNCTION(discover, arg){
    (void) arg;
    
    // RAZ Robot position
    mod_com_writeMessage("Entering discover thread", 3);
    
    mod_mapping_resetCoordinates();
    
    measurement_t * measurement = malloc(NUMBER_OF_STEPS*sizeof(measurement_t));
    
    rotateAndMeasureWallsDistance(measurement, NUMBER_OF_STEPS);
    mod_com_writeMessage("Finished rotation", 3);
    mod_mapping_computeWallLocation(measurement,2);
    free(measurement);
    measurement = NULL;
    
    mod_sensors_initObjectDetection();
    moveInDirection(.0);
    
    
    // Adapt the coordinate system with results
    
    // Move the robot near a wall until it finds another wall
    
    // Turn the robot and found the last wall
    
    // Go back to initial position (or stay here)
    signalEndOfWork();
    
}

// A mettre en route pour l'exploration
static THD_WORKING_AREA(obstacleCheck_wa, 1024);
static THD_FUNCTION(obstacleCheck, arg){
    (void) arg;
    while(1){
        //mod_com_writeMessage("Computation of obstacles", 3);
        int tab[8];
        mod_sensors_waitForObstacle();
        mod_sensors_getAllProximityValues(tab);
        // Check for the obstacle specificity
        // if object => Take photo
        
        // Change  direction
        chThdSleepMilliseconds(100);
    }
    
    
}

static THD_WORKING_AREA(robotDirectionBias_wa, 1024);
static THD_FUNCTION(robotDirectionBias, arg){
    (void) arg;
    while(1){
        //mod_com_writeMessage("Computation of direction bias", 3);
        int tab[8];
        mod_sensors_getAllProximityValues(tab);
        speedBias_t speedBias;
    
        static const int weight[8] = {0.9659,0.6427,0,-0.8660, -0.8660, 0,0.6427,0.9659};
    
        speedBias.BiasLeftSpeed = - weight[0]/tab[0] - weight[1]/tab[1] - weight[3]/tab[3] ;
        speedBias.BiasRightSpeed = - weight[7]/tab[7] - weight[6]/tab[6] - weight[4]/tab[4];
    
        (void)chMsgSend(applyMotorThread, (msg_t) &speedBias);
    
        chThdSleepMilliseconds(100);
    }
    //Msg the bias to speed
}

static THD_WORKING_AREA(applyMotorBias_wa, 1024);
static THD_FUNCTION(applyMotorBias, arg){
    while(1){
        thread_t *tp = chMsgWait();
        const speedBias_t * order = (const speedBias_t *)chMsgGet(tp);
        chMsgRelease(tp, MSG_OK);
        //changeMotorsState((wheelSpeed_t){(lastOrder.left+order->BiasLeftSpeed), (lastOrder.right+order->BiasRightSpeed)});
    }
}



/********************
 *  Public functions
 */


void mod_explo_initModule(void){
    mod_sensors_initSensors();
    mod_motors_init();
    mod_mapping_init();
    chBSemObjectInit(&wipEndSignal_sem, false);
    chBSemObjectInit(&isMessage, false);
    chBSemObjectInit(&wipEndMovingSignal_sem, false);
    motorsControlThread = chThdCreateStatic(changeMotorsStateAsync_wa, sizeof(changeMotorsStateAsync_wa), NORMALPRIO+3, changeMotorsStateAsync, NULL);
    //applyMotorThread = chThdCreateStatic(applyMotorBias_wa, sizeof(applyMotorBias_wa), NORMALPRIO+2, applyMotorBias, NULL);
    //robotDirection = chThdCreateStatic(robotDirectionBias_wa, sizeof(robotDirectionBias_wa), NORMALPRIO+2, robotDirectionBias, NULL);
    
}


void mod_explo_calibration(void){
    mod_sensors_initCalibration();
    mod_motors_initCalibration();
    
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
    mod_sensors_calibrateIRSensors(mod_sensors_getValueTOF());
}

void mod_explo_discoverTheAreaOnThread(void){
    discoverThread = chThdCreateStatic(discover_wa, sizeof(discover_wa), NORMALPRIO+2, discover, NULL);

}

void mod_explo_explorateTheAreaOnThread(exploration_t type){
    explorationThread = chThdCreateStatic(exploration_wa, sizeof(exploration_wa), NORMALPRIO+2, exploration, NULL);
}

void mod_explo_sendTheMap(void){
    
}

void mod_explo_waitUntilEndOfWork(void){
    chBSemWait(&wipEndSignal_sem);
}


