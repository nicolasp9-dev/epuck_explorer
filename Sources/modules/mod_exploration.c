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
#include "mod_image.h"
#include <stdio.h>
#include "mod_check.h"

#include <ch.h>
#include "hal.h"

#define ACCELERATION_FACTOR         2
#define DEFAULT_TRANSLATION_SPEED   40 //mm/s
#define DEFAULT_ROTATION_SPEED      0.33 //rad/s

#define ARENA_WALL_DISTANCE                             66  // Between epuck and wall in the arena (in mm)
#define CALIBRATION_REF_TIME                            4000
#define ROTATION_ELMT_TIME                              50
#define TOLERATE_ERROR                                  4   // Error in mm


static thread_t * discoverThread;
static thread_t * explorationThread;
static thread_t * motorsControlThread;


history_t history = {false , false};

// Semaphores
BSEMAPHORE_DECL (wipEndSignal_sem, true);
BSEMAPHORE_DECL (wipEndMovingSignal_sem, true);
BSEMAPHORE_DECL (isMessage, true);

wheelSpeed_t lastOrder;

/********************
 *  Private functions (Actions definitions in the funcitons body)
 */


// Movements

/**
 * @brief Send a message to the motor thread to change the motors with the send value
 *
 * @note The new position of the robot will be computed
 *
 * @param[in] order       The speed to apply to wheels
 */
void changeMotorsState(wheelSpeed_t order);

/**
 * @brief Send a message to the motor thread to change the motors with the send value
 *
 * @note The new position of the robot will be computed
 */
void stopMotors(void);


/**
 * @brief Move the robot at a defined speed for a certain time
 *
 * @note The new position of the robot will be computed
 *
 * @param[in] wheelSpeed       The wheels speed of the robot
 * @param[in] time             The time the robot was at the indicated speed
 */
void moveAndComputePositionWheelSpeedType(const wheelSpeed_t* wheelSpeed, int deltaTime);

/**
 * @brief Move the robot at a defined speed for a certain time
 *
 * @note The new position of the robot will be computed
 *
 * @param[in] robotSpeed       The robot style speed of the robot
 * @param[in] time             The time the robot was at the indicated speed
 */
void moveAndComputePositionRobotSpeedType(const robotSpeed_t* robotSpeed, int deltaTime);

/**
 * @brief Change motors states
 *
 * @note Does NOT compute the new robot position
 *
 * @param[in] wheelSpeed       The wheels speed of the robot
 * @param[in] time             The time the robot was at the indicated speed
 */
void moveWheelSpeedType(const wheelSpeed_t* wheelSpeed, int deltaTime);

/**
 * @brief Change motors state
 *
 * @note Does NOT compute the new robot position
 *
 * @param[in] robotSpeed       The robot style speed of the robot
 * @param[in] time             The time the robot was at the indicated speed
 */
void moveRobotSpeedType(const robotSpeed_t* robotSpeed, int deltaTime);


/**
 * @brief Do the movement that consist of a rotation and a translation
 *
 * @param[in] robotDistance       The movement to do (robot relatives)
 */
void moveAndComputePositionDistanceType(const robotDistance_t* robotDistance);


/**
 * @brief Move the robot in a certain direction without stop
 *
 * @param[in] absoluteAngle     The absolute angle to have
 */
void moveInAbsoluteDirection(float absoluteAngle);

/**
 * @brief Change the angle of the robot
 *
 * @note The new position of the robot will be computed
 *
 * @param[in] relativeAngle     The relative to robot angle to have
 */
void changeAngleRelative(float relativeAngle);

/**
 * @brief Change the angle of the robot
 *
 * @note The new position of the robot will be computed
 *
 * @param[in] absoluteAngle     The absolute angle to have
 */
void changeAngleAbsolute(float absoluteAngle);

/**
 * @brief Move the robot to a point
 *
 * @note No interruption if obstacle on the way !
 *
 * @param[in] newAbsolutePosition       The absolute position of the point where to go
 */
void goTo(const point_t * newAbsolutePosition);


// Communication between threads

/**
 * @brief Function to call when the major task is finished, it will unlock the main thread
*/
void signalEndOfWork(void);

/**
 * @brief Function to call at the end of a movement, it will unlock process that was waiting for the movement end
 */
void signalEndMovement(void);

/**
 * @brief Block the current thread until exploration work is finished
 */
void waitForMovementEnd(void);


// Measurements

/**
 * @brief Store the front sensor value in the pointed space
 *
 * @param[in] measurement       Pointer to an editable measurement
 */
void storeFrontDistanceSensorValue(measurement_t* measurement);

/**
 * @brief Do a complete rotation and store all measurement for point tracing
 *
 * @param[in] measurement       A pointer to a storing area
 * @param[in] number            The number of unit spaces in the measurement area
 */
void rotateAndMeasureWallsDistance(measurement_t* measurement, int number);




/***************/


// Movements

/**
 * @brief Function that manages motors and positions computation on a distinct thread
 *
 * @note    For more precision it uses real time to compute displacements
 *          To send new commands a message must be sent
 */
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


void moveWheelSpeedType(const wheelSpeed_t* wheelSpeed, int deltaTime){
    mod_com_writeMessage("WARNING : You're using a function that corrupts your coordinates system", 5);
    mod_motors_changeStateWheelSpeedType(*wheelSpeed);
    systime_t time = chVTGetSystemTime();
    chThdSleepUntilWindowed(time, time + MS2ST(deltaTime));
    mod_motors_stop();
}


void moveRobotSpeedType(const robotSpeed_t* robotSpeed, int deltaTime){
    mod_com_writeMessage("WARNING : You're using a function that corrupts your coordinates system", 5);
    wheelSpeed_t wheelSpeed = mod_motors_convertRobotSpeedToWheelspeed(*robotSpeed);
    moveWheelSpeedType(&wheelSpeed, deltaTime);
}


void moveAndComputePositionDistanceType(const robotDistance_t* robotDistance){
    changeAngleRelative(robotDistance->rotation);
    moveAndComputePositionRobotSpeedType(&((robotSpeed_t){((robotDistance->translation< 0) ? -1 : 1 )*DEFAULT_TRANSLATION_SPEED, 0}), 1000*fabs(robotDistance->translation/DEFAULT_TRANSLATION_SPEED));
}


void moveInAbsoluteDirection(float absoluteAngle){
    mod_com_writeMessage("WARNING : You're using a useless function", 5);
    float relativeAngle = mod_mapping_getRelativeAngle(absoluteAngle);
    moveAndComputePositionRobotSpeedType(&((robotSpeed_t){0, ((relativeAngle < 0) ? -1 : 1 ) * DEFAULT_ROTATION_SPEED}), 1000*fabs(relativeAngle)/DEFAULT_ROTATION_SPEED);
    changeMotorsState(mod_motors_convertRobotSpeedToWheelspeed((robotSpeed_t){DEFAULT_TRANSLATION_SPEED, 0}));
}


void changeAngleRelative(float relativeAngle){
    moveAndComputePositionRobotSpeedType(&((robotSpeed_t){0, ((relativeAngle< 0) ? -1 : 1 )*DEFAULT_ROTATION_SPEED}), (float)1000*fabs(relativeAngle)/DEFAULT_ROTATION_SPEED);
}


void changeAngleAbsolute(float absoluteAngle){
    changeAngleRelative(mod_mapping_getRelativeAngle(absoluteAngle));
}


void goTo(const point_t * newAbsolutePosition){
    robotDistance_t toDo = mod_mapping_getRobotDisplacement(newAbsolutePosition);
    moveAndComputePositionDistanceType(&toDo);
}


// Communication between threads

void signalEndOfWork(void){
    chBSemSignal(&wipEndSignal_sem);
}


void signalEndMovement(void){
    chBSemSignal(&wipEndMovingSignal_sem);
}


void waitForMovementEnd(void){
    chBSemWait(&wipEndMovingSignal_sem);
}

// Measurements

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

// FAIRE DEF DE CETTE FONCTION
void scanInFront(void){
    measurement_t *measurement = malloc(NUMBER_OF_STEPS_FRONT*sizeof(measurement_t));
    changeAngleRelative(-M_PI/8);
    for(int i=0; i < NUMBER_OF_STEPS_FRONT; i++){
        chThdSleepMilliseconds(150);
        storeFrontDistanceSensorValue(&measurement[i]);
        changeAngleRelative(ANGLE_ELEMENT_FRONT);
    }
    changeAngleRelative(M_PI/8);
    mod_mapping_checkEnvironment(measurement, NUMBER_OF_STEPS_FRONT);
    
    for(int i = 0; i < environment.numberOfnewObjects ;i++){
        robotDistance_t toDo = mod_mapping_computeDistanceForPicture(environment.newObjectsLocation[i]);
        moveAndComputePositionDistanceType(&toDo);
        mod_image_sendPicture(environment.newObjectsLocation[i].x, environment.newObjectsLocation[i].y);
        chThdSleepMilliseconds(1000);
        moveAndComputePositionRobotSpeedType(&((robotSpeed_t){((-toDo.translation< 0) ? -1 : 1 )*DEFAULT_TRANSLATION_SPEED, 0}), 1000*fabs(-toDo.translation/DEFAULT_TRANSLATION_SPEED));
        changeAngleRelative(-toDo.rotation);
    }
}

void scan360(void){
    measurement_t measurement;
    
    for(int i=0; i < NUMBER_OF_STEPS_SCAN360; i++){
        chThdSleepMilliseconds(150);
        storeFrontDistanceSensorValue(&measurement);
        point_t newObject = mod_mapping_checkEnvironmentRobotReferencial(&measurement, history.exploration);
        if(newObject.x == -1 && newObject.y == -1){
            changeAngleRelative(ANGLE_ELEMENT_SCAN360);
            continue;
        }
        int distance = mod_mapping_computeDistanceForPictureRobotReferencial(&measurement);
        moveAndComputePositionDistanceType(&((robotDistance_t){distance, 0}));
        mod_image_sendPicture(newObject.x, newObject.y);
        moveAndComputePositionDistanceType(&((robotDistance_t){-distance, 0}));
        changeAngleRelative(ANGLE_ELEMENT_SCAN360);
    }
}

// Actions on threads

/**
 * @brief Function that launches the discovering of the area
 */
static THD_WORKING_AREA(discover_wa, 1024);
static THD_FUNCTION(discover, arg){
    (void) arg;
    
    mod_mapping_resetCoordinates();
    
    measurement_t * measurement = malloc(NUMBER_OF_STEPS*sizeof(measurement_t));
    assert(measurement);
    rotateAndMeasureWallsDistance(measurement, NUMBER_OF_STEPS);
    mod_mapping_computeWallLocation(measurement);
    free(measurement);
    measurement = NULL;
    
    goTo(&((point_t){80, 80}));
    
    signalEndOfWork();
}

/**
 * @brief Function that launches the exploration of the area
 */
static THD_WORKING_AREA(exploration_wa, 1024);
static THD_FUNCTION(exploration, arg){
    (void) arg;
    mod_com_writeMessage("Entering exploration thread", 3);
    scan360();
    
    signalEndOfWork();
}



/********************
 *  Public functions (Definition in header)
 */


void mod_explo_initModule(void){
    mod_sensors_initSensors();
    mod_motors_init();
    mod_mapping_init();
    mod_img_init();
    
    //mod_sensors_initObjectDetection();
    
    motorsControlThread = chThdCreateStatic(changeMotorsStateAsync_wa, sizeof(changeMotorsStateAsync_wa), NORMALPRIO+8, changeMotorsStateAsync, NULL);
}


void mod_explo_discoverTheAreaOnThread(void){
    discoverThread = chThdCreateStatic(discover_wa, sizeof(discover_wa), NORMALPRIO+7, discover, NULL);
    
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
