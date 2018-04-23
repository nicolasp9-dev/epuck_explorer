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

#include <ch.h>
#include "hal.h"

#define ACCELERATION_FACTOR         2
#define DEFAULT_SPEED               10

#define ARENA_WALL_DISTANCE                             66// Between epuck and wall in the arena (in mm)
#define CALIBRATION_REF_TIME                            4000
#define ROTATION_ELMT_TIME                              50
#define TOLERATE_ERROR                                  4 // Error in mm

static thread_t * discoverThread;
static thread_t * explorationThread;
static thread_t * motorsControlThread;

// Semaphores
binary_semaphore_t wipEndSignal_sem;

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

void goTo(const robotPosition_t * newAbsolutePosition);

/***************/

static THD_WORKING_AREA(changeMotorsStateAsync_wa, 1024);
static THD_FUNCTION(changeMotorsStateAsync, arg){
    
    static bool isMoving = false;
    static systime_t lastOrderTime;
    static wheelSpeed_t lastOrder;
    while(1){
        thread_t *tp = chMsgWait();
        const wheelSpeed_t *order = (const wheelSpeed_t *)chMsgGet(tp);
        chMsgRelease(tp, MSG_OK);
        if(isMoving){
            systime_t deltaTime = chVTGetSystemTime() - lastOrderTime;
            lastOrderTime = chVTGetSystemTime();
            mod_motors_changeStateWheelSpeedType(*order);
            mod_mapping_updatePositionWheelSpeedType(&lastOrder, MS2ST(deltaTime));
        }
        else{
            lastOrderTime = chVTGetSystemTime();
            mod_motors_changeStateWheelSpeedType(*order);
        }

        if(order->left == 0 && order->right==0){
            isMoving = false;
        }
        else{
            isMoving = true;
            lastOrder = *order;
        }
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
    chThdSleepUntilWindowed(time, time + MS2ST(deltaTime));
    stopMotors();
}

void moveAndComputePositionRobotSpeedType(const robotSpeed_t* robotSpeed, int deltaTime){
    wheelSpeed_t wheelSpeed = mod_motors_convertRobotSpeedToWheelspeed(*robotSpeed);
    moveAndComputePositionWheelSpeedType(&wheelSpeed, deltaTime);
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
    int i;
    for(i=0; i < number; i++){
        storeFrontDistanceSensorValue(&(measurement[i]));
        moveAndComputePositionRobotSpeedType(&( (robotSpeed_t) {0,ANGLE_ELEMENT*ACCELERATION_FACTOR } ), MS_TO_S/ACCELERATION_FACTOR);
    }
}

void moveInDirection(float theta){
    
}

void goTo(const robotPosition_t * newAbsolutePosition){
    mod_mapping_getRobotDisplacement(newAbsolutePosition);
}

static THD_WORKING_AREA(exploration_wa, 1024);
static THD_FUNCTION(exploration, arg){
    while(1){
        
    }
}

static THD_WORKING_AREA(discover_wa, 1024);
static THD_FUNCTION(discover, arg){
    // RAZ Robot position
    mod_mapping_resetCoordinates();
    measurement_t measurement[NUMBER_OF_STEPS];
    
    rotateAndMeasureWallsDistance(measurement, NUMBER_OF_STEPS);
    mod_mapping_computeWallLocation(measurement,2);
    
    // Adapt the coordinate system with results
    
    // Move the robot near a wall until it finds another wall
    
    // Turn the robot and found the last wall
    
    // Go back to initial position (or stay here)
}

static THD_WORKING_AREA(obstacleCheck_wa, 1024);
static THD_FUNCTION(obstacleCheck, arg){
    while(1){
        int tab[8];
        chSemWait(&isObstacle_sem);
        mod_sensors_getAllProximityValues(tab);
        // Check for the obstacle specificity
        // if object => Take photo
        
        // Change  direction
    }
    
}

static THD_WORKING_AREA(robotDirectionBias_wa, 1024);
static THD_FUNCTION(robotDirectionBias, arg){
    int tab[8];
    mod_sensors_getAllProximityValues(tab);
    speedBias_t speedBias;
    
    static const int weight[8] = {0.9659,0.6427,0,-0.8660, -0.8660, 0,0.6427,0.9659};
    
    speedBias.BiasLeftSpeed = - weight[0]/tab[0] - weight[1]/tab[1] - weight[3]/tab[3] ;
    speedBias.BiasRightSpeed = - weight[7]/tab[7] - weight[6]/tab[6] - weight[4]/tab[4];
    
    //Msg the bias to speed
}

void signalEndOfWork(void){
    chSysLockFromISR();
    chSemSignalI(&wipEndSignal_sem);
    chSysUnlockFromISR();
}

/********************
 *  Public functions
 */


void mod_explo_initModule(void){
    mod_sensors_initSensors();
    mod_motors_init();
    mod_mapping_init();
    chBSemObjectInit(&wipEndSignal_sem, false);
    motorsControlThread = chThdCreateStatic(changeMotorsStateAsync_wa, sizeof(changeMotorsStateAsync_wa), NORMALPRIO+2, changeMotorsStateAsync, NULL);
    
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
    chSemWait(&wipEndSignal_sem);
}


