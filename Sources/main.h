#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

/********************
 *  Private functions to the main
 */
    
/**
 * @brief Function that initialize the system
 */
void initSystem(void);

/**
 * @brief Function that launches exploration
 */
void exploration(void);

/**
 * @brief Function that launches discovering
 */
void discovering(void);

/**
 * @brief Function that launches song play
 */
void song(void);

/**
 * @brief Function that send the command ask the python programm to save the map
 */
void sendMap(void);

/**
 * @brief Function that launches system calibration
 */
void calibrateSystem(void);

/**
 * @brief Function that perform the choice of action
 */
void actionChoice(void);

    
#ifdef __cplusplus
}
#endif

#endif
