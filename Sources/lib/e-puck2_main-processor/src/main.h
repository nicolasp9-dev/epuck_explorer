#ifndef MAIN_H
#define MAIN_H

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

/** Robot wide IPC bus. */
extern messagebus_t bus;

/** Robot parameters. */
extern parameter_namespace_t parameter_root;


/**
 * @brief Initialize modules and connexions
 */
void initSystem(void);

/**
 * @brief Launch the exploration of the area
 *
 * @note It blocks the thread until the end of the action
 */
void exploration(void);

/**
 * @brief Launch the discovering of the area
 *
 * @note It blocks the thread until the end of the action
 */
void discovering(void);

/**
 * @brief Launch the action that send the map to the user
 *
 * @note It blocks the thread until the end of the action
 */
void sendMap(void);

/**
 * @brief Launch the action that play StarWars song
 *
 * @note It blocks the thread until the end of the action
 */
void song(void);

/**
 * @brief Launch the motor and sensors calibration (in the arena)
 */
void calibrateSystem(void);

/**
 * @brief Launch the action to do based on audio feedback
 *
 * @note The robot emit a different audio signal if the action is not possible (dependent on a previous one)
 */
void actionChoice(void);

/**
 * @brief Kernal panic guard
*/
void __stack_chk_fail(void);

#ifdef __cplusplus
extern "C" {
#endif


    
    
    
#ifdef __cplusplus
}
#endif

#endif
