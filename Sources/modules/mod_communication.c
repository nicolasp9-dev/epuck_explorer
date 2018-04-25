/*
 * File : mod_communication.c
 * Project : e_puck_project
 * Description : Module in charge of bluetooth communcations with distant device
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */


#include "headers/mod_communication.h"

// Standard headers
#include <stdio.h>
#include <string.h>

// Epuck/ChibiOS headers
#include <ch.h>
#include <hal.h>
#include <chprintf.h>


// Our headers
#include "mod_check.h"
#include "mod_secure_conv.h"



#define SEPARATOR_SIZE              2
#define UNSIGNED_INT_IN_CHAR_SIZE   6
#define NULL_CHAR_SIZE              1
#define SERIAL_BIT_RATE             115200

#define DISPLAY_LEVEL               1

/********************
 *  Private functions
 */

/**
 * @brief Load parameters and start serial drivers
 */
static void serial_start(void)
{
    static SerialConfig ser_cfg = {
        SERIAL_BIT_RATE,
        0,
        0,
        0,
    };
    
    sdStart(&SD3, &ser_cfg); // UART3. Connected to the second com port of the programmer
}


/********************
 *  Public functions (Informations in header)
 */

void mod_com_initModule(){
    
    serial_start();
    
}

void mod_com_writeDatas(char* type, char* toWrite, size_t toWriteSize){
    
    void* toSend = NULL;
    
    // Prepare the size of datas to introduce it in the header
    if(toWriteSize == 0) toWriteSize = strlen(toWrite);
    char sizeText[sizeof(char)*(UNSIGNED_INT_IN_CHAR_SIZE + NULL_CHAR_SIZE)];
    sprintf(sizeText, "%5d", size_t2int(toWriteSize));
    
    size_t sizeOfType = strlen(type);

    const char separator[SEPARATOR_SIZE] = "||";
    
    size_t headerSize = strlen(sizeText) + SEPARATOR_SIZE +
                                sizeOfType +
                                SEPARATOR_SIZE;
    
    toSend = malloc(headerSize*sizeof(char));
    assert (toSend != NULL);

    // Prepare the header
    void * toSendPointer = toSend;
    memcpy(toSendPointer, sizeText, strlen(sizeText));
    toSendPointer += strlen(sizeText);
    memcpy(toSendPointer, separator, (size_t) SEPARATOR_SIZE);
    toSendPointer += SEPARATOR_SIZE;
    memcpy(toSendPointer, type, sizeOfType);
    toSendPointer += sizeOfType;
    memcpy(toSendPointer, separator, (size_t) SEPARATOR_SIZE);
    
    size_t i;
    
    // Write the header first
    for(i=0; i<headerSize;i++)
        chprintf((BaseSequentialStream *)&SD3, "%c",((uint8_t*) toSend)[i]);
    
    // Write content of the message
    for(i=0; i<toWriteSize;i++)
        chprintf((BaseSequentialStream *)&SD3, "%02x",toWrite[i]);
    
    free(toSend);
    toSend = NULL;
}


void mod_com_writeMessage(char* message, int level){
    
    if(level < DISPLAY_LEVEL){
        return;
    }
    strcat(message,"\n");
    
    void* toSend = NULL;
    
    // Prepare the size of datas to introduce it in the header
    size_t toWriteSize = strlen(message);
    char sizeText[sizeof(char)*(UNSIGNED_INT_IN_CHAR_SIZE + NULL_CHAR_SIZE)];
    sprintf(sizeText, "%5d", size_t2int(toWriteSize));
    
    size_t sizeOfType = strlen("Message");
    const char separator[SEPARATOR_SIZE] = "||";
    
    size_t totalSize =  strlen(sizeText) + SEPARATOR_SIZE +
                        sizeOfType + SEPARATOR_SIZE + strlen(message) ;
    
    toSend = malloc(totalSize*sizeof(char));
    assert (toSend != NULL);
    
    // Prepare the header
    void * toSendPointer = toSend;
    memcpy(toSendPointer, sizeText, strlen(sizeText));
    toSendPointer += strlen(sizeText);
    memcpy(toSendPointer, separator, (size_t) SEPARATOR_SIZE);
    toSendPointer += SEPARATOR_SIZE;
    memcpy(toSendPointer, "Message", sizeOfType);
    toSendPointer += sizeOfType;
    memcpy(toSendPointer, separator, (size_t) SEPARATOR_SIZE);
    toSendPointer += SEPARATOR_SIZE;
    memcpy(toSendPointer, message, (size_t) strlen(message));
    
    size_t i;
    
    // Write the header first
    for(i=0; i<totalSize;i++)
        chprintf((BaseSequentialStream *)&SD3, "%c",((uint8_t*) toSend)[i]);
    
    free(toSend);
    toSend = NULL;
}

void mod_com_writeCommand(cmd_t order){
    
    void* toSend = NULL;
    char orderString[30];
    char sizeText[sizeof(char)*(UNSIGNED_INT_IN_CHAR_SIZE + NULL_CHAR_SIZE)];
    sprintf(sizeText, "%5d", 0);
    switch (order) {
        case SEND_MAP:
            strcpy(orderString, "Send map\n");
            break;
    }
    
    size_t sizeOfType = strlen(orderString);
    const char separator[SEPARATOR_SIZE] = "||";
    
    size_t totalSize =  strlen(sizeText) + SEPARATOR_SIZE +
    sizeOfType;
    
    toSend = malloc(totalSize*sizeof(char));
    assert (toSend != NULL);
    
    // Prepare the header
    void * toSendPointer = toSend;
    memcpy(toSendPointer, sizeText, strlen(sizeText));
    toSendPointer += strlen(sizeText);
    memcpy(toSendPointer, separator, (size_t) SEPARATOR_SIZE);
    toSendPointer += SEPARATOR_SIZE;
    memcpy(toSendPointer, orderString, sizeOfType);
    
    size_t i;
    
    // Write the header first
    for(i=0; i<totalSize;i++)
        chprintf((BaseSequentialStream *)&SD3, "%c",((uint8_t*) toSend)[i]);
    free(toSend);
    toSend = NULL;
}
