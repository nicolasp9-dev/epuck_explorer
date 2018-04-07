
#This is a template to build your own project with the e-puck2_main-processor folder as a library.
#Simply adapt the lines below to be able to compile

# Define project name here
PROJECT = epuck_explorer_project

#Define path to the e-puck2_main-processor folder
GLOBAL_PATH = lib/e-puck2_main-processor

#Source files to include
CSRC += ./main.c \
        ./modules/mod_basicIO.c \
        ./modules/mod_communication.c \
        ./modules/mod_exploration.c \
        ./modules/mod_image.c \
        ./modules/mod_mapping.c \
        ./modules/mod_motors.c \
        ./modules/mod_audio.c \
        ./modules/mod_sensors.c

#Header folders to include
INCDIR += modules/headers

#Jump to the main Makefile
include $(GLOBAL_PATH)/Makefile
