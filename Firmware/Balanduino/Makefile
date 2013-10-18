# You should only have to change these values
PORT = /dev/tty.usbserial-DN0031EV

# You should set the path to your Arduino application if it is not the default one
# The default for Linux and Solaris is:
#ARD_HOME = /opt/Arduino
# The default for Mac is:
#ARD_HOME = /Applications/Arduino.app

# Uncomment to activate serial debugging
#EXTRA_FLAGS += -D DEBUG_USB_HOST

# Leave these alone
THIRD_PARTY_HARDWARE = ../hardware/Balanduino/avr
BOARD = balanduino
BOARD_SUB = balanduino.menu.cpu.atmega1284p
MON_SPEED = 115200

LIB_DIRS += ../libraries/USB_Host_Shield_2.0
LIB_DIRS += ../libraries/KalmanFilter
LIB_DIRS += $(ARD_HOME)/libraries/Wire/src
LIB_DIRS += $(ARD_HOME)/libraries/Wire/arch/avr
LIB_DIRS += $(ARD_HOME)/libraries/Wire/arch/avr/utility

include Arduino_Makefile_master/_Makefile.master